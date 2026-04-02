package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.StrategicTWAgentMemory.KnownTarget;
import tileworld.agent.StrategicTWAgentMemory.TargetType;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class SimpleTWAgent extends TWAgent {

    private static final boolean LARGE_MAP = Parameters.xDimension >= 80 || Parameters.yDimension >= 80;
    private static final boolean TEAM_MODE = Parameters.agentCount > 1;
    private static final int MAX_CARRIED_TILES = 3;
    private static final int EXPLORATION_STRIDE = Parameters.defaultSensorRange * 2 + 1;
    private static final int FUEL_BUFFER = LARGE_MAP ? 70 : 36;
    private static final int PRE_FUEL_TARGET_RADIUS = LARGE_MAP ? 4 : 6;
    private static final int PRE_FUEL_BUDGET_BUFFER = LARGE_MAP ? 65 : 45;
    private static final int PATH_SAFETY_MARGIN = LARGE_MAP ? 18 : 10;
    private static final double MAX_MEMORY_TARGET_AGE = LARGE_MAP ? 12.0 : 24.0;
    private static final double TILE_AGE_WEIGHT = 0.30;
    private static final double HOLE_AGE_WEIGHT = 0.30;
    private static final double TILE_PRIORITY_BONUS = 1.2;
    private static final double HOLE_PRIORITY_BONUS = 1.1;
    private static final int EXTRA_TILE_DETOUR = 3;
    private static final int FORCE_DELIVERY_AT_TILES = 2;
    private static final int TARGET_REFINEMENT_LIMIT = 3;
    private static final int TARGET_CLUSTER_RADIUS = Parameters.defaultSensorRange + 2;
    private static final double TILE_TRAVEL_DETOUR_WEIGHT = 0.70;
    private static final double HOLE_TRAVEL_DETOUR_WEIGHT = 0.45;
    private static final double TARGET_SLACK_WEIGHT = 0.12;
    private static final double TILE_CLUSTER_BONUS = 0.20;
    private static final double HOLE_CLUSTER_BONUS = 0.30;
    private static final double TILE_SECTOR_PENALTY_WEIGHT = TEAM_MODE ? (LARGE_MAP ? 0.80 : 0.55) : 0.0;
    private static final double HOLE_SECTOR_PENALTY_WEIGHT = TEAM_MODE ? 0.18 : 0.0;
    private static final Object TEAM_STATE_LOCK = new Object();
    private static Int2D sharedFuelStation;

    private enum GoalMode {
        TILE,
        HOLE,
        FUEL,
        EXPLORE
    }

    private final int agentIndex;
    private final int sectorStartX;
    private final int sectorEndX;
    private final String name;
    private final StrategicTWAgentMemory strategicMemory;
    private final AstarPathGenerator pathGenerator;

    private GoalMode currentMode;
    private KnownTarget currentTarget;
    private TWPath currentPath;
    private Int2D currentExploreTarget;
    private List<Int2D> explorationWaypoints;
    private int explorationIndex;

    public SimpleTWAgent(int agentIndex, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.agentIndex = Math.max(0, agentIndex);
        this.name = name;
        this.strategicMemory = new StrategicTWAgentMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
        this.memory = this.strategicMemory;
        this.pathGenerator = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());
        this.currentMode = GoalMode.EXPLORE;
        this.explorationIndex = 0;
        int[] sector = computeSectorBounds(env.getxDimension(), this.agentIndex);
        this.sectorStartX = sector[0];
        this.sectorEndX = sector[1];
    }

    public static SimpleTWAgent createAgent(int index, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        return new SimpleTWAgent(index - 1, "agent" + index, xpos, ypos, env, fuelLevel);
    }

    public static void resetSharedState() {
        synchronized (TEAM_STATE_LOCK) {
            sharedFuelStation = null;
        }
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void communicate() {
        KnownTarget fuelStation = strategicMemory.getFuelStation();
        if (fuelStation != null) {
            publishFuelStation(fuelStation);
        } else {
            syncSharedFuelStation();
        }
    }

    @Override
    protected TWThought think() {
        if (shouldRefuelHere()) {
            clearCurrentGoal();
            return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        TWEntity currentCell = getCurrentCellObject();
        if (currentCell instanceof TWHole && hasTile()) {
            clearCurrentGoal();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (currentCell instanceof TWTile && carriedTiles.size() < MAX_CARRIED_TILES) {
            clearCurrentGoal();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        return new TWThought(TWAction.MOVE, chooseDirection());
    }

    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case PICKUP:
                    TWEntity currentCell = getCurrentCellObject();
                    if (currentCell instanceof TWTile) {
                        pickUpTile((TWTile) currentCell);
                        strategicMemory.invalidateTarget(TargetType.TILE, getX(), getY());
                        clearCurrentGoal();
                    }
                    break;
                case PUTDOWN:
                    currentCell = getCurrentCellObject();
                    if (currentCell instanceof TWHole) {
                        putTileInHole((TWHole) currentCell);
                        strategicMemory.invalidateTarget(TargetType.HOLE, getX(), getY());
                        clearCurrentGoal();
                    }
                    break;
                case REFUEL:
                    refuel();
                    clearCurrentGoal();
                    break;
                case MOVE:
                default:
                    this.move(thought.getDirection());
                    break;
            }
        } catch (CellBlockedException ex) {
            currentPath = null;
        }
    }

    private TWDirection chooseDirection() {
        syncSharedFuelStation();
        KnownTarget fuelStation = strategicMemory.getFuelStation();
        KnownTarget tileTarget = selectTileTarget();
        KnownTarget holeTarget = selectHoleTarget();

        if (fuelStation == null) {
            if (hasTile() && holeTarget != null && isNearby(holeTarget, PRE_FUEL_TARGET_RADIUS)) {
                return followTargetGoal(GoalMode.HOLE, holeTarget);
            }
            if (carriedTiles.size() < MAX_CARRIED_TILES && tileTarget != null && isNearby(tileTarget, PRE_FUEL_TARGET_RADIUS)) {
                return followTargetGoal(GoalMode.TILE, tileTarget);
            }
            return followExplorationGoal();
        }

        if (shouldHeadForFuel(fuelStation, holeTarget)) {
            return followTargetGoal(GoalMode.FUEL, fuelStation);
        }
        if (shouldDeliverTiles(holeTarget, tileTarget)) {
            return followTargetGoal(GoalMode.HOLE, holeTarget);
        }
        if (tileTarget != null && carriedTiles.size() < MAX_CARRIED_TILES) {
            return followTargetGoal(GoalMode.TILE, tileTarget);
        }
        if (hasTile() && holeTarget != null) {
            return followTargetGoal(GoalMode.HOLE, holeTarget);
        }
        return followExplorationGoal();
    }

    private boolean shouldRefuelHere() {
        return this.getEnvironment().inFuelStation(this) && this.getFuelLevel() < Parameters.defaultFuelLevel;
    }

    private boolean shouldHeadForFuel(KnownTarget fuelStation, KnownTarget holeTarget) {
        if (fuelStation == null) {
            return false;
        }

        int distanceToFuel = travelDistanceTo(fuelStation);
        int reserve = FUEL_BUFFER + (currentMode == GoalMode.EXPLORE ? EXPLORATION_STRIDE / 2 : 0);
        if (hasTile() && holeTarget != null) {
            int distanceViaHole = travelDistanceTo(holeTarget)
                    + estimatedPathDistance(holeTarget.getX(), holeTarget.getY(), fuelStation.getX(), fuelStation.getY());
            return getFuelLevel() <= distanceViaHole + reserve;
        }

        return getFuelLevel() <= distanceToFuel + reserve;
    }

    private boolean shouldDeliverTiles(KnownTarget holeTarget, KnownTarget tileTarget) {
        if (!hasTile() || holeTarget == null) {
            return false;
        }

        int holeTravelDistance = travelDistanceTo(holeTarget);
        if (targetSlack(holeTarget, holeTravelDistance) <= EXPLORATION_STRIDE / 2.0) {
            return true;
        }
        if (carriedTiles.size() >= MAX_CARRIED_TILES) {
            return true;
        }
        if (tileTarget == null) {
            return true;
        }

        int tileTravelDistance = travelDistanceTo(tileTarget);
        int tilePairDistance = nearestKnownHoleDistance(tileTarget);
        if (carriedTiles.size() >= FORCE_DELIVERY_AT_TILES) {
            return holeTravelDistance <= tileTravelDistance + 1
                    || tilePairDistance > EXPLORATION_STRIDE
                    || targetSlack(tileTarget, tileTravelDistance) <= Parameters.defaultSensorRange;
        }

        return holeTravelDistance + EXTRA_TILE_DETOUR <= tileTravelDistance
                || tilePairDistance > EXPLORATION_STRIDE
                || targetSlack(holeTarget, holeTravelDistance) <= EXPLORATION_STRIDE;
    }

    private KnownTarget selectTileTarget() {
        return selectBestTarget(TargetType.TILE);
    }

    private KnownTarget selectHoleTarget() {
        return selectBestTarget(TargetType.HOLE);
    }

    private KnownTarget selectBestTarget(TargetType targetType) {
        KnownTarget fuelStation = strategicMemory.getFuelStation();
        List<KnownTarget> shortlistedTargets = new ArrayList<KnownTarget>(TARGET_REFINEMENT_LIMIT);
        List<Double> shortlistedScores = new ArrayList<Double>(TARGET_REFINEMENT_LIMIT);

        for (KnownTarget candidate : strategicMemory.getKnownTargets(targetType)) {
            if (!isTargetViable(candidate, fuelStation)) {
                continue;
            }
            if (!sameTarget(candidate, currentTarget) && !isCurrentOrRecent(candidate)) {
                continue;
            }

            double score = targetType == TargetType.TILE ? scoreTile(candidate) : scoreHole(candidate);
            if (sameTarget(candidate, currentTarget)) {
                score -= 0.75;
            }
            insertShortlistedCandidate(shortlistedTargets, shortlistedScores, candidate, score);
        }

        KnownTarget bestTarget = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (int i = 0; i < shortlistedTargets.size(); i++) {
            KnownTarget candidate = shortlistedTargets.get(i);
            double score = refineTargetScore(candidate, shortlistedScores.get(i));
            if (bestTarget == null || score < bestScore) {
                bestTarget = candidate;
                bestScore = score;
            }
        }

        return bestTarget;
    }

    private double scoreTile(KnownTarget tile) {
        double score = distanceTo(tile);
        score += 0.6 * nearestKnownHoleDistance(tile);
        score += TILE_AGE_WEIGHT * strategicMemory.getObservationAge(tile);
        score -= TILE_PRIORITY_BONUS * Math.max(1, MAX_CARRIED_TILES - carriedTiles.size());
        score += sectorPenalty(tile);
        return score;
    }

    private double scoreHole(KnownTarget hole) {
        double score = distanceTo(hole) - carriedTiles.size();
        score += HOLE_AGE_WEIGHT * strategicMemory.getObservationAge(hole);
        score -= HOLE_PRIORITY_BONUS * Math.max(1, carriedTiles.size());
        score += sectorPenalty(hole);
        return score;
    }

    private void insertShortlistedCandidate(List<KnownTarget> shortlistedTargets, List<Double> shortlistedScores,
                                            KnownTarget candidate, double score) {
        int insertAt = 0;
        while (insertAt < shortlistedScores.size() && shortlistedScores.get(insertAt) <= score) {
            insertAt++;
        }

        if (insertAt >= TARGET_REFINEMENT_LIMIT) {
            return;
        }

        shortlistedTargets.add(insertAt, candidate);
        shortlistedScores.add(insertAt, score);
        if (shortlistedTargets.size() > TARGET_REFINEMENT_LIMIT) {
            shortlistedTargets.remove(TARGET_REFINEMENT_LIMIT);
            shortlistedScores.remove(TARGET_REFINEMENT_LIMIT);
        }
    }

    private double refineTargetScore(KnownTarget target, double score) {
        int directDistance = distanceTo(target);
        int travelDistance = travelDistanceTo(target);
        double detour = Math.max(0, travelDistance - directDistance);
        double slack = targetSlack(target, travelDistance);

        if (target.getType() == TargetType.TILE) {
            score += detour * TILE_TRAVEL_DETOUR_WEIGHT;
            score -= Math.min(EXPLORATION_STRIDE, slack) * TARGET_SLACK_WEIGHT;
            score -= countNearbyTargets(TargetType.TILE, target, TARGET_CLUSTER_RADIUS) * TILE_CLUSTER_BONUS;
            return score;
        }

        score += detour * HOLE_TRAVEL_DETOUR_WEIGHT;
        score -= Math.min(EXPLORATION_STRIDE, slack) * TARGET_SLACK_WEIGHT;
        if (carriedTiles.size() > 1) {
            score -= countNearbyTargets(TargetType.HOLE, target, TARGET_CLUSTER_RADIUS) * HOLE_CLUSTER_BONUS;
        }
        return score;
    }

    private boolean isTargetViable(KnownTarget target, KnownTarget fuelStation) {
        if (target == null) {
            return false;
        }
        if (target.getExpiresAt() <= currentTime()) {
            return false;
        }
        int lowerBoundDistance = distanceTo(target);
        if (target.getType() != TargetType.FUEL_STATION && target.getExpiresAt() < currentTime() + lowerBoundDistance + 1) {
            return false;
        }
        if (target.getType() != TargetType.FUEL_STATION
                && target.getExpiresAt() < currentTime() + lowerBoundDistance + PATH_SAFETY_MARGIN + 1
                && target.getExpiresAt() < currentTime() + travelDistanceTo(target) + 1) {
            return false;
        }
        if (!hasFuelBudgetFor(target, fuelStation)) {
            return false;
        }

        if (!isWithinSensorRange(target.getX(), target.getY())) {
            return true;
        }

        Object observed = this.getEnvironment().getObjectGrid().get(target.getX(), target.getY());
        switch (target.getType()) {
            case TILE:
                return observed instanceof TWTile;
            case HOLE:
                return observed instanceof TWHole;
            case FUEL_STATION:
                return observed instanceof TWFuelStation;
            default:
                return false;
        }
    }

    private boolean hasFuelBudgetFor(KnownTarget target, KnownTarget fuelStation) {
        if (target == null || target.getType() == TargetType.FUEL_STATION) {
            return true;
        }

        int toTarget = distanceTo(target);
        if (fuelStation == null) {
            return getFuelLevel() > toTarget + PRE_FUEL_BUDGET_BUFFER;
        }

        int optimisticTotal = toTarget
                + manhattanDistance(target.getX(), target.getY(), fuelStation.getX(), fuelStation.getY())
                + FUEL_BUFFER;
        if (getFuelLevel() > optimisticTotal + PATH_SAFETY_MARGIN) {
            return true;
        }

        int toFuelAfterTarget = estimatedPathDistance(target.getX(), target.getY(), fuelStation.getX(), fuelStation.getY());
        return getFuelLevel() > travelDistanceTo(target) + toFuelAfterTarget + FUEL_BUFFER;
    }

    private boolean isCurrentOrRecent(KnownTarget target) {
        return isWithinSensorRange(target.getX(), target.getY())
                || strategicMemory.getObservationAge(target) <= MAX_MEMORY_TARGET_AGE;
    }

    private double targetSlack(KnownTarget target, int travelDistance) {
        if (target == null || target.getExpiresAt() == Double.POSITIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        }
        return target.getExpiresAt() - currentTime() - travelDistance - 1;
    }

    private boolean isNearby(KnownTarget target, int radius) {
        return target != null && distanceTo(target) <= radius;
    }

    private int distanceTo(KnownTarget target) {
        return manhattanDistance(getX(), getY(), target.getX(), target.getY());
    }

    private int travelDistanceTo(KnownTarget target) {
        return estimatedPathDistance(getX(), getY(), target.getX(), target.getY());
    }

    private int manhattanDistance(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private int estimatedPathDistance(int startX, int startY, int targetX, int targetY) {
        if (startX == targetX && startY == targetY) {
            return 0;
        }

        TWPath path = pathGenerator.findPath(startX, startY, targetX, targetY);
        if (path != null) {
            return path.getpath().size();
        }

        return manhattanDistance(startX, startY, targetX, targetY) + EXPLORATION_STRIDE;
    }

    private double currentTime() {
        return this.getEnvironment().schedule.getTime();
    }

    private boolean isWithinSensorRange(int targetX, int targetY) {
        return Math.max(Math.abs(targetX - getX()), Math.abs(targetY - getY())) <= Parameters.defaultSensorRange;
    }

    private TWDirection followTargetGoal(GoalMode mode, KnownTarget target) {
        if (target == null) {
            return followExplorationGoal();
        }

        if (currentMode != mode || !sameTarget(currentTarget, target)) {
            currentMode = mode;
            currentTarget = target;
            currentExploreTarget = null;
            currentPath = null;
        }
        return nextDirectionTo(target.getX(), target.getY());
    }

    private boolean sameTarget(KnownTarget left, KnownTarget right) {
        return left != null
                && right != null
                && left.getType() == right.getType()
                && left.getX() == right.getX()
                && left.getY() == right.getY();
    }

    private TWDirection followExplorationGoal() {
        ensureExplorationWaypoints();
        currentMode = GoalMode.EXPLORE;
        currentTarget = null;

        if (explorationWaypoints.isEmpty()) {
            return TWDirection.Z;
        }

        for (int attempt = 0; attempt < explorationWaypoints.size(); attempt++) {
            Int2D waypoint = explorationWaypoints.get(explorationIndex);
            if (waypoint.x == getX() && waypoint.y == getY()) {
                advanceExplorationIndex();
                continue;
            }

            if (!waypoint.equals(currentExploreTarget)) {
                currentExploreTarget = waypoint;
                currentPath = null;
            }

            TWDirection direction = nextDirectionTo(waypoint.x, waypoint.y);
            if (direction != TWDirection.Z) {
                return direction;
            }

            advanceExplorationIndex();
            currentExploreTarget = null;
            currentPath = null;
        }

        return chooseFallbackDirection(currentExploreTarget);
    }

    private TWDirection nextDirectionTo(int targetX, int targetY) {
        if (targetX == getX() && targetY == getY()) {
            return TWDirection.Z;
        }

        if (currentPath == null || !currentPath.hasNext()) {
            currentPath = pathGenerator.findPath(getX(), getY(), targetX, targetY);
        }

        if (currentPath != null && currentPath.hasNext()) {
            TWPathStep step = currentPath.popNext();
            return step.getDirection();
        }

        return chooseFallbackDirection(new Int2D(targetX, targetY));
    }

    private TWDirection chooseFallbackDirection(Int2D target) {
        List<TWDirection> directions = new ArrayList<TWDirection>(4);
        if (target != null) {
            if (target.x > getX()) {
                directions.add(TWDirection.E);
            } else if (target.x < getX()) {
                directions.add(TWDirection.W);
            }
            if (target.y > getY()) {
                directions.add(TWDirection.S);
            } else if (target.y < getY()) {
                directions.add(TWDirection.N);
            }
        }

        addMissingDirections(directions);
        for (TWDirection direction : directions) {
            if (canMove(direction)) {
                currentPath = null;
                return direction;
            }
        }
        return TWDirection.Z;
    }

    private void addMissingDirections(List<TWDirection> directions) {
        for (TWDirection direction : new TWDirection[]{TWDirection.E, TWDirection.W, TWDirection.N, TWDirection.S}) {
            if (!directions.contains(direction)) {
                directions.add(direction);
            }
        }
    }

    private boolean canMove(TWDirection direction) {
        int nextX = getX() + direction.dx;
        int nextY = getY() + direction.dy;
        return this.getEnvironment().isInBounds(nextX, nextY) && !this.getEnvironment().isCellBlocked(nextX, nextY);
    }

    private TWEntity getCurrentCellObject() {
        return (TWEntity) this.getEnvironment().getObjectGrid().get(getX(), getY());
    }

    private void clearCurrentGoal() {
        currentTarget = null;
        currentPath = null;
        currentExploreTarget = null;
        currentMode = GoalMode.EXPLORE;
    }

    private void ensureExplorationWaypoints() {
        if (explorationWaypoints != null && !explorationWaypoints.isEmpty()) {
            return;
        }

        explorationWaypoints = new ArrayList<Int2D>();
        buildSweepWaypoints();
        explorationIndex = explorationWaypoints.isEmpty() ? 0 : findClosestWaypointIndex(explorationWaypoints);
        currentExploreTarget = null;
        currentPath = null;
    }

    private void buildSweepWaypoints() {
        int minX = sectorStartX;
        int maxX = sectorEndX;
        int maxY = this.getEnvironment().getyDimension() - 1;
        boolean topToBottom = true;

        for (int x = minX; x <= maxX; x += EXPLORATION_STRIDE) {
            explorationWaypoints.add(new Int2D(x, topToBottom ? 0 : maxY));
            explorationWaypoints.add(new Int2D(x, topToBottom ? maxY : 0));
            topToBottom = !topToBottom;
        }

        if (explorationWaypoints.isEmpty() || explorationWaypoints.get(explorationWaypoints.size() - 1).x != maxX) {
            explorationWaypoints.add(new Int2D(maxX, topToBottom ? 0 : maxY));
            explorationWaypoints.add(new Int2D(maxX, topToBottom ? maxY : 0));
        }
    }

    private int findClosestWaypointIndex(List<Int2D> waypoints) {
        int bestIndex = 0;
        double bestDistance = Double.MAX_VALUE;

        for (int i = 0; i < waypoints.size(); i++) {
            Int2D waypoint = waypoints.get(i);
            double distance = Math.abs(waypoint.x - getX()) + Math.abs(waypoint.y - getY());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    private int nearestKnownHoleDistance(KnownTarget tile) {
        int bestDistance = EXPLORATION_STRIDE + Parameters.defaultSensorRange;
        boolean foundHole = false;
        for (KnownTarget hole : strategicMemory.getKnownTargets(TargetType.HOLE)) {
            if (!isCurrentOrRecent(hole)) {
                continue;
            }
            int distance = manhattanDistance(tile.getX(), tile.getY(), hole.getX(), hole.getY());
            if (targetSlack(hole, distance) <= 0) {
                continue;
            }
            bestDistance = Math.min(bestDistance, distance);
            foundHole = true;
        }
        return foundHole ? bestDistance : EXPLORATION_STRIDE + Parameters.defaultSensorRange;
    }

    private void advanceExplorationIndex() {
        if (!explorationWaypoints.isEmpty()) {
            explorationIndex = (explorationIndex + 1) % explorationWaypoints.size();
        }
    }

    private void publishFuelStation(KnownTarget fuelStation) {
        if (fuelStation == null) {
            return;
        }
        synchronized (TEAM_STATE_LOCK) {
            sharedFuelStation = new Int2D(fuelStation.getX(), fuelStation.getY());
        }
    }

    private void syncSharedFuelStation() {
        Int2D teamFuel;
        synchronized (TEAM_STATE_LOCK) {
            teamFuel = sharedFuelStation;
        }
        if (teamFuel != null && strategicMemory.getFuelStation() == null) {
            strategicMemory.rememberFuelStation(teamFuel.x, teamFuel.y);
        }
    }

    private double sectorPenalty(KnownTarget target) {
        if (!TEAM_MODE || target == null) {
            return 0.0;
        }
        double weight = target.getType() == TargetType.HOLE ? HOLE_SECTOR_PENALTY_WEIGHT : TILE_SECTOR_PENALTY_WEIGHT;
        if (target.getType() == TargetType.TILE && carriedTiles.size() >= FORCE_DELIVERY_AT_TILES) {
            weight *= 1.25;
        }
        return distanceOutsideSector(target.getX()) * weight;
    }

    private int countNearbyTargets(TargetType type, KnownTarget center, int radius) {
        int count = 0;
        for (KnownTarget candidate : strategicMemory.getKnownTargets(type)) {
            if (candidate == center || !isCurrentOrRecent(candidate)) {
                continue;
            }
            if (manhattanDistance(center.getX(), center.getY(), candidate.getX(), candidate.getY()) <= radius) {
                count++;
            }
        }
        return count;
    }

    private int distanceOutsideSector(int x) {
        if (x < sectorStartX) {
            return sectorStartX - x;
        }
        if (x > sectorEndX) {
            return x - sectorEndX;
        }
        return 0;
    }

    private int[] computeSectorBounds(int width, int index) {
        if (!TEAM_MODE) {
            return new int[]{0, width - 1};
        }

        int teamSize = Math.max(1, Parameters.agentCount);
        int start = (index * width) / teamSize;
        int end = (((index + 1) * width) / teamSize) - 1;
        if (index >= teamSize - 1) {
            end = width - 1;
        }
        if (end < start) {
            end = start;
        }
        return new int[]{start, end};
    }
}
