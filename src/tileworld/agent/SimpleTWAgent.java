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

    private static final int MAX_CARRIED_TILES = 3;
    private static final int EXPLORATION_STRIDE = Parameters.defaultSensorRange * 2 + 1;
    private static final int FUEL_BUFFER = 36;
    private static final int PRE_FUEL_TARGET_RADIUS = 6;
    private static final double MAX_MEMORY_TARGET_AGE = 20.0;
    private static final double TILE_AGE_WEIGHT = 0.30;
    private static final double HOLE_AGE_WEIGHT = 0.30;
    private static final double TILE_PRIORITY_BONUS = 1.2;
    private static final double HOLE_PRIORITY_BONUS = 1.1;
    private static final int EXTRA_TILE_DETOUR = 3;
    private static final int FORCE_DELIVERY_AT_TILES = 2;

    private enum GoalMode {
        TILE,
        HOLE,
        FUEL,
        EXPLORE
    }

    private final String name;
    private final StrategicTWAgentMemory strategicMemory;
    private final AstarPathGenerator pathGenerator;

    private GoalMode currentMode;
    private KnownTarget currentTarget;
    private TWPath currentPath;
    private Int2D currentExploreTarget;
    private List<Int2D> explorationWaypoints;
    private int explorationIndex;

    public SimpleTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.strategicMemory = new StrategicTWAgentMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
        this.memory = this.strategicMemory;
        this.pathGenerator = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());
        this.currentMode = GoalMode.EXPLORE;
        this.explorationIndex = 0;
    }

    public static SimpleTWAgent createAgent(int index, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        return new SimpleTWAgent("agent" + index, xpos, ypos, env, fuelLevel);
    }

    @Override
    public String getName() {
        return name;
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

        int distanceToFuel = distanceTo(fuelStation);
        int reserve = FUEL_BUFFER + (currentMode == GoalMode.EXPLORE ? EXPLORATION_STRIDE / 2 : 0);
        if (hasTile() && holeTarget != null) {
            int distanceViaHole = distanceTo(holeTarget)
                    + manhattanDistance(holeTarget.getX(), holeTarget.getY(), fuelStation.getX(), fuelStation.getY());
            return getFuelLevel() <= distanceViaHole + reserve;
        }

        return getFuelLevel() <= distanceToFuel + reserve;
    }

    private boolean shouldDeliverTiles(KnownTarget holeTarget, KnownTarget tileTarget) {
        if (!hasTile() || holeTarget == null) {
            return false;
        }
        if (carriedTiles.size() >= MAX_CARRIED_TILES || carriedTiles.size() >= FORCE_DELIVERY_AT_TILES) {
            return true;
        }
        if (tileTarget == null) {
            return true;
        }
        return distanceTo(holeTarget) <= distanceTo(tileTarget) + EXTRA_TILE_DETOUR;
    }

    private KnownTarget selectTileTarget() {
        return selectBestTarget(TargetType.TILE);
    }

    private KnownTarget selectHoleTarget() {
        return selectBestTarget(TargetType.HOLE);
    }

    private KnownTarget selectBestTarget(TargetType targetType) {
        KnownTarget bestTarget = null;
        double bestScore = Double.POSITIVE_INFINITY;
        KnownTarget fuelStation = strategicMemory.getFuelStation();

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
        return score;
    }

    private double scoreHole(KnownTarget hole) {
        double score = distanceTo(hole) - carriedTiles.size();
        score += HOLE_AGE_WEIGHT * strategicMemory.getObservationAge(hole);
        score -= HOLE_PRIORITY_BONUS * Math.max(1, carriedTiles.size());
        return score;
    }

    private boolean isTargetViable(KnownTarget target, KnownTarget fuelStation) {
        if (target == null) {
            return false;
        }
        if (target.getExpiresAt() <= currentTime()) {
            return false;
        }
        if (target.getType() != TargetType.FUEL_STATION && target.getExpiresAt() < currentTime() + distanceTo(target) + 1) {
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
            return getFuelLevel() > toTarget + FUEL_BUFFER;
        }

        int toFuelAfterTarget = manhattanDistance(target.getX(), target.getY(), fuelStation.getX(), fuelStation.getY());
        return getFuelLevel() > toTarget + toFuelAfterTarget + FUEL_BUFFER;
    }

    private boolean isCurrentOrRecent(KnownTarget target) {
        return isWithinSensorRange(target.getX(), target.getY())
                || strategicMemory.getObservationAge(target) <= MAX_MEMORY_TARGET_AGE;
    }

    private boolean isNearby(KnownTarget target, int radius) {
        return target != null && distanceTo(target) <= radius;
    }

    private int distanceTo(KnownTarget target) {
        return manhattanDistance(getX(), getY(), target.getX(), target.getY());
    }

    private int manhattanDistance(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
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
        int maxX = this.getEnvironment().getxDimension() - 1;
        int maxY = this.getEnvironment().getyDimension() - 1;
        boolean topToBottom = true;

        for (int x = 0; x <= maxX; x += EXPLORATION_STRIDE) {
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
        int bestDistance = EXPLORATION_STRIDE;
        boolean foundHole = false;
        for (KnownTarget hole : strategicMemory.getKnownTargets(TargetType.HOLE)) {
            if (!isCurrentOrRecent(hole)) {
                continue;
            }
            bestDistance = Math.min(bestDistance,
                    manhattanDistance(tile.getX(), tile.getY(), hole.getX(), hole.getY()));
            foundHole = true;
        }
        return foundHole ? bestDistance : EXPLORATION_STRIDE;
    }

    private void advanceExplorationIndex() {
        if (!explorationWaypoints.isEmpty()) {
            explorationIndex = (explorationIndex + 1) % explorationWaypoints.size();
        }
    }
}
