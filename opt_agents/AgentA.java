package tileworld.agent;

import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;

import java.util.HashMap;
import java.util.Map;

public class AgentA extends TWAgent {

    private static final int FUEL_REFUEL_THRESHOLD = 150;
    private static final int FUEL_EMERGENCY_BUFFER = 10;
    private static final int MAX_DETOUR            = 6;
    private static final int OBS_EXPIRY            = 80;
    private static final int CLAIM_EXPIRY          = 25;

    private final String name;

    private final int secXMin, secXMax, secYMin, secYMax;
    private final int secCX, secCY;

    private final AstarPathGenerator pathGen;
    private TWPath currentPath;
    private int goalX = -1, goalY = -1;

    
    private int     stationX     = -1;
    private int     stationY     = -1;
    private boolean stationKnown = false;

    private int planHoleX = -1, planHoleY = -1;

    private int[][] holeQueue   = new int[3][2];
    private int     holeQueueSz = 0, holeQueueIdx = 0;

    private int stuckCount = 0, lastX = -1, lastY = -1;

    private int     exploreCol = 0;
    private boolean goingDown  = true;
    private int     exploreRow = 0;
    private boolean goingRight = true;
    private final boolean sweepByRow;  

    private final Map<String, Double> teamTiles  = new HashMap<>();
    private final Map<String, Double> teamHoles  = new HashMap<>();
    private final Map<String, String> claimOwner = new HashMap<>();
    private final Map<String, Double> claimTime  = new HashMap<>();
    
    
    private static final int BLACKLIST_EXPIRY = 50;
    private final Map<String, Double> unreachable = new HashMap<>();

    public AgentA(String name,
                  int secXMin, int secXMax,
                  int secYMin, int secYMax,
                  int xpos, int ypos,
                  TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name    = name;
        this.secXMin = secXMin;  this.secXMax = secXMax;
        this.secYMin = secYMin;  this.secYMax = secYMax;
        this.secCX   = (secXMin + secXMax) / 2;
        this.secCY   = (secYMin + secYMax) / 2;
        this.pathGen = new AstarPathGenerator(env, this,
                env.getxDimension() * env.getyDimension());
        
        int sr0 = tileworld.Parameters.defaultSensorRange;
        this.exploreCol = secXMin / (sr0 * 2);
        this.exploreRow = secYMin / (sr0 * 2);
        
        int secW = secXMax - secXMin + 1, secH = secYMax - secYMin + 1;
        int colSteps = ((secW + sr0*2-1)/(sr0*2)) * secH;
        int rowSteps = ((secH + sr0*2-1)/(sr0*2)) * secW;
        this.sweepByRow = rowSteps < colSteps;
        System.out.println("[" + name + "] spawned at (" + xpos + "," + ypos + ")"
                + " sector x=" + secXMin + ".." + secXMax
                + " y=" + secYMin + ".." + secYMax
                + " centre=(" + secCX + "," + secCY + ")"
                );
    }

    
    @Override
    public void communicate() {
        int sr = tileworld.Parameters.defaultSensorRange;
        int ax = getX(), ay = getY();
        int w  = getEnvironment().getxDimension();
        int h  = getEnvironment().getyDimension();
        double now = getEnvironment().schedule.getTime();

        
        if (stationKnown)
            getEnvironment().receiveMessage(
                    new TeamMessage(name, TeamMessage.FUEL, stationX, stationY, "0"));
        if (!stationKnown) scanForStation(getX(), getY());

        for (int x = Math.max(0, ax-sr); x <= Math.min(w-1, ax+sr); x++)
            for (int y = Math.max(0, ay-sr); y <= Math.min(h-1, ay+sr); y++) {
                Object obj = getEnvironment().getObjectGrid().get(x, y);
                if (obj instanceof TWTile)
                    getEnvironment().receiveMessage(new TeamMessage(
                            name, TeamMessage.TILE, x, y, String.valueOf((int)now)));
                else if (obj instanceof TWHole)
                    getEnvironment().receiveMessage(new TeamMessage(
                            name, TeamMessage.HOLE, x, y, String.valueOf((int)now)));
            }

        if (goalX >= 0 && goalY >= 0)
            getEnvironment().receiveMessage(
                    new TeamMessage(name, TeamMessage.CLAIM, goalX, goalY, "target"));
    }

    
    @Override
    protected TWThought think() {
        int ax = getX(), ay = getY();
        double now = getEnvironment().schedule.getTime();

        readMessages(now);
        if (!stationKnown) scanForStation(ax, ay);

        
        if ((int)now % 500 == 0 || (int)now == tileworld.Parameters.endTime - 1)
            System.out.println("[" + name + "] step=" + (int)now
                    + " score=" + this.score + " fuel=" + (int)fuelLevel
                    + " pos=(" + ax + "," + ay + ")");

        boolean onStation = stationKnown && (ax == stationX) && (ay == stationY);
        double  distStn   = stationKnown
                ? manhattanDist(ax, ay, stationX, stationY) : Double.MAX_VALUE;

        
        if (ax == lastX && ay == lastY) stuckCount++;
        else { stuckCount = 0; lastX = ax; lastY = ay; }
        if (stuckCount >= 3) {
            stuckCount = 0; currentPath = null; goalX = -1; goalY = -1;
            planHoleX = -1; planHoleY = -1;
            holeQueueSz = 0; holeQueueIdx = 0;
            return new TWThought(TWAction.MOVE, anyOpenDir());
        }

        
        if (goalX >= 0 && ax == goalX && ay == goalY) {
            if (getTileAt(ax,ay) == null && getHoleAt(ax,ay) == null)
                memory.removeAgentPercept(goalX, goalY);
            releaseClaim(goalX, goalY);
            goalX = -1; goalY = -1; currentPath = null;
        }

        
        if (onStation && fuelLevel < FUEL_REFUEL_THRESHOLD)
            return new TWThought(TWAction.REFUEL, TWDirection.Z);

        
        if (stationKnown && fuelLevel <= distStn + FUEL_EMERGENCY_BUFFER) {
            planHoleX = -1; planHoleY = -1;
            holeQueueSz = 0; holeQueueIdx = 0;
            goalX = -1; goalY = -1; currentPath = null;
            return headTo(stationX, stationY);
        }

        
        
        if (!stationKnown) {
            
            if (hasTile()) {
                TWHole h = getHoleAt(ax, ay);
                if (h != null) { currentPath = null; return new TWThought(TWAction.PUTDOWN, TWDirection.Z); }
            }
            
            if (carriedTiles.size() < 3) {
                TWTile t = getTileAt(ax, ay);
                if (t != null) { currentPath = null; return new TWThought(TWAction.PICKUP, TWDirection.Z); }
            }
            
            if (fuelLevel < 80) return new TWThought(TWAction.MOVE, TWDirection.Z);
            return preStationSweep();
        }

        
        if (hasTile()) {
            TWHole hole = getHoleAt(ax, ay);
            if (hole != null) {
                if (holeQueueIdx < holeQueueSz
                        && holeQueue[holeQueueIdx][0] == ax
                        && holeQueue[holeQueueIdx][1] == ay) holeQueueIdx++;
                if (ax == planHoleX && ay == planHoleY) { planHoleX=-1; planHoleY=-1; }
                currentPath = null;
                return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
            }
        }

        
        if (carriedTiles.size() < 3) {
            TWTile tile = getTileAt(ax, ay);
            if (tile != null) { currentPath = null; return new TWThought(TWAction.PICKUP, TWDirection.Z); }
        }

        
        
        
        if (stationKnown && goalX == stationX && goalY == stationY) {
            int sr = tileworld.Parameters.defaultSensorRange;
            if (hasTile()) {
                
                for (int x = Math.max(0,ax-2); x <= Math.min(getEnvironment().getxDimension()-1,ax+2); x++)
                    for (int y = Math.max(0,ay-2); y <= Math.min(getEnvironment().getyDimension()-1,ay+2); y++) {
                        if (Math.abs(x-ax)+Math.abs(y-ay) > 2) continue;
                        Object obj = getEnvironment().getObjectGrid().get(x, y);
                        if (obj instanceof TWHole) {
                            
                            double curDist = manhattanDist(ax, ay, stationX, stationY);
                            double newDist = manhattanDist(x, y, stationX, stationY);
                            if (newDist <= curDist + 2)
                                return setGoalMove(x, y);
                        }
                    }
            } else if (carriedTiles.size() < 3) {
                
                for (int x = Math.max(0,ax-2); x <= Math.min(getEnvironment().getxDimension()-1,ax+2); x++)
                    for (int y = Math.max(0,ay-2); y <= Math.min(getEnvironment().getyDimension()-1,ay+2); y++) {
                        if (Math.abs(x-ax)+Math.abs(y-ay) > 2) continue;
                        Object obj = getEnvironment().getObjectGrid().get(x, y);
                        if (obj instanceof TWTile) {
                            double curDist = manhattanDist(ax, ay, stationX, stationY);
                            double newDist = manhattanDist(x, y, stationX, stationY);
                            if (newDist <= curDist + 2)
                                return setGoalMove(x, y);
                        }
                    }
            }
        }

        
        if (hasTile()) {
            if (holeQueueSz == 0) buildHoleQueue(ax, ay, now);

            while (holeQueueIdx < holeQueueSz &&
                    !(getEnvironment().getObjectGrid().get(
                            holeQueue[holeQueueIdx][0],
                            holeQueue[holeQueueIdx][1]) instanceof TWHole))
                holeQueueIdx++;

            if (holeQueueIdx < holeQueueSz) {
                int hx = holeQueue[holeQueueIdx][0], hy = holeQueue[holeQueueIdx][1];
                if (carriedTiles.size() < 3) {
                    TWTile op = findOpportunisticTile(ax, ay, hx, hy);
                    if (op != null) return setGoalMove(op.getX(), op.getY());
                }
                return setGoalMove(hx, hy);
            }

            holeQueueSz = 0; holeQueueIdx = 0;
            if (planHoleX >= 0) {
                if (getEnvironment().getObjectGrid().get(planHoleX, planHoleY) instanceof TWHole)
                    return setGoalMove(planHoleX, planHoleY);
                planHoleX = -1; planHoleY = -1;
            }
            TWHole bh = findBestHole(ax, ay, now);
            if (bh != null) return setGoalMove(bh.getX(), bh.getY());
        }

        
        
        
        
        if (carriedTiles.size() < 3) {
            if (goalX < 0) {
                int[] pair = findBestPair(ax, ay, now);
                if (pair != null) {
                    planHoleX = pair[2]; planHoleY = pair[3];
                    return setGoalMove(pair[0], pair[1]);
                }
            } else {
                
                if (!isTileKnown(goalX, goalY, now) && !isHoleKnown(goalX, goalY, now)) {
                    releaseClaim(goalX, goalY);
                    goalX = -1; goalY = -1; currentPath = null;
                    
                } else {
                    return setGoalMove(goalX, goalY);
                }
            }
        }

        
        
        
        if (!inSector(ax, ay)) {
            return setGoalMove(secCX, secCY);
        }

        
        
        
        if (stationKnown) {
            int refuelThreshold = Math.max((int)(distStn * 1.2 + 30),
                    (int)(distStn + FUEL_EMERGENCY_BUFFER + 5));
            if (fuelLevel < refuelThreshold)
                return headTo(stationX, stationY);
        }

        
        currentPath = null; goalX = -1; goalY = -1;
        return explore();
    }

    
    @Override
    protected void act(TWThought t) {
        switch (t.getAction()) {
            case MOVE:
                try { move(t.getDirection()); }
                catch (CellBlockedException e) { currentPath = null; goalX = -1; goalY = -1; }
                break;
            case PICKUP:
                TWTile tile = getTileAt(getX(), getY());
                if (tile != null) { pickUpTile(tile); memory.removeObject(tile); }
                break;
            case PUTDOWN:
                TWHole hole = getHoleAt(getX(), getY());
                if (hole != null) { putTileInHole(hole); memory.removeObject(hole); }
                break;
            case REFUEL: refuel(); break;
        }
    }

    
    private void readMessages(double now) {
        for (Message raw : getEnvironment().getMessages()) {
            if (raw.getFrom().equals(name)) continue;
            TeamMessage msg = TeamMessage.parse(raw);
            if (msg == null) continue;
            switch (msg.getType()) {
                case TeamMessage.TILE:
                    teamTiles.merge(key(msg.getPx(), msg.getPy()),
                            parseDouble(msg.getExtra(), now), Math::max);
                    break;
                case TeamMessage.HOLE:
                    teamHoles.merge(key(msg.getPx(), msg.getPy()),
                            parseDouble(msg.getExtra(), now), Math::max);
                    break;
                case TeamMessage.FUEL:
                    if (!stationKnown) {
                        stationX = msg.getPx(); stationY = msg.getPy();
                        stationKnown = true;
                        System.out.println("[" + name + "] learned station at ("
                                + stationX + "," + stationY + ") from " + msg.getFrom());
                    }
                    break;
                case TeamMessage.CLAIM:
                    claimOwner.put(key(msg.getPx(), msg.getPy()), msg.getFrom());
                    claimTime.put(key(msg.getPx(), msg.getPy()), now);
                    break;
            }
        }
    }

    
    private int[] findBestPair(int ax, int ay, double now) {
        int w = getEnvironment().getxDimension(), h = getEnvironment().getyDimension();
        int[] best = null; double bestU = -1;

        for (int tx = 0; tx < w; tx++) for (int ty = 0; ty < h; ty++) {
            if (!inSector(tx, ty)) continue;
            if (!isTileKnown(tx, ty, now)) continue;
            if (isClaimedByOther(tx, ty, now)) continue;
            if (isUnreachable(tx, ty, now)) continue;
            double dAT = manhattanDist(ax, ay, tx, ty);

            int bHx = -1, bHy = -1; double bHC = Double.MAX_VALUE;
            for (int hx = 0; hx < w; hx++) for (int hy = 0; hy < h; hy++) {
                if (!inSector(hx, hy)) continue;
                if (!isHoleKnown(hx, hy, now)) continue;
                if (isUnreachable(hx, hy, now)) continue;
                double dTH  = manhattanDist(tx, ty, hx, hy);
                if (stationKnown) {
                    double fuel = dAT + dTH + manhattanDist(hx, hy, stationX, stationY)
                            + FUEL_EMERGENCY_BUFFER;
                    if (fuelLevel < fuel) continue;
                }
                if (dTH < bHC) { bHC = dTH; bHx = hx; bHy = hy; }
            }
            if (bHC == Double.MAX_VALUE) continue;
            double u = 1.0 / (dAT + bHC);
            if (u > bestU) { bestU = u; best = new int[]{tx, ty, bHx, bHy}; }
        }
        if (best != null) return best;

        
        double bd = Double.MAX_VALUE; int[] bt = null;
        for (int tx = 0; tx < w; tx++) for (int ty = 0; ty < h; ty++) {
            if (!inSector(tx, ty)) continue;
            if (!isTileKnown(tx, ty, now)) continue;
            if (isClaimedByOther(tx, ty, now)) continue;
            double d = manhattanDist(ax, ay, tx, ty);
            if (d < bd) { bd = d; bt = new int[]{tx, ty, -1, -1}; }
        }
        return bt;
    }

    private TWHole findBestHole(int ax, int ay, double now) {
        int w = getEnvironment().getxDimension(), h = getEnvironment().getyDimension();
        TWHole best = null; double bd = Double.MAX_VALUE;
        for (int x = 0; x < w; x++) for (int y = 0; y < h; y++) {
            if (!inSector(x, y)) continue;
            if (!isHoleKnown(x, y, now)) continue;
            if (isClaimedByOther(x, y, now)) continue;
            if (isUnreachable(x, y, now)) continue;
            double d = manhattanDist(ax, ay, x, y);
            if (stationKnown) {
                double fuel = d + manhattanDist(x, y, stationX, stationY) + FUEL_EMERGENCY_BUFFER;
                if (fuelLevel < fuel) continue;
            }
            if (d < bd) {
                bd = d;
                Object o = getEnvironment().getObjectGrid().get(x, y);
                if (o instanceof TWHole) best = (TWHole) o;
            }
        }
        return best;
    }

    private TWTile findOpportunisticTile(int ax, int ay, int hx, int hy) {
        int w = getEnvironment().getxDimension(), h = getEnvironment().getyDimension();
        double direct = manhattanDist(ax, ay, hx, hy);
        TWTile best = null; double bd = Double.MAX_VALUE;
        for (int x = 0; x < w; x++) for (int y = 0; y < h; y++) {
            if (!inSector(x, y)) continue;
            Object obj = memory.getMemoryGrid().get(x, y);
            if (!(obj instanceof TWTile)) continue;
            if (getEnvironment().getObjectGrid().get(x, y) == null) {
                memory.removeAgentPercept(x, y); continue;
            }
            double toTile = manhattanDist(ax, ay, x, y);
            double toHole = manhattanDist(x, y, hx, hy);
            double detour = toTile + toHole - direct;
            if (detour <= MAX_DETOUR && toTile < direct && detour < bd) {
                bd = detour; best = (TWTile) obj;
            }
        }
        return best;
    }

    
    private void buildHoleQueue(int ax, int ay, double now) {
        holeQueueSz = 0; holeQueueIdx = 0;
        int w = getEnvironment().getxDimension(), h = getEnvironment().getyDimension();
        boolean[] used = new boolean[w * h];
        int cx = ax, cy = ay;
        for (int slot = 0; slot < 3; slot++) {
            int bx = -1, by = -1; double bd = Double.MAX_VALUE;
            for (int x = 0; x < w; x++) for (int y = 0; y < h; y++) {
                if (used[x*h+y]) continue;
                if (!inSector(x, y)) continue;
                if (!isHoleKnown(x, y, now)) continue;
                if (isClaimedByOther(x, y, now)) continue;
                if (stationKnown) {
                    double dStn = manhattanDist(x, y, stationX, stationY);
                    if (fuelLevel < manhattanDist(ax,ay,x,y)+dStn+FUEL_EMERGENCY_BUFFER) continue;
                }
                double d = manhattanDist(cx, cy, x, y);
                if (d < bd) { bd = d; bx = x; by = y; }
            }
            if (bx < 0) break;
            holeQueue[holeQueueSz][0] = bx; holeQueue[holeQueueSz][1] = by;
            holeQueueSz++; used[bx*h+by] = true; cx = bx; cy = by;
        }
    }

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    private int circuitLayer  = 0;
    private int circuitCorner = 0;
    private TWPath circuitPath = null;
    private int circuitGoalX = -1, circuitGoalY = -1;

    private TWThought preStationSweep() {
        int sr = tileworld.Parameters.defaultSensorRange;
        int ax = getX(), ay = getY();

        
        int[] target = circuitTarget(circuitLayer, circuitCorner, sr);

        
        if (target == null) {
            circuitLayer = 0; circuitCorner = 0; circuitPath = null;
            target = circuitTarget(0, 0, sr);
            if (target == null) return new TWThought(TWAction.MOVE, anyOpenDir());
        }

        
        if (ax == target[0] && ay == target[1]) {
            circuitCorner++;
            if (circuitCorner > 3) {
                circuitCorner = 0;
                circuitLayer++;
                if (circuitTarget(circuitLayer, 0, sr) == null) circuitLayer = 0;
            }
            circuitPath = null;
            target = circuitTarget(circuitLayer, circuitCorner, sr);
            if (target == null) return new TWThought(TWAction.MOVE, anyOpenDir());
        }

        int tx = target[0], ty = target[1];

        
        
        if (circuitGoalX != tx || circuitGoalY != ty) {
            circuitPath = null;
            circuitGoalX = tx; circuitGoalY = ty;
        }
        if (circuitPath == null || !circuitPath.hasNext())
            circuitPath = pathGen.findPath(ax, ay, tx, ty);

        if (circuitPath != null && circuitPath.hasNext()) {
            return new TWThought(TWAction.MOVE, circuitPath.popNext().getDirection());
        }

        
        
        circuitCorner++;
        if (circuitCorner > 3) {
            circuitCorner = 0;
            circuitLayer++;
            if (circuitTarget(circuitLayer, 0, sr) == null) circuitLayer = 0;
        }
        circuitPath = null;
        
        return new TWThought(TWAction.MOVE, anyOpenDir());
    }

    
    private int[] circuitTarget(int layer, int corner, int sr) {
        int pad = sr + layer * sr * 2;
        int x1 = secXMin + pad, x2 = secXMax - pad;
        int y1 = secYMin + pad, y2 = secYMax - pad;
        if (x2 <= x1 || y2 <= y1) return null;
        switch (corner) {
            case 0: return new int[]{x1, y1};  
            case 1: return new int[]{x2, y1};  
            case 2: return new int[]{x2, y2};  
            case 3: return new int[]{x1, y2};  
            default: return null;
        }
    }

    
    
    
    
    
    
    
    
    
    
    private TWThought explore() { return explore(false); }
    private TWThought explore(boolean shrink) {
        int sr  = tileworld.Parameters.defaultSensorRange;
        int w   = getEnvironment().getxDimension();
        int h   = getEnvironment().getyDimension();
        int ax  = getX(), ay = getY();

        
        int pad    = shrink ? sr : 0;
        int effXMin = Math.min(secXMin + pad, secCX);
        int effXMax = Math.max(secXMax - pad, secCX);
        int effYMin = Math.min(secYMin + pad, secCY);
        int effYMax = Math.max(secYMax - pad, secCY);

        
        for (int x = Math.max(0, ax-sr); x <= Math.min(w-1, ax+sr); x++)
            for (int y = Math.max(0, ay-sr); y <= Math.min(h-1, ay+sr); y++) {
                if (x < effXMin || x > effXMax || y < effYMin || y > effYMax) continue;
                Object obj = getEnvironment().getObjectGrid().get(x, y);
                if (obj instanceof TWTile && carriedTiles.size() < 3) return setGoalMove(x, y);
                if (obj instanceof TWHole && hasTile())               return setGoalMove(x, y);
            }

        if (sweepByRow) {
            
            int rowStart = effYMin / (sr * 2);
            int rowEnd   = effYMax / (sr * 2);
            if (exploreRow < rowStart) exploreRow = rowStart;
            if (exploreRow > rowEnd)   exploreRow = rowStart;

            
            int targetY = Math.max(Math.min(exploreRow * sr * 2 + sr, effYMax), effYMin);
            if (Math.abs(ay - targetY) > 1)
                return new TWThought(TWAction.MOVE, dirTo(ax, targetY));

            
            int xLeft = effXMin + 1, xRight = effXMax - 1;
            if (goingRight && ax >= xRight) {
                goingRight = false; exploreRow++;
                if (exploreRow > rowEnd) exploreRow = rowStart;
            } else if (!goingRight && ax <= xLeft) {
                goingRight = true; exploreRow++;
                if (exploreRow > rowEnd) exploreRow = rowStart;
            }

            TWDirection dir = goingRight ? TWDirection.E : TWDirection.W;
            int nx = ax + dir.dx, ny = ay + dir.dy;
            if (!getEnvironment().isInBounds(nx, ny) || memory.isCellBlocked(nx, ny)) {
                exploreRow++;
                if (exploreRow > rowEnd) exploreRow = rowStart;
                goingRight = !goingRight;
                int newTY = Math.max(Math.min(exploreRow * sr * 2 + sr, effYMax), effYMin);
                return new TWThought(TWAction.MOVE, dirTo(ax, newTY));
            }
            return new TWThought(TWAction.MOVE, dir);

        } else {
            
            int colStart = effXMin / (sr * 2);
            int colEnd   = effXMax / (sr * 2);
            if (exploreCol < colStart) exploreCol = colStart;
            if (exploreCol > colEnd)   exploreCol = colStart;

            int targetX = Math.max(Math.min(exploreCol * sr * 2 + sr, effXMax), effXMin);
            if (Math.abs(ax - targetX) > 1)
                return new TWThought(TWAction.MOVE, dirTo(targetX, ay));

            int yTop = effYMin + 1, yBot = effYMax - 1;
            if (goingDown && ay >= yBot) {
                goingDown = false; exploreCol++;
                if (exploreCol > colEnd) exploreCol = colStart;
            } else if (!goingDown && ay <= yTop) {
                goingDown = true; exploreCol++;
                if (exploreCol > colEnd) exploreCol = colStart;
            }

            TWDirection dir = goingDown ? TWDirection.S : TWDirection.N;
            int nx = ax + dir.dx, ny = ay + dir.dy;
            if (!getEnvironment().isInBounds(nx, ny) || memory.isCellBlocked(nx, ny)) {
                exploreCol++;
                if (exploreCol > colEnd) exploreCol = colStart;
                goingDown = !goingDown;
                int newTX = Math.max(Math.min(exploreCol * sr * 2 + sr, effXMax), effXMin);
                return new TWThought(TWAction.MOVE, dirTo(newTX, ay));
            }
            return new TWThought(TWAction.MOVE, dir);
        }
    }

    
    private TWThought setGoalMove(int tx, int ty) {
        if (goalX != tx || goalY != ty) {
            currentPath = null; goalX = tx; goalY = ty;
            recordClaim(tx, ty);
        }
        TWThought m = followPath(tx, ty);
        return m != null ? m : new TWThought(TWAction.MOVE, dirTo(tx, ty));
    }

    private TWThought followPath(int tx, int ty) {
        if (currentPath == null || !currentPath.hasNext())
            currentPath = pathGen.findPath(getX(), getY(), tx, ty);
        if (currentPath == null || !currentPath.hasNext()) {
            
            
            unreachable.put(key(tx, ty), getEnvironment().schedule.getTime());
            currentPath = null; goalX = -1; goalY = -1; return null;
        }
        return new TWThought(TWAction.MOVE, currentPath.popNext().getDirection());
    }

    private TWThought headTo(int tx, int ty) {
        if (goalX != tx || goalY != ty) { currentPath = null; goalX = tx; goalY = ty; }
        TWThought t = followPath(tx, ty);
        return t != null ? t : new TWThought(TWAction.MOVE, dirTo(tx, ty));
    }

    
    private void recordClaim(int x, int y) {
        claimOwner.put(key(x,y), name);
        claimTime.put(key(x,y), getEnvironment().schedule.getTime());
    }
    private void releaseClaim(int x, int y) {
        if (name.equals(claimOwner.get(key(x,y)))) {
            claimOwner.remove(key(x,y)); claimTime.remove(key(x,y));
        }
    }
    private boolean isClaimedByOther(int x, int y, double now) {
        String k = key(x,y); String owner = claimOwner.get(k);
        if (owner == null) return false;
        Double ct = claimTime.get(k);
        if (ct == null || now - ct > CLAIM_EXPIRY) {
            claimOwner.remove(k); claimTime.remove(k); return false;
        }
        return !owner.equals(name);
    }

    
    private boolean isTileKnown(int x, int y, double now) {
        Object mem = memory.getMemoryGrid().get(x, y);
        if (mem instanceof TWTile && getEnvironment().getObjectGrid().get(x,y) != null) return true;
        Double t = teamTiles.get(key(x,y));
        return t != null && now - t <= OBS_EXPIRY && getEnvironment().getObjectGrid().get(x,y) instanceof TWTile;
    }
    private boolean isHoleKnown(int x, int y, double now) {
        Object mem = memory.getMemoryGrid().get(x, y);
        if (mem instanceof TWHole && getEnvironment().getObjectGrid().get(x,y) != null) return true;
        Double t = teamHoles.get(key(x,y));
        return t != null && now - t <= OBS_EXPIRY && getEnvironment().getObjectGrid().get(x,y) instanceof TWHole;
    }

    
    private void scanForStation(int ax, int ay) {
        int sr  = tileworld.Parameters.defaultSensorRange;
        int mapW = getEnvironment().getxDimension();
        int mapH = getEnvironment().getyDimension();
        for (int x = Math.max(0,ax-sr); x <= Math.min(mapW-1,ax+sr); x++)
            for (int y = Math.max(0,ay-sr); y <= Math.min(mapH-1,ay+sr); y++) {
                Object obj = getEnvironment().getObjectGrid().get(x, y);
                if (obj instanceof TWFuelStation) {
                    stationX = x; stationY = y; stationKnown = true;
                    System.out.println("[" + name + "] found station at (" + x + "," + y + ")");
                    return;
                }
            }
    }

    private boolean isUnreachable(int x, int y, double now) {
        Double t = unreachable.get(key(x, y));
        if (t == null) return false;
        if (now - t > BLACKLIST_EXPIRY) { unreachable.remove(key(x, y)); return false; }
        return true;
    }

    private boolean inSector(int x, int y) {
        return x >= secXMin && x <= secXMax && y >= secYMin && y <= secYMax;
    }
    private TWDirection dirTo(int tx, int ty) {
        int dx = tx-getX(), dy = ty-getY();
        if (dx==0&&dy==0) return TWDirection.Z;
        TWDirection h = dx>0?TWDirection.E:TWDirection.W;
        TWDirection v = dy>0?TWDirection.S:TWDirection.N;
        if (dx==0) return v; if (dy==0) return h;
        TWDirection pri = Math.abs(dx)>=Math.abs(dy)?h:v;
        TWDirection sec = Math.abs(dx)>=Math.abs(dy)?v:h;
        if (canMove(pri)) return pri; if (canMove(sec)) return sec;
        return anyOpenDir();
    }
    private boolean canMove(TWDirection d) {
        int nx=getX()+d.dx, ny=getY()+d.dy;
        return getEnvironment().isInBounds(nx,ny) && !memory.isCellBlocked(nx,ny);
    }
    private TWDirection anyOpenDir() {
        TWDirection[] all={TWDirection.N,TWDirection.E,TWDirection.S,TWDirection.W};
        int s=(getX()+getY())%4;
        for (int i=0;i<4;i++) { TWDirection d=all[(s+i)%4]; if(canMove(d)) return d; }
        return TWDirection.Z;
    }
    private double manhattanDist(int x1,int y1,int x2,int y2){return Math.abs(x1-x2)+Math.abs(y1-y2);}
    private String key(int x,int y){return x+":"+y;}
    private double parseDouble(String s,double f){try{return Double.parseDouble(s);}catch(Exception e){return f;}}
    private TWTile getTileAt(int x,int y){Object o=getEnvironment().getObjectGrid().get(x,y);return(o instanceof TWTile)?(TWTile)o:null;}
    private TWHole getHoleAt(int x,int y){Object o=getEnvironment().getObjectGrid().get(x,y);return(o instanceof TWHole)?(TWHole)o:null;}

    @Override public String getName() { return name; }
}