package tileworld.agent;

import java.util.HashMap;
import java.util.List;
import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.NeighbourSpiral;
import tileworld.environment.TWEntity;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

public class TWAgentWorkingMemory {

    private final Schedule schedule;
    private final TWAgent me;
    private final ObjectGrid2D memoryGrid;
    private final TWAgentPercept[][] objects;
    private int memorySize;
    private HashMap<Class<?>, TWEntity> closestInSensorRange;
    private static final List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();

    public TWAgentWorkingMemory(TWAgent moi, Schedule schedule, int x, int y) {
        this.closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);
        this.me = moi;
        this.objects = new TWAgentPercept[x][y];
        this.schedule = schedule;
        this.memoryGrid = new ObjectGrid2D(me.getEnvironment().getxDimension(), me.getEnvironment().getyDimension());
    }

    public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
        closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);
        assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());

        for (int i = 0; i < sensedObjects.size(); i++) {
            TWEntity o = (TWEntity) sensedObjects.get(i);
            if (!(o instanceof TWObject)) {
                continue;
            }

            if (objects[o.getX()][o.getY()] == null) {
                memorySize++;
            }

            objects[o.getX()][o.getY()] = new TWAgentPercept(o, getSimulationTime());
            memoryGrid.set(o.getX(), o.getY(), o);
            updateClosest(o);
        }
    }

    public TWTile getNearbyTile(int x, int y, double threshold) {
        return (TWTile) getNearbyObject(x, y, threshold, TWTile.class);
    }

    public TWHole getNearbyHole(int x, int y, double threshold) {
        return (TWHole) getNearbyObject(x, y, threshold, TWHole.class);
    }

    public int getMemorySize() {
        return memorySize;
    }

    private TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {
        double maxTimestamp = 0;
        TWObject current = null;
        TWObject fallback = null;

        for (Int2D offset : spiral) {
            int x = offset.x + sx;
            int y = offset.y + sy;

            if (me.getEnvironment().isInBounds(x, y) && objects[x][y] != null) {
                current = (TWObject) objects[x][y].getO();
                if (!type.isInstance(current)) {
                    continue;
                }

                double seenAt = objects[x][y].getT();
                if (getSimulationTime() - seenAt <= threshold) {
                    return current;
                }
                if (seenAt > maxTimestamp) {
                    fallback = current;
                    maxTimestamp = seenAt;
                }
            }
        }

        return fallback;
    }

    public TWEntity getClosestObjectInSensorRange(Class<?> type) {
        return closestInSensorRange.get(type);
    }

    private void updateClosest(TWEntity o) {
        if (closestInSensorRange.get(o.getClass()) == null || me.closerTo(o, closestInSensorRange.get(o.getClass()))) {
            closestInSensorRange.put(o.getClass(), o);
        }
    }

    public boolean isCellBlocked(int tx, int ty) {
        if (objects[tx][ty] == null) {
            return false;
        }
        TWEntity e = (TWEntity) objects[tx][ty].getO();
        return (e instanceof TWObstacle);
    }

    public ObjectGrid2D getMemoryGrid() {
        return this.memoryGrid;
    }

    private double getSimulationTime() {
        return schedule.getTime();
    }
}
