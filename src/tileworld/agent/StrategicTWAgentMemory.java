package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import sim.engine.Schedule;
import sim.util.Bag;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;

public class StrategicTWAgentMemory extends TWAgentWorkingMemory {

    public enum TargetType {
        TILE,
        HOLE,
        FUEL_STATION
    }

    public static final class KnownTarget {
        private final TargetType type;
        private final int x;
        private final int y;
        private final double observedAt;
        private final double expiresAt;

        private KnownTarget(TargetType type, int x, int y, double observedAt, double expiresAt) {
            this.type = type;
            this.x = x;
            this.y = y;
            this.observedAt = observedAt;
            this.expiresAt = expiresAt;
        }

        public TargetType getType() {
            return type;
        }

        public int getX() {
            return x;
        }

        public int getY() {
            return y;
        }

        public double getObservedAt() {
            return observedAt;
        }

        public double getExpiresAt() {
            return expiresAt;
        }
    }

    private final TWAgent agent;
    private final Schedule schedule;
    private final Map<Long, KnownTarget> knownTiles;
    private final Map<Long, KnownTarget> knownHoles;
    private KnownTarget fuelStation;

    public StrategicTWAgentMemory(TWAgent agent, Schedule schedule, int xDimension, int yDimension) {
        super(agent, schedule, xDimension, yDimension);
        this.agent = agent;
        this.schedule = schedule;
        this.knownTiles = new HashMap<Long, KnownTarget>();
        this.knownHoles = new HashMap<Long, KnownTarget>();
    }

    @Override
    public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
        super.updateMemory(sensedObjects, objectXCoords, objectYCoords, sensedAgents, agentXCoords, agentYCoords);
        reconcileVisibleArea(sensedObjects);
        pruneExpiredKnowledge();

        for (int i = 0; i < sensedObjects.size(); i++) {
            Object object = sensedObjects.get(i);
            if (object instanceof TWEntity) {
                rememberObservedEntity((TWEntity) object, currentTime());
            }
        }
    }

    public List<KnownTarget> getKnownTargets(TargetType type) {
        pruneExpiredKnowledge();
        return new ArrayList<KnownTarget>(targetMap(type).values());
    }

    public KnownTarget getFuelStation() {
        return fuelStation;
    }

    public void rememberFuelStation(int x, int y) {
        fuelStation = new KnownTarget(TargetType.FUEL_STATION, x, y, currentTime(), Double.POSITIVE_INFINITY);
    }

    public double getObservationAge(KnownTarget target) {
        return target == null ? Double.MAX_VALUE : currentTime() - target.getObservedAt();
    }

    public void invalidateTargetAt(int x, int y) {
        invalidateTarget(TargetType.TILE, x, y);
        invalidateTarget(TargetType.HOLE, x, y);
    }

    public void invalidateTarget(TargetType type, int x, int y) {
        targetMap(type).remove(longKey(x, y));
        if (type == TargetType.FUEL_STATION && fuelStation != null && fuelStation.getX() == x && fuelStation.getY() == y) {
            fuelStation = null;
        }
    }

    private void rememberObservedEntity(TWEntity entity, double observedAt) {
        if (entity instanceof TWFuelStation) {
            fuelStation = new KnownTarget(TargetType.FUEL_STATION, entity.getX(), entity.getY(), observedAt, Double.POSITIVE_INFINITY);
            return;
        }

        if (entity instanceof TWTile) {
            TWObject object = (TWObject) entity;
            rememberTarget(new KnownTarget(TargetType.TILE, entity.getX(), entity.getY(), observedAt, object.getDeathTime()));
            return;
        }

        if (entity instanceof TWHole) {
            TWObject object = (TWObject) entity;
            rememberTarget(new KnownTarget(TargetType.HOLE, entity.getX(), entity.getY(), observedAt, object.getDeathTime()));
        }
    }

    private void rememberTarget(KnownTarget target) {
        Map<Long, KnownTarget> targetMap = targetMap(target.getType());
        long key = longKey(target.getX(), target.getY());
        KnownTarget existing = targetMap.get(key);
        if (existing == null || target.getObservedAt() >= existing.getObservedAt()) {
            targetMap.put(key, target);
        }
    }

    private void reconcileVisibleArea(Bag sensedObjects) {
        Map<Long, TargetType> visibleTargets = new HashMap<Long, TargetType>();
        for (int i = 0; i < sensedObjects.size(); i++) {
            Object object = sensedObjects.get(i);
            if (!(object instanceof TWEntity)) {
                continue;
            }

            TWEntity entity = (TWEntity) object;
            TargetType type = targetTypeOf(entity);
            if (type != null) {
                visibleTargets.put(longKey(entity.getX(), entity.getY()), type);
            }
        }

        int minX = Math.max(0, agent.getX() - Parameters.defaultSensorRange);
        int maxX = Math.min(agent.getEnvironment().getxDimension() - 1, agent.getX() + Parameters.defaultSensorRange);
        int minY = Math.max(0, agent.getY() - Parameters.defaultSensorRange);
        int maxY = Math.min(agent.getEnvironment().getyDimension() - 1, agent.getY() + Parameters.defaultSensorRange);

        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                long key = longKey(x, y);
                TargetType visibleType = visibleTargets.get(key);
                if (visibleType == TargetType.TILE) {
                    knownHoles.remove(key);
                    continue;
                }
                if (visibleType == TargetType.HOLE) {
                    knownTiles.remove(key);
                    continue;
                }
                if (visibleType == TargetType.FUEL_STATION) {
                    continue;
                }

                knownTiles.remove(key);
                knownHoles.remove(key);
                if (fuelStation != null && fuelStation.getX() == x && fuelStation.getY() == y) {
                    fuelStation = null;
                }
            }
        }
    }

    private void pruneExpiredKnowledge() {
        pruneExpiredTargets(knownTiles);
        pruneExpiredTargets(knownHoles);
    }

    private void pruneExpiredTargets(Map<Long, KnownTarget> targets) {
        Iterator<Map.Entry<Long, KnownTarget>> iterator = targets.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<Long, KnownTarget> entry = iterator.next();
            if (entry.getValue().getExpiresAt() <= currentTime()) {
                iterator.remove();
            }
        }
    }

    private Map<Long, KnownTarget> targetMap(TargetType type) {
        if (type == TargetType.TILE) {
            return knownTiles;
        }
        if (type == TargetType.HOLE) {
            return knownHoles;
        }
        return new HashMap<Long, KnownTarget>();
    }

    private TargetType targetTypeOf(TWEntity entity) {
        if (entity instanceof TWTile) {
            return TargetType.TILE;
        }
        if (entity instanceof TWHole) {
            return TargetType.HOLE;
        }
        if (entity instanceof TWFuelStation) {
            return TargetType.FUEL_STATION;
        }
        return null;
    }

    private double currentTime() {
        return schedule.getTime();
    }

    private long longKey(int x, int y) {
        return (((long) x) << 32) | (y & 0xffffffffL);
    }
}
