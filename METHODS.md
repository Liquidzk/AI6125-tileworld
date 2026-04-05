# Methods Documentation

This document describes the current final strategy implemented on branch `6-agent-sectors`.

The goal of this version is to provide a practical, explainable multi-agent method for Tileworld under the two public configurations:

- `config1`: `50 x 50`
- `config2`: `80 x 80`

The implementation is not a globally optimal planner. It is a layered method built from:

- reactive decision making
- short-horizon utility scoring
- local path planning with A*
- working memory plus strategic memory
- static geographic team partitioning
- sector-based exploration on large maps
- lightweight team communication for high-value shared state

The design emphasis is stability, low per-step planning cost, and reasonable cooperation without building a heavy centralized controller.

## 1. Execution Setup

The benchmark entry point is `src/tileworld/TileworldMain.java`.

Default run settings are defined in `src/tileworld/Parameters.java`:

- `5000` steps per run
- `6` agents
- initial fuel `500`
- sensor range `3`

Profile-specific environment parameters:

### `config1`

- map size: `50 x 50`
- tile generation mean/std: `0.2 / 0.05`
- hole generation mean/std: `0.2 / 0.05`
- obstacle generation mean/std: `0.2 / 0.05`
- object lifetime: `100`

### `config2`

- map size: `80 x 80`
- tile generation mean/std: `2.0 / 0.5`
- hole generation mean/std: `2.0 / 0.5`
- obstacle generation mean/std: `2.0 / 0.5`
- object lifetime: `30`

The environment creates:

- `6` agents at random starting locations
- one fixed fuel station at a random valid object location
- new tiles, holes, and obstacles every step

The environment stepping order is:

1. environment update
2. all agents `sense()` and `communicate()`
3. all agents `think()` and `act()`

This matters because communication is synchronous at the step level: agents first observe and publish, then decide.

## 2. High-Level Architecture

The current implementation is organized around four main components.

### 2.1 Environment

`src/tileworld/environment/TWEnvironment.java`

Responsible for:

- maintaining the object grid and agent grid
- spawning agents
- creating and deleting world objects
- maintaining total reward
- providing movement and occupancy checks

### 2.2 Base Agent

`src/tileworld/agent/TWAgent.java`

Provides:

- movement
- pickup / putdown / refuel primitives
- fuel accounting
- individual score accounting
- scheduling hook via `step(...)`

### 2.3 Strategy Agent

`src/tileworld/agent/SimpleTWAgent.java`

This is the main strategy class. It implements:

- communication
- decision making
- target selection
- exploration
- team coordination
- fuel-aware navigation

### 2.4 Strategic Memory

`src/tileworld/agent/StrategicTWAgentMemory.java`

This extends the basic working memory and adds structured strategic state:

- known tiles
- known holes
- known fuel station
- sector freshness and local/shared sector statistics

## 3. Agent Behavior: Overall Decision Flow

The strategy is reactive and re-evaluates its situation every step.

The decision order in `SimpleTWAgent` is:

1. if currently on the fuel station and not full, refuel
2. if currently on a hole and carrying at least one tile, put down
3. if currently on a tile and carrying fewer than three tiles, pick up
4. otherwise compute a movement decision

This ordering is intentional.

- `PUTDOWN` is immediate reward realization
- `PICKUP` is immediate resource acquisition
- `REFUEL` is immediate survivability restoration
- movement is deferred until no higher-value zero-travel action is available

The movement decision then branches on whether the fuel station is known.

## 4. Strategic Memory

The base working memory supplied by the framework stores perceived entities in a grid-like memory structure. The current method adds a higher-level memory layer on top of it.

### 4.1 Known Targets

The strategy tracks three explicit target types:

- `TILE`
- `HOLE`
- `FUEL_STATION`

Each remembered target stores:

- type
- `x, y`
- observation time
- expiry time

Fuel stations are treated as permanent once observed.

Tiles and holes carry expiry times based on the environment object lifetime.

### 4.2 Memory Maintenance

Memory is refreshed every time the agent senses the environment.

The update process is:

1. call base working memory update
2. reconcile visible area with the new perception
3. prune expired knowledge
4. remember newly observed entities
5. update sector freshness
6. rebuild sector-level target counts

Two cleanup rules are important:

- if a location is currently visible and no longer contains a remembered tile or hole, the stale memory entry is removed
- if a remembered tile or hole has passed its expiry time, it is removed

This prevents the agent from chasing stale targets indefinitely.

## 5. Target Selection

Target selection is one of the major changes from the original starter behavior.

The agent does not simply choose the nearest tile or nearest hole.

Instead it uses a two-stage process:

1. cheap shortlist
2. refined reranking

### 5.1 Candidate Filtering

A remembered target is only considered if:

- it still exists in memory
- it is not already expired
- it appears reachable before expiry
- it passes a fuel-budget check
- it is either visible now or recently seen

This already removes many bad candidates.

### 5.2 Tile Scoring

Tiles are scored using:

- distance from the agent
- distance from the tile to the nearest known viable hole
- observation age
- carried tile bonus
- cross-sector penalty

Interpretation:

- close tiles are better
- tiles near holes are better
- fresher tiles are better
- tiles are slightly more attractive when the agent is emptier
- tiles deep in another agent's macro-zone are penalized

### 5.3 Hole Scoring

Holes are scored using:

- distance from the agent
- observation age
- carried tile bonus
- cross-sector penalty

Interpretation:

- close holes are better
- fresher holes are better
- holes become increasingly attractive as the agent carries more tiles

### 5.4 Refinement Stage

Only a small shortlist of candidates is kept, then re-evaluated using a more expensive score.

The refinement stage adds:

- A* path detour relative to Manhattan distance
- slack before object expiry
- local cluster bonus

For tiles:

- shorter path detour is better
- more time remaining before expiry is better
- nearby tile density gives a small bonus

For holes:

- shorter path detour is better
- more slack is better
- if carrying multiple tiles, nearby hole density gives a bonus

The main effect is that the agent stops being purely distance-greedy and becomes more delivery-oriented.

## 6. Delivery Decision

After the fuel station is known, the agent must decide whether to:

- continue collecting tiles
- or deliver what it currently carries

This is handled by `shouldDeliverTiles(...)`.

The delivery heuristic uses:

- whether the current hole is urgent
- whether the agent is full
- whether any viable tile remains
- travel distance to the next hole
- travel distance to the next tile
- distance from that tile to a hole

Important rules:

- if the hole is close to expiring, deliver now
- if the agent is carrying three tiles, deliver now
- if no good tile remains, deliver now
- if already carrying two tiles, the policy becomes more conservative

This reduces a common failure mode where agents keep greedily picking up tiles and lose guaranteed reward they could have deposited immediately.

## 7. Fuel Management

Fuel management is a major part of this method.

### 7.1 Basic Rule

If the fuel station is known, the agent continuously checks whether it should head back.

The return decision depends on:

- path distance to the fuel station
- carried load
- possibility of delivering through a hole before refueling
- a safety buffer

### 7.2 Buffers

Different buffers are used for large and small maps.

The large-map configuration is more conservative:

- larger `FUEL_BUFFER`
- larger pre-fuel budget buffer
- larger path safety margin
- tighter freshness window for remembered targets

### 7.3 Fuel Budget for Targets

Before chasing a tile or hole, the agent checks if it can afford:

- travel to the target
- then eventually return to the fuel station
- plus safety reserve

Two levels of estimation are used:

1. a cheap optimistic estimate
2. a more expensive A* estimate if the optimistic one is near the boundary

Before the fuel station is known, the agent cannot do a full return-to-station calculation, so it uses a fixed pre-fuel buffer instead.

### 7.4 Known Weakness

Large-map failures still exist in seeds where the team fails to discover the fuel station early enough. This is primarily a pre-fuel exploration problem rather than a post-discovery refueling bug.

## 8. Path Planning

Movement to a target uses A* through `AstarPathGenerator`.

The design is:

- target selection decides where to go
- A* decides how to go there

If A* finds a path:

- the path is stored
- the agent consumes the next step each turn

If A* fails:

- the agent falls back to directional greedy movement with obstacle checks

If a movement attempt fails because of a blocked path:

- the cached path is discarded
- the agent re-plans on a later step

This keeps the path layer lightweight and robust without requiring a centralized multi-agent path planner.

## 9. Spatial Team Structure

The team uses two spatial layers.

### 9.1 Macro-Zones

The map is split by `x` into six vertical macro-zones, one per agent.

This is static geographic partitioning.

Purpose:

- reduce duplicate exploration
- reduce cross-map roaming
- maintain stable team coverage

The partition is soft, not hard:

- agents prefer their own macro-zone
- but may cross out of it if a target is sufficiently attractive

### 9.2 Sectors

On top of the macro-zones, the world is subdivided into `10 x 10` sectors.

Each sector stores:

- coordinates and bounds
- center point
- local freshness
- shared freshness
- local known tile count
- local known hole count
- shared counts from teammates

The current strategy uses these sectors only as part of the large-map exploration layer.

## 10. Exploration Strategy

Exploration is intentionally configuration-dependent.

### 10.1 `50 x 50`

The method keeps the older macro sweep strategy.

This is a stable lawnmower-style scan inside the agent's macro-zone.

Reason:

- experiments showed that forcing sector exploration on `50 x 50` degraded performance
- the map is small enough that the simple sweep already works well

### 10.2 `80 x 80`

Once the fuel station is known, the method switches to sector exploration.

This is the main addition on the `6-agent-sectors` branch.

The logic is:

1. look at candidate sectors overlapping the agent's macro-zone
2. compute a sector score
3. choose the best sector
4. build a sweep path inside that sector
5. follow that local sweep until it is exhausted

### 10.3 Sector Score

The sector score uses:

- sector freshness
- tile opportunity
- hole opportunity
- delivery opportunity if carrying tiles
- tile-hole pairing opportunity
- travel cost to the sector center
- sector stay bonus
- sector claim penalty from teammates

The result is a sector-level version of utility-driven exploration:

- recently neglected sectors become attractive
- sectors rich in useful objects become attractive
- sectors already being handled by a teammate become slightly less attractive

## 11. Communication Design

Communication in this version is deliberately sparse and high-value.

It is not a full broadcast of all percepts.

### 11.1 Shared Fuel Station

This is the most important shared fact.

If any agent knows the fuel station:

- it publishes the station location to team shared state
- other agents synchronize it into their local memory

This allows the entire team to switch from pre-fuel survival behavior to stable fuel-aware behavior after one discovery.

### 11.2 Sector Snapshot Blackboard

Each agent publishes snapshots of sectors currently visible in its sensor range.

Each snapshot contains:

- sector coordinates
- seen time
- local tile count
- local hole count

These snapshots are merged into a team-shared board and then re-imported by agents into local strategic memory.

This creates a lightweight blackboard:

- not every target is globally broadcast as a first-class task
- but sector-level freshness and sector-level opportunity can propagate

### 11.3 Shared Sector Freshness

Sector state distinguishes:

- what I saw
- what the team saw

The current implementation fuses this conservatively:

- if the agent has never seen a sector locally, the shared freshness matters
- if the agent has already seen it locally, local freshness dominates

This avoids over-trusting remote information while still allowing unseen sectors to benefit from team exploration.

### 11.4 Sector Claim

Each agent also publishes a lightweight claim representing the sector it is currently working in.

The claim contains:

- agent name
- sector coordinates
- current mode
- report time

When another agent scores sectors, it receives a small penalty for:

- exactly the same sector
- or a nearby sector under active exploration by a teammate

The claim is weak and short-lived by design.

It is intended to reduce local crowding, not to implement strict ownership.

## 12. Why Communication Is Limited

The current method does not broadcast every tile and hole for several reasons:

- objects are dynamic and expire
- remote object data becomes stale quickly
- full broadcasting would create a lot of noise
- the team already gets significant value from fuel sharing and sector-level coordination

So communication is currently focused on:

- permanent critical information: fuel station
- medium-grain exploration state: sectors
- lightweight deconfliction: claims

This is consistent with the goal of keeping the method understandable and computationally cheap.

## 13. Benchmark Methodology

The repository contains a benchmark harness under `benchmark/`.

Important files:

- `benchmark/seed-groups.json`
- `benchmark/run-seed-groups.ps1`
- `benchmark/results/`

This harness was added so that strategy changes can be compared on the same fixed seed groups instead of relying only on random 10-run averages.

This was important because:

- random batches can fluctuate
- large-map failures are rare but very costly
- fixed seed comparisons make it much easier to judge whether a change actually helps

## 14. What Changed Relative to the Original Starter Code

At a high level, the main improvements are:

1. one weak baseline agent became a coordinated six-agent team
2. purely local perception was extended with structured strategic memory
3. target choice became utility-driven instead of purely distance-driven
4. fuel became part of target feasibility, not an afterthought
5. exploration became spatially structured
6. communication was added for fuel and sector-level coordination

## 15. Known Limitations

This version is stronger than the starter code, but it still has limitations.

### 15.1 Pre-Fuel Failure on Some `80 x 80` Seeds

There are still rare seeds where:

- no agent discovers the fuel station early enough
- the entire team eventually runs out of fuel

This is not the dominant case, but it is a real failure mode on large maps.

### 15.2 Static Macro-Zone Ownership

Macro-zone ownership is based on agent index, not spawn location.

This is stable and simple, but not always ideal.

An agent may spawn far away from its assigned macro-zone, which can delay useful early exploration.

### 15.3 Shared Sector Counts Are Not Yet Fully Exploited

Shared sector counts are stored in memory, but the current sector scoring still relies mainly on local counts and conservative freshness fusion.

This was done to avoid destabilizing the small-map configuration and to limit noise.

### 15.4 No Full Target Claim System

There is no explicit tile-level or hole-level ownership protocol.

That means some duplicated pursuit can still occur, especially when good targets lie close to macro-zone boundaries.

## 16. Design Rationale

The current method deliberately prefers:

- clear structure
- bounded computation per step
- stable team coverage
- explainable heuristics

over:

- centralized optimization
- global task auctioning
- heavy communication
- expensive multi-agent path planning

For this project, that tradeoff has been reasonable:

- `50 x 50` remains stable with macro sweep
- `80 x 80` improves with sector-based exploration and sector sharing
- the method stays understandable enough to document and defend

## 17. Summary

The current `6-agent-sectors` method can be summarized as follows:

- six homogeneous agents operate under a shared reactive policy
- each agent has a static macro-zone for coarse coverage
- all agents maintain strategic memory of tiles, holes, fuel, and sectors
- fuel station discovery is globally shared
- sector-level observations are shared through a lightweight blackboard
- agents choose targets using utility heuristics rather than raw nearest distance
- small maps use stable macro sweeping
- large maps switch to sector-based exploration after fuel discovery
- light sector claims reduce crowding without rigid central control

This is the current final method implemented in the repository.
