# Local Avoidance 2D

English | [日本語](README.ja.md)

Local Avoidance 2D is a Unity library that processes large numbers of circular agents using Burst and the Job System.
It provides 2D crowd simulation without depending on Unity Physics.

![video01](media/video01.gif)

![video02](media/video02.gif)

> [!IMPORTANT]
> Local Avoidance 2D is currently under development. The API may change significantly in the future.

## Features

- Predictive velocity avoidance
- Jacobi-style overlap correction
- Circle and segment obstacles; segments with thickness behave as capsules
- Agent and obstacle collision layers
- Per-agent contact aggregation
- Persistent native container reuse for zero steady-state frame GC allocations
- A shared pipeline for asynchronous `Schedule` and synchronous `Step` calls

## Usage

Import the bidirectional-flow and rectangular-obstacle examples from the **Samples** tab in Package Manager.

```csharp
const int agentCapacity = 10_000;
const int obstacleCapacity = 64;
const int agentIndex = 0;
const int activeAgentCount = 1;

using var simulation = new LocalAvoidanceSimulation(
    agentCapacity: agentCapacity,
    obstacleCapacity: obstacleCapacity,
    allocator: Allocator.Persistent);

var initialPosition = new float2(0f, 0f);
var desiredVelocity = new float2(2f, 0f);
const float agentRadius = .4f;

simulation.ActivateAgent(
    agentIndex: agentIndex,
    position: initialPosition,
    desiredVelocity: desiredVelocity,
    radius: agentRadius);

simulation.Schedule(
    deltaTime: deltaTime,
    agentCount: activeAgentCount,
    obstacleCount: 0).Complete();

var resolvedPosition = simulation.ResolvedPositions[agentIndex];
```

`agentIndex` is a fixed slot number within the simulation. `agentCount` specifies how many slots, starting at slot `0`, are processed. The example above processes the single agent in slot `0`.

Write input data to the structure-of-arrays `NativeArray` buffers, call `Schedule`, and then read the result buffers. Integration layers for GameObjects, Unity Entities, LitheEcs, and other frameworks are intentionally left to the application.

## Coordinate convention

The simulation does not define a world plane. Internally, all positions and directions are plain `float2` values and are independent of the world axes.

Only gizmo rendering needs a display plane. Select `XY` or `XZ` with `LocalAvoidanceGizmoSettings.Plane`.

## Lifetime

Do not modify input buffers while a simulation job is running. The handle returned by the previous call is automatically chained into the next `Schedule` call.

Before disposing the simulation, complete any outstanding job so that its native memory is no longer in use.

## Simulation capacity

```csharp
new LocalAvoidanceSimulation(
    agentCapacity: 10_000,
    obstacleCapacity: 64,
    allocator: Allocator.Persistent);
```

| Parameter | Description |
|---|---|
| `agentCapacity` | Maximum number of agent slots owned by the simulation. Capacity does not grow while jobs are running. |
| `obstacleCapacity` | Maximum number of circle or segment obstacles. Zero is allowed. |
| `allocator` | Allocator used by the native containers. `Allocator.Persistent` is normally appropriate. |

The simulation allocates its native memory once in the constructor and reuses it every frame. Passing a count greater than the corresponding capacity to `Schedule` throws `ArgumentOutOfRangeException`.

## LocalAvoidanceSettings

```csharp
simulation.Settings = LocalAvoidanceSettings.Default;
```

`Default` provides general-purpose initial values. Tune `CellSize` and `NeighborDistance` first according to agent diameter and density.

| Setting | Description |
|---|---|
| `CellSize` | Side length of one spatial-grid cell used for neighbor queries. |
| `NeighborDistance` | Radius used to find nearby agents for avoidance. |
| `MaximumNeighbors` | Maximum number of nearby agents retained per agent for avoidance and contact resolution. |
| `MaximumCandidateChecks` | Maximum number of grid candidates examined while selecting neighbors. |
| `CollisionPredictionTime` | How far into the future the solver predicts collisions. |
| `VelocityResponse` | How quickly the current velocity follows the newly calculated desired velocity. |
| `SeparationSpeedRatio` | Strength of the velocity used to separate nearby agents. |
| `LateralSpeedRatio` | Strength of sideways avoidance around agents and obstacles ahead. |
| `LateralFlowFollowing` | Amount of alignment with the lateral flow of agents ahead. |
| `MinimumSpacingRatio` | Minimum maintained spacing relative to the sum of agent radii. |
| `MaximumCorrectionRatio` | Maximum positional correction per solver iteration relative to agent radius. |
| `SolverIterations` | Number of iterations used for overlap and obstacle-penetration correction. |
| `InnerLoopBatchCount` | Number of agents assigned to a worker in each parallel-job batch. |
| `ContactSlowdown` | Maximum slowdown caused by contact pressure. |
| `ContactsForMaximumSlowdown` | Number of agent contacts at which contact slowdown reaches its maximum. |
| `CorrectionVelocityInfluence` | Amount of positional correction retained in the next frame's velocity. |
| `ContactSkinRatio` | Extra distance relative to radius used when counting contacts. |
| `PreferredSeparationMultiplier` | Distance multiplier at which predictive separation begins. |
| `ContactRetentionSkinMultiplier` | Contact-skin multiplier used to retain important contact constraints. |
| `DominantMassRatioThreshold` | Mass ratio at which the heavier agent is treated as a dominant, movement-blocking contact. |

### CellSize

```csharp
CellSize = 1.2f;
```

The side length of one spatial-grid cell, in world-coordinate units.

**Performance impact: high.** This setting determines both the number of candidate agents in each cell and the number of cells searched. Smaller is not always faster; the optimum depends on agent density and the ratio to `NeighborDistance`.

- Too small increases the number of cells searched.
- Too large increases the candidates in each cell.
- Start at 1.5 to 3 times the typical agent diameter.
- Values at or below zero are clamped to `0.01`.

For an agent radius of `0.4`, `1.2` is a reasonable starting value.

### NeighborDistance

```csharp
NeighborDistance = 1.2f;
```

The distance within which agents are considered for velocity avoidance.

**Performance impact: high.** Crossing a cell boundary increases the number of cells searched in steps. The search covers approximately `(2 * ceil(NeighborDistance / CellSize) + 1)^2` cells. Values below `CellSize` are raised to `CellSize`, so they do not reduce the search cost.

- Larger values start slowing and avoidance earlier.
- Excessively large values cause unnecessary avoidance and search more cells.
- Values that are too small make the solver rely more heavily on positional correction.
- Start at 2 to 4 times the typical agent diameter.

The neighbor search runs once per frame. Its cached indices are reused for predictive avoidance and every solver iteration, with corrected distances recalculated during the solver. Set `NeighborDistance` to at least the maximum agent diameter plus the contact skin. Agents that become neighbors during the solver are discovered on the next frame.

### MaximumNeighbors

```csharp
MaximumNeighbors = 8;
```

The maximum number of nearest agents retained for each agent and used for avoidance and contact resolution.

**Performance impact: high.** Lower values reduce neighbor selection, avoidance, and per-iteration contact work in dense crowds. If the value is too small, necessary contacts can fall out of the cache and increase overlap or tunneling. The default is `8`; the valid range is `1` to `10`. Values at or below zero use the default. The maximum of `10` is the physical capacity of the internal `FixedList128Bytes`.

### MaximumCandidateChecks

```csharp
MaximumCandidateChecks = 64;
```

The maximum number of spatial-grid candidates examined per agent before selecting the nearest neighbors. Limiting candidate checks prevents per-agent work from growing indefinitely with density.

- Lower values limit processing time in dense cells.
- Values that are too small can miss important neighbors.
- Values below `MaximumNeighbors` are raised to `MaximumNeighbors`.
- Values at or below zero use the default of `64`.

Agents with `DirectControl` or `StableContactResolution` enabled are examined separately and do not consume this limit.

### CollisionPredictionTime

```csharp
CollisionPredictionTime = 0.5f;
```

How many seconds ahead the solver considers a collision while current velocities are maintained. For example, `0.5` applies forward slowdown and lateral avoidance to objects that would be contacted within half a second.

```text
prediction time = surface distance between agents / relative approach speed
```

A larger value starts avoidance earlier; a smaller value preserves forward speed until later. Stationary or separating agents and agents beside or behind the direction of travel are excluded. Agent prediction only sees agents within `NeighborDistance`; obstacles are checked directly up to `obstacleCount`. Values at or below zero use the default of `0.5` rather than disabling prediction. Increase `NeighborDistance` as well for high-speed opposing flows.

### VelocityResponse

```csharp
VelocityResponse = 10f;
```

How quickly `CurrentVelocity` follows the calculated target velocity, approximately in inverse seconds.

```text
Small value  Smooth, but slow to respond
Large value  Fast response with sharper direction changes
0            Retains CurrentVelocity during ordinary updates
```

The value is used in frame-rate-independent exponential interpolation. On a frame where `ImmediateVelocity[index] = 1`, the calculated velocity is applied immediately regardless of this setting.

### SeparationSpeedRatio

```csharp
SeparationSpeedRatio = 0.4f;
```

The amount of velocity added, relative to desired speed, to move away from nearby agents.

- `0` disables predictive separation velocity.
- Larger values separate agents more strongly before they become crowded.
- Excessive values expand crowds and increase small direction changes.
- Negative values are clamped to `0`.

A typical tuning range is `0.25` to `0.6`.

### LateralSpeedRatio

```csharp
LateralSpeedRatio = 0.15f;
```

The lateral velocity ratio applied when an agent or obstacle is ahead. Agents choose a consistent passing side. For segment obstacles, the nearer endpoint is preferred and the choice is retained briefly to prevent direction flips around corners.

- `0` disables lateral avoidance and relies mainly on slowdown.
- Larger values route more aggressively around objects ahead.
- Excessive values can produce weaving and crowd dispersion.

Start around `0.1` to `0.3`.

### LateralFlowFollowing

```csharp
LateralFlowFollowing = 0.65f;
```

How much a following agent adopts the lateral velocity of an agent ahead moving in the same direction. This reduces collisions where only the lead agent avoids. `0` disables the behavior, `1` applies the full lateral flow, and values are clamped to `0` through `1`.

### MinimumSpacingRatio

```csharp
MinimumSpacingRatio = 0.95f;
```

The minimum spacing maintained by positional correction.

```csharp
minimumDistance = (radiusA + radiusB) * MinimumSpacingRatio;
```

- `1.0` requires the full sum of the radii.
- `0.95` permits a small visual overlap.
- Smaller values allow tighter crowds and can reduce correction cost and oscillation.
- Values are clamped to `0.01` through `1.0`.

When many agents converge on one point, `0.9` to `0.98` is generally more stable than completely overlap-free `1.0`.

### MaximumCorrectionRatio

```csharp
MaximumCorrectionRatio = 0.25f;
```

The maximum distance an agent can be moved in one solver iteration, relative to its radius.

```csharp
maximumCorrection = agentRadius * MaximumCorrectionRatio;
```

Larger values resolve overlap faster but can cause violent movement or oscillation in crowds. Smaller values can require several frames to resolve deep overlaps. Negative values are clamped to `0`; `0.15` to `0.35` is normally appropriate.

### SolverIterations

```csharp
SolverIterations = 2;
```

The number of iterations used to correct overlap and obstacle penetration after movement.

**Performance impact: high.** Every iteration adds constraint work for all active agents, so cost is approximately proportional to the iteration count.

| Value | Behavior |
|---:|---|
| `1` | Lowest cost; overlap is more likely to remain in dense crowds. |
| `2` | Default balance between performance and appearance. |
| `3` to `4` | Stricter resolution with additional job time. |

Values are clamped to `1` through `8`.

### InnerLoopBatchCount

```csharp
InnerLoopBatchCount = 128;
```

The batch size passed to `IJobParallelFor.Schedule`.

**Performance impact: medium to high.** Smaller batches distribute uneven per-agent workloads better but add scheduling overhead. Larger batches reduce overhead but can leave one worker running longer and increase the wait at `Complete`. The minimum is `1`; profile on the target hardware because the optimum depends on core count, agent count, and density.

### ContactSlowdown

```csharp
ContactSlowdown = 0.8f;
```

The maximum proportion by which this frame's `DesiredVelocity` is reduced when the agent contacted surrounding agents on the previous frame. At `0.8`, at least 20% of desired speed remains at maximum pressure.

- `0` disables contact-pressure slowdown.
- Larger values reduce continued pushing into dense centers.
- `1` reduces desired velocity to zero at maximum pressure.
- Values are clamped to `0` through `1`.

### ContactsForMaximumSlowdown

```csharp
ContactsForMaximumSlowdown = 6f;
```

The agent contact count at which `ContactSlowdown` reaches its maximum.

```csharp
pressure = saturate(agentContactCount / ContactsForMaximumSlowdown);
desiredVelocity *= 1 - pressure * ContactSlowdown;
```

Smaller values slow strongly after only a few contacts. Larger values retain speed until density is higher. The minimum is `1`.

### CorrectionVelocityInfluence

```csharp
CorrectionVelocityInfluence = 0.15f;
```

The proportion of solver positional correction converted into velocity carried to the next frame.

```csharp
velocity += correction / deltaTime * CorrectionVelocityInfluence;
```

`0` disables velocity retention. Larger values preserve outward flow from dense centers, but excessive values can cause popping, oscillation, or excessive speed. Start around `0.1` to `0.2`; negative values are clamped to `0`. Corrections accumulate over solver iterations, so retune this value after increasing `SolverIterations`.

### ContactSkinRatio

```csharp
ContactSkinRatio = 0.1f;
```

Extra spacing, relative to the sum of radii, within which an almost-touching agent counts toward crowd pressure.

```csharp
contactDistance = minimumDistance +
                  (radiusA + radiusB) * ContactSkinRatio;
```

At `0`, only agents below the actual minimum spacing are counted. Positive values include near contacts in `AgentContactCount`, while positional correction still occurs only below the minimum spacing. Excessive values slow crowds that are not touching. Start around `0.05` to `0.15`.

This prevents agents from accelerating immediately after the solver temporarily removes their overlap.

### PreferredSeparationMultiplier

The distance multiplier, relative to the sum of agent radii, at which predictive separation begins. The default is `1.2` and the minimum is `1`. Larger values spread the crowd earlier. Values at or below zero use the default for compatibility with partially initialized settings.

### ContactRetentionSkinMultiplier

How far the contact skin is extended while retaining the constraint that most strongly limits an agent's movement. The default is `2` and the minimum is `1`. Values that are too small switch constraint partners frequently; values that are too large retain agents that have already moved apart.

### DominantMassRatioThreshold

The mass ratio at which the other agent is treated as the contact that most strongly limits movement. The default is `4` and the minimum is `1`. If the other agent's mass is at least this multiple of the current agent's mass, avoidance and contact constraints stably retain it as difficult to move.

## Agent input buffers

Populate the simulation's structure-of-arrays input buffers before scheduling a step. Inactive slots must have `Active = 0`; the library does not clear the complete active array every frame.

| Buffer | Description |
|---|---|
| `Positions` | Center position at the start of the frame. |
| `DesiredVelocities` | Velocity vector describing the intended direction and speed before avoidance. |
| `CurrentVelocities` | Resolved velocity from the previous frame, used for velocity smoothing. |
| `Radii` | Radius of the circular agent. Set a value greater than zero. |
| `Masses` | Resistance to displacement, used for correction distribution and dominant-contact selection. The standard value is `1`. |
| `AvoidancePriorities` | Higher-priority agents omit predictive avoidance of lower-priority agents and take precedence during contact correction. |
| `AvoidanceWeights` | Predictive-avoidance strength: `0` moves directly and `1` uses standard avoidance. |
| `CorrectionVelocityWeights` | Per-agent multiplier for retaining positional correction in the next frame's velocity. |
| `MaximumCorrectionSpeeds` | Maximum positional-correction speed. `0` means unlimited. |
| `Layers` | 32-bit layer to which the agent belongs. |
| `CollisionMasks` | Layers recognized for avoidance and contact. |
| `ContactEventMasks` | Other layers for which enter and exit events are collected. This does not affect collision response. |
| `Active` | `1` processes the slot; `0` ignores it. |
| `ImmediateVelocity` | `1` applies the calculated velocity immediately without smoothing. |
| `DirectControl` | Disables predictive avoidance and contact slowdown in favor of input velocity. Non-penetration constraints remain active. |
| `StableContactResolution` | Uses the deterministic deepest contact as the constraint instead of combining contact normals. |

### Layers

Agent-to-agent interaction requires both masks to match:

```csharp
(agentA.CollisionMask & agentB.Layer) != 0 &&
(agentB.CollisionMask & agentA.Layer) != 0
```

`ContactEventMasks` is separate from collision filtering and selects which contacts produce enter and exit events.

If A includes B but B excludes A, neither agent performs avoidance or contact resolution against the other.

### Masses

Mass determines how positional correction is shared and helps select movement-blocking contacts. It is not used to calculate acceleration.

```text
Mass 0.5  Light; displaced easily
Mass 1.0  Standard
Mass 3.0  About three times harder to displace than the standard mass
```

The correction share is derived from inverse mass:

```csharp
inverseMassA = 1 / massA;
shareA = inverseMassA / (inverseMassA + inverseMassB);
```

Use obstacles rather than an extremely large agent mass for objects that must be completely immovable.

For equal masses, both agents receive half of the correction. With `Mass A = 1` and `Mass B = 3`, A moves by 75% of the overlap and B by 25%. Values at or below zero are treated as `0.0001` during calculation.

When the mass ratio reaches `DominantMassRatioThreshold`, the heavier agent is retained as the contact that most strongly limits movement. This stabilizes the primary constraint normal in dense crowds; it is more than a correction-share calculation.

### AvoidanceWeights

Per-agent avoidance strength is combined with the global `SeparationSpeedRatio` and `LateralSpeedRatio` settings.

| Value | Behavior |
|---:|---|
| `0` | Moves directly toward `DesiredVelocity` without predictive or lateral avoidance. Mass-based overlap correction remains active. |
| `0.25` | Weak avoidance, favoring direct progress toward the goal. |
| `1` | Standard predictive avoidance. |
| `2` | Stronger separation and lateral avoidance. |

Negative values are treated as `0`.

There is no implementation maximum. Forward slowdown saturates at `1`, while values above `1` continue to amplify separation, lateral avoidance, and lateral-flow following. A practical range is `0` to `2`. Start at `1` for autopiloted agents that should avoid congestion, or `0.1` to `0.3` for enemies that should favor direct pursuit.

## Teleport

Use `Teleport` to move an agent without applying the normal movement calculation:

```csharp
simulation.Teleport(agentIndex, new float2(x, y));
```

This synchronizes position-related buffers and resets velocity, contact constraints, and obstacle-avoidance direction. If a simulation job is outstanding, `Teleport` completes it before modifying the buffers; when teleporting many agents, call it for the group after job completion. Pass `resetVelocity: false` when velocity must be preserved.

## Obstacles

### Circle

```csharp
obstacles[0] = Obstacle.Circle(
    position,
    radius,
    layer,
    collidesWith);
```

Circle obstacles represent stationary objects such as pillars, rocks, or trees. `position` is the center.

### Segment / Capsule

```csharp
obstacles[0] = Obstacle.Segment(
    pointA,
    pointB,
    radius,
    layer,
    collidesWith);
```

A segment extends from `PointA` to `PointB`. With `radius = 0` it is a line wall; with a positive radius it behaves as a capsule-shaped wall. Negative radii are clamped to `0`.

When an agent will approach an obstacle within `CollisionPredictionTime`, it slows and avoids laterally before contact. A segment favors the nearer endpoint and a circle favors the agent's current side. That choice is retained slightly beyond the prediction period to prevent avoidance direction from flipping near corners. If prediction is too late, the post-movement non-penetration constraint returns the agent to the obstacle surface.

Agent-to-obstacle filtering also checks both masks:

```csharp
(agent.CollisionMask & obstacle.Layer) != 0 &&
(obstacle.CollidesWith & agent.Layer) != 0
```

In version `0.1.0`, every active agent checks the requested obstacles sequentially, so obstacle processing scales with `agent count × obstacle count`. Prefer a small number of gameplay walls rather than splitting a complex map into many segments until an obstacle spatial grid is available.

## Schedule / Step parameters

```csharp
JobHandle handle = simulation.Schedule(
    deltaTime,
    agentCount,
    obstacleCount,
    dependency);
```

| Parameter | Description |
|---|---|
| `deltaTime` | Time step in seconds used for position integration. Non-positive values perform no processing and return the dependency handle. |
| `agentCount` | Number of slots processed from the beginning of the agent buffers. This is not necessarily the number of active agents. |
| `obstacleCount` | Number of slots used from the beginning of the obstacle buffer. |
| `dependency` | Jobs that must complete before the simulation reads its input buffers. |

`Schedule` returns the final `JobHandle` and does not call `Complete` internally. It also automatically includes the handle returned by the previous call on this simulation in its dependencies. The synchronous API is equivalent to scheduling and immediately completing:

```csharp
simulation.Step(deltaTime, agentCount, obstacleCount);
```

Overloads without explicit counts process the complete configured capacities. Supplying counts is preferable when only part of a large buffer is in use.

```csharp
simulation.Step(deltaTime);
JobHandle handle = simulation.Schedule(deltaTime);
```

Inactive agent slots are skipped. Overloads without counts still inspect every obstacle slot through `ObstacleCapacity`, so use the counted overload when fewer obstacles are populated than were allocated.

## Output buffers

| Buffer | Description |
|---|---|
| `ResolvedPositions` | Positions after avoidance, movement, overlap correction, and obstacle constraints. |
| `MovedPositions` | Positions after velocity integration and initial obstacle constraints, before the agent-agent solver. |
| `ResolvedVelocities` | Velocities after avoidance and obstacle-normal constraints. |
| `Contacts` | Aggregated contact state for each agent. |
| `EnteredContacts` | Agent pairs that became event-eligible contacts during this step. |
| `ExitedContacts` | Agent pairs that ceased to be event-eligible contacts during this step. |

## AgentContactState

`AgentContactState` is an aggregate intended for crowd pressure, animation, effects, and gameplay events; it is not a complete list of every contact pair.

| Field | Description |
|---|---|
| `AgentContactCount` | Nearby agents within the contact skin during the final solver iteration. |
| `ObstacleContactCount` | Obstacles contacted during the final solver iteration. |
| `BlockingAgentContactCount` | Contacts ahead of the desired direction whose avoidance priority is equal or higher. |
| `ForwardPenetrationPressure` | Maximum penetration ratio ahead. It reaches `1` at 25% of the combined radii. |
| `Priority0ContactCount` through `Priority2ContactCount` | Agent contact counts grouped by avoidance priority. |
| `CombinedNormal` | Sum of contact normals. Normalize it in application code if needed. |
| `ConstraintNormal` / `AllowedNormalSpeed` | Primary non-penetration constraint retained for the next frame. |
| `ConstraintAgentIndex` | Slot of the agent selected as the primary constraint. Check `HasConstraint` before using it. |
| `ConstraintOtherMass` / `ConstraintOtherRadius` | Properties of the selected constraint partner. |
| `ConstraintPenetration` | Penetration distance against the selected constraint partner. |
| `CorrectionLimit` | Maximum positional correction distance available to this agent in the final solver iteration. |
| `HasConstraint` | `1` when a primary non-penetration constraint is retained. |
| `ConstraintBlocksMovement` | `1` when the representative contact can constrain this agent's velocity. |
| `ConstraintIsDominant` | `1` when the contact was selected as movement-limiting due to the mass ratio. |
| `IsTouching` | `1` when touching an agent or obstacle. |

Contact data is aggregated, not a list of every contact pair. Handle damage, effects, audio, and gameplay events in the integration layer. Because `AgentContactCount` includes agents within `ContactSkinRatio`, treat it as a crowd-density indicator rather than an exact collision event.

## Agent raycast

```csharp
int count = simulation.Raycast(
    origin, direction, distance, capsuleRadius,
    queryLayer, queryCollisionMask, results);
```

This is a non-allocating swept-circle query against current agent positions. Results are ordered by distance and limited by the length of the supplied `NativeArray<RaycastHit>`. The query completes any outstanding simulation job first. Filtering requires both `queryCollisionMask & agent.Layer` and `agent.CollisionMask & queryLayer` to match. Obstacles are not included.

## Recommended tuning order

1. Set `CellSize` to 1.5 to 3 times the typical agent diameter.
2. Set `NeighborDistance` to 2 to 4 times the typical agent diameter.
3. Start with `MinimumSpacingRatio = 0.95` and `SolverIterations = 2`.
4. Configure per-agent `Masses` and `AvoidanceWeights`.
5. Adjust `ContactSlowdown` when agents keep pushing into a dense center.
6. Increase `CorrectionVelocityInfluence` slightly when sideways flow does not settle.
7. Increase `MaximumCorrectionRatio` slightly when overlap remains visible.
8. If agents move violently, reduce `MaximumCorrectionRatio`; if that is not enough, reduce `CorrectionVelocityInfluence` as well.
9. Increase `LateralSpeedRatio` when agents jam during head-on encounters.

## Scope

This package does not provide rigid-body physics, friction, restitution, joints, or pathfinding. `Raycast` only supplies a non-allocating 2D ray/capsule query against registered agents; it does not query obstacles or arbitrary colliders.

The package is focused on movement avoidance and penetration prevention for large numbers of circular agents.

## Installation
Git Path (Unity Package Manager)
> ```https://github.com/kurobon-jp/LocalAvoidance2D.git?path=Packages/com.github.kurobon.local-avoidance-2d/```
