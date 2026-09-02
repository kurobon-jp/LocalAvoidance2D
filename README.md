# Local Avoidance 2D

English | [日本語](README.ja.md)

Local Avoidance 2D is a Unity library that processes large numbers of circular agents using Burst and the Job System.
It provides 2D crowd simulation without depending on Unity Physics.

![video01](media/video01.gif)

![video02](media/video02.gif)

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

### Practical tuning notes

- Start `CellSize` at roughly 1.5 times the typical agent diameter.
- Start `NeighborDistance` at roughly twice the typical agent diameter.
- Increase `MaximumNeighbors` in dense crowds if important contacts are being missed.
- A longer `CollisionPredictionTime` produces earlier avoidance; a shorter value reacts later and relies more on positional correction.
- Higher `SolverIterations` improves overlap resolution at additional CPU cost.
- Keep `MaximumCorrectionRatio` conservative to reduce visible jitter.

## Agent input buffers

Populate the simulation's structure-of-arrays input buffers before scheduling a step. Inactive slots must have `Active = 0`; the library does not clear the complete active array every frame.

### Layers

Agent-to-agent interaction requires both masks to match:

```csharp
(agentA.CollisionMask & agentB.Layer) != 0 &&
(agentB.CollisionMask & agentA.Layer) != 0
```

`ContactEventMasks` is separate from collision filtering and selects which contacts produce enter and exit events.

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

### AvoidanceWeights

Per-agent avoidance strength is combined with the global `SeparationSpeedRatio` and `LateralSpeedRatio` settings.

| Value | Behavior |
|---:|---|
| `0` | Moves directly toward `DesiredVelocity` without predictive or lateral avoidance. Mass-based overlap correction remains active. |
| `0.25` | Weak avoidance, favoring direct progress toward the goal. |
| `1` | Standard predictive avoidance. |
| `2` | Stronger separation and lateral avoidance. |

Negative values are treated as `0`.

## Teleport

Use `Teleport` to move an agent without applying the normal movement calculation:

```csharp
simulation.Teleport(agentIndex, new float2(x, y));
```

This synchronizes position-related buffers and resets velocity, contact constraints, and obstacle-avoidance direction. Pass `resetVelocity: false` when velocity must be preserved.

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

A segment extends from `PointA` to `PointB`. With `radius = 0` it is a line wall; with a positive radius it behaves as a capsule-shaped wall.

Agent-to-obstacle filtering also checks both masks:

```csharp
(agent.CollisionMask & obstacle.Layer) != 0 &&
(obstacle.CollidesWith & agent.Layer) != 0
```

Obstacle processing currently scales with `agent count × obstacle count`. Prefer a small number of gameplay walls rather than splitting a complex map into many segments.

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

`Schedule` returns the final `JobHandle` and does not call `Complete` internally. The synchronous API is equivalent to scheduling and immediately completing:

```csharp
simulation.Step(deltaTime, agentCount, obstacleCount);
```

Overloads without explicit counts process the complete configured capacities. Supplying counts is preferable when only part of a large buffer is in use.

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

Important fields include:

- `AgentContactCount` and `ObstacleContactCount`
- `BlockingAgentContactCount`
- `ForwardPenetrationPressure`
- `Priority0ContactCount` through `Priority2ContactCount`
- `CombinedNormal`
- `ConstraintNormal` and `AllowedNormalSpeed`
- `ConstraintAgentIndex`, `ConstraintOtherMass`, and `ConstraintOtherRadius`
- `ConstraintPenetration` and `CorrectionLimit`
- `HasConstraint`, `ConstraintBlocksMovement`, and `ConstraintIsDominant`
- `IsTouching`

## Agent raycast

```csharp
int count = simulation.Raycast(
    origin, direction, distance, capsuleRadius,
    queryLayer, queryCollisionMask, results);
```

This is a non-allocating 2D ray/capsule query against current agent positions. Results are ordered by distance and limited by the length of the supplied `NativeArray<RaycastHit>`. Complete any outstanding simulation job before querying. Obstacles are not included.

## Recommended tuning order

1. Set `CellSize` to roughly 1.5 times the typical agent diameter.
2. Set `NeighborDistance` to roughly twice the typical agent diameter.
3. Start with `MinimumSpacingRatio = 0.95` and `SolverIterations = 2`.
4. Configure per-agent `Masses` and `AvoidanceWeights`.
5. Adjust `ContactSlowdown` when agents keep pushing into a dense center.
6. Increase `CorrectionVelocityInfluence` slightly when sideways flow does not settle.
7. Reduce `MaximumCorrectionRatio` when overlap correction produces visible jitter.
8. If agents still bounce, reduce `CorrectionVelocityInfluence` as well.
9. Increase `LateralSpeedRatio` when agents jam during head-on encounters.

## Scope

This package does not provide rigid-body physics, friction, acceleration, joints, or pathfinding. `Raycast` only supplies a non-allocating 2D query against registered agents; it does not query obstacles or arbitrary colliders.

The package is focused on movement avoidance and penetration prevention for large numbers of circular agents.

## Installation
Git Path (Unity Package Manager)
> ```https://github.com/kurobon-jp/LocalAvoidance2D.git?path=Packages/com.github.kurobon.local-avoidance-2d/```