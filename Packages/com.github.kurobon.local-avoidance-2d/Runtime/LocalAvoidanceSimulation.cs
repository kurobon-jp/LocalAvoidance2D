using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace LocalAvoidance2D
{
    public sealed class LocalAvoidanceSimulation : IDisposable
    {
        // FixedList128Bytes<Neighbor> has this physical capacity for the current Neighbor layout.
        // LocalAvoidanceSettings.MaximumNeighbors must not exceed it.
        private const int NeighborCapacity = 10;
        private NativeParallelMultiHashMap<int, int> _agentGrid;
        private NativeList<int> _largeAgents;
        private NativeList<int> _requiredAgents;
        private NativeArray<float2> _scratchPositions;
        private NativeArray<float2> _constraintInputVelocities;
        private NativeArray<NeighborCacheEntry> _neighborCache;
        private NativeArray<sbyte> _obstacleAvoidanceSides;
        private NativeArray<float> _obstacleAvoidanceRetentionTimes;
        private NativeParallelHashSet<ulong> _currentContactPairs;
        private NativeParallelHashSet<ulong> _previousContactPairs;
        private JobHandle _lastHandle;
#if ENABLE_DIAGNOSTICS_LOG
        private LocalAvoidanceDiagnostics _fallbackDiagnostics;
#endif

        public int Capacity { get; }
        public int ObstacleCapacity { get; }
        public LocalAvoidanceSettings Settings { get; set; }
        public NativeArray<float2> Positions { get; }
        public NativeArray<float2> DesiredVelocities { get; }
        public NativeArray<float2> CurrentVelocities { get; }
        public NativeArray<float> Radii { get; }
        public NativeArray<float> Masses { get; }
        public NativeArray<byte> AvoidancePriorities { get; }
        public NativeArray<float> AvoidanceWeights { get; }
        /// <summary>Per-agent multiplier for converting positional correction into retained velocity.</summary>
        public NativeArray<float> CorrectionVelocityWeights { get; }
        /// <summary>Maximum positional correction speed. Zero means unlimited.</summary>
        public NativeArray<float> MaximumCorrectionSpeeds { get; }
        public NativeArray<uint> Layers { get; }
        public NativeArray<uint> CollisionMasks { get; }
        /// <summary>Layers for which contact events should be collected. Does not affect collision response.</summary>
        public NativeArray<uint> ContactEventMasks { get; }
        public NativeArray<byte> Active { get; }
        public NativeArray<byte> ImmediateVelocity { get; }
        /// <summary>Bypasses predictive avoidance and contact slowdown while retaining collision constraints.</summary>
        public NativeArray<byte> DirectControl { get; }
        /// <summary>Uses one deterministic deepest contact instead of summing crowd corrections.</summary>
        public NativeArray<byte> StableContactResolution { get; }
        /// <summary>Retained obstacle passing side: -1 right, 0 unset, 1 left.</summary>
        public NativeArray<sbyte> ObstacleAvoidanceSides => _obstacleAvoidanceSides;
        public NativeArray<float> ObstacleAvoidanceRetentionTimes => _obstacleAvoidanceRetentionTimes;
        public NativeArray<Obstacle> Obstacles { get; }
        public NativeArray<float2> ResolvedPositions { get; }
        /// <summary>Positions after movement integration and before contact constraints.</summary>
        public NativeArray<float2> MovedPositions { get; }
        public NativeArray<float2> ResolvedVelocities { get; }
        public NativeArray<AgentContactState> Contacts { get; }
        /// <summary>Agent pairs that started touching during the last completed step.</summary>
        public NativeList<AgentContactPair> EnteredContacts { get; }
        public int EnteredContactCount => EnteredContacts.Length;
        public AgentContactPair GetEnteredContact(int index) => EnteredContacts[index];
        /// <summary>Agent pairs that stopped touching during the last completed step.</summary>
        public NativeList<AgentContactPair> ExitedContacts { get; }
        public int ExitedContactCount => ExitedContacts.Length;
        public AgentContactPair GetExitedContact(int index) => ExitedContacts[index];

        public LocalAvoidanceSimulation(int agentCapacity, int obstacleCapacity = 16,
            Allocator allocator = Allocator.Persistent)
        {
            if (agentCapacity <= 0) throw new ArgumentOutOfRangeException(nameof(agentCapacity));
            if (obstacleCapacity < 0) throw new ArgumentOutOfRangeException(nameof(obstacleCapacity));
            Capacity = agentCapacity;
            ObstacleCapacity = obstacleCapacity;
            Settings = LocalAvoidanceSettings.Default;
            Positions = new NativeArray<float2>(agentCapacity, allocator);
            DesiredVelocities = new NativeArray<float2>(agentCapacity, allocator);
            CurrentVelocities = new NativeArray<float2>(agentCapacity, allocator);
            Radii = new NativeArray<float>(agentCapacity, allocator);
            Masses = new NativeArray<float>(agentCapacity, allocator);
            AvoidancePriorities = new NativeArray<byte>(agentCapacity, allocator);
            AvoidanceWeights = new NativeArray<float>(agentCapacity, allocator);
            CorrectionVelocityWeights = new NativeArray<float>(agentCapacity, allocator);
            MaximumCorrectionSpeeds = new NativeArray<float>(agentCapacity, allocator);
            Layers = new NativeArray<uint>(agentCapacity, allocator);
            CollisionMasks = new NativeArray<uint>(agentCapacity, allocator);
            ContactEventMasks = new NativeArray<uint>(agentCapacity, allocator);
            Active = new NativeArray<byte>(agentCapacity, allocator);
            var masses = Masses;
            var avoidanceWeights = AvoidanceWeights;
            var correctionVelocityWeights = CorrectionVelocityWeights;
            var layers = Layers;
            var collisionMasks = CollisionMasks;
            for (var i = 0; i < agentCapacity; i++)
            {
                masses[i] = 1f;
                avoidanceWeights[i] = 1f;
                correctionVelocityWeights[i] = 1f;
                layers[i] = 1u;
                collisionMasks[i] = 1u;
            }
            ImmediateVelocity = new NativeArray<byte>(agentCapacity, allocator);
            DirectControl = new NativeArray<byte>(agentCapacity, allocator);
            StableContactResolution = new NativeArray<byte>(agentCapacity, allocator);
            Obstacles = new NativeArray<Obstacle>(obstacleCapacity, allocator);
            ResolvedPositions = new NativeArray<float2>(agentCapacity, allocator);
            MovedPositions = new NativeArray<float2>(agentCapacity, allocator);
            ResolvedVelocities = new NativeArray<float2>(agentCapacity, allocator);
            Contacts = new NativeArray<AgentContactState>(agentCapacity, allocator);
            // Neighbor caches are directional; in the worst case every agent can contribute
            // Directional neighbor caches can contribute up to NeighborCapacity canonical pairs
            // per agent when the runtime setting is at its maximum.
            var pairCapacity = math.max(agentCapacity, agentCapacity * NeighborCapacity);
            EnteredContacts = new NativeList<AgentContactPair>(pairCapacity, allocator);
            ExitedContacts = new NativeList<AgentContactPair>(pairCapacity, allocator);
            _currentContactPairs = new NativeParallelHashSet<ulong>(pairCapacity, allocator);
            _previousContactPairs = new NativeParallelHashSet<ulong>(pairCapacity, allocator);
            _scratchPositions = new NativeArray<float2>(agentCapacity, allocator);
            _constraintInputVelocities = new NativeArray<float2>(agentCapacity, allocator);
            _neighborCache = new NativeArray<NeighborCacheEntry>(agentCapacity, allocator);
            _obstacleAvoidanceSides = new NativeArray<sbyte>(agentCapacity, allocator);
            _obstacleAvoidanceRetentionTimes = new NativeArray<float>(agentCapacity, allocator);
            _agentGrid = new NativeParallelMultiHashMap<int, int>(agentCapacity, allocator);
            _largeAgents = new NativeList<int>(agentCapacity, allocator);
            _requiredAgents = new NativeList<int>(agentCapacity, allocator);
        }

        /// <summary>
        /// Activates an agent slot with the required per-agent inputs and clears transient state
        /// left by a previous use of the slot. Standard mass, avoidance and layer values are
        /// supplied by the simulation constructor and can be overridden through their buffers.
        /// </summary>
        public void ActivateAgent(int agentIndex, float2 position, float2 desiredVelocity, float radius)
        {
            if ((uint)agentIndex >= (uint)Capacity)
                throw new ArgumentOutOfRangeException(nameof(agentIndex));
            if (!(radius > 0f) || !math.isfinite(radius))
                throw new ArgumentOutOfRangeException(nameof(radius));

            Teleport(agentIndex, position);
            var desiredVelocities = DesiredVelocities;
            var radii = Radii;
            var active = Active;
            desiredVelocities[agentIndex] = desiredVelocity;
            radii[agentIndex] = radius;
            active[agentIndex] = 1;
        }

        /// <summary>
        /// Moves one agent immediately and clears state that is invalid across a discontinuous move.
        /// Completes the simulation's previously scheduled work before writing its buffers.
        /// </summary>
        public void Teleport(int agentIndex, float2 destination, bool resetVelocity = true)
        {
            if ((uint)agentIndex >= (uint)Capacity)
                throw new ArgumentOutOfRangeException(nameof(agentIndex));
            _lastHandle.Complete();

            var positions = Positions;
            var resolvedPositions = ResolvedPositions;
            var movedPositions = MovedPositions;
            positions[agentIndex] = destination;
            resolvedPositions[agentIndex] = destination;
            movedPositions[agentIndex] = destination;

            if (resetVelocity)
            {
                var currentVelocities = CurrentVelocities;
                var resolvedVelocities = ResolvedVelocities;
                currentVelocities[agentIndex] = float2.zero;
                resolvedVelocities[agentIndex] = float2.zero;
            }

            var contacts = Contacts;
            var obstacleSides = _obstacleAvoidanceSides;
            var obstacleRetentionTimes = _obstacleAvoidanceRetentionTimes;
            contacts[agentIndex] = default;
            obstacleSides[agentIndex] = 0;
            obstacleRetentionTimes[agentIndex] = 0f;

        }

        /// <summary>
        /// Schedules every agent and obstacle slot in the allocated capacities. Inactive agent
        /// slots are skipped; every obstacle slot is inspected.
        /// </summary>
        public JobHandle Schedule(float deltaTime) =>
            Schedule(deltaTime, Capacity, ObstacleCapacity);

        public JobHandle Schedule(float deltaTime, int agentCount, int obstacleCount = 0,
            JobHandle dependency = default) =>
            Schedule(deltaTime, agentCount, obstacleCount, null, dependency);

        public JobHandle Schedule(float deltaTime, int agentCount, int obstacleCount,
            LocalAvoidanceDiagnostics diagnostics, JobHandle dependency = default)
        {
            if (deltaTime <= 0f) return default;
            if ((uint)agentCount > (uint)Capacity) throw new ArgumentOutOfRangeException(nameof(agentCount));
            if ((uint)obstacleCount > (uint)ObstacleCapacity)
                throw new ArgumentOutOfRangeException(nameof(obstacleCount));
#if ENABLE_DIAGNOSTICS_LOG
            diagnostics ??= _fallbackDiagnostics ??= new LocalAvoidanceDiagnostics(Capacity);
            if (diagnostics.Capacity != Capacity)
                throw new ArgumentException("Diagnostic capacity must match simulation capacity.",
                    nameof(diagnostics));
#endif
            var settings = Settings.Sanitized();
            var combinedDependency = JobHandle.CombineDependencies(_lastHandle, dependency);
            var clearGrid = new ClearGridJob
            {
                Grid = _agentGrid,
                LargeAgents = _largeAgents,
                RequiredAgents = _requiredAgents
            }.Schedule(combinedDependency);
            var clearPairs = new ClearContactPairsJob
            {
                Current = _currentContactPairs,
                Entered = EnteredContacts,
                Exited = ExitedContacts
            }.Schedule(combinedDependency);
            var clear = JobHandle.CombineDependencies(clearGrid, clearPairs);
            var build = new BuildGridJob
            {
                Positions = Positions,
                Radii = Radii,
                Active = Active,
                DirectControl = DirectControl,
                StableContactResolution = StableContactResolution,
                LargeRadiusThreshold = settings.CellSize * .5f,
                InverseCellSize = 1f / settings.CellSize,
                Grid = _agentGrid.AsParallelWriter(),
                LargeAgents = _largeAgents.AsParallelWriter(),
                RequiredAgents = _requiredAgents.AsParallelWriter()
            }.Schedule(agentCount, settings.InnerLoopBatchCount, clear);
            var move = new MoveJob
            {
                Positions = Positions,
                DesiredVelocities = DesiredVelocities,
                CurrentVelocities = CurrentVelocities,
                Radii = Radii,
                AvoidancePriorities = AvoidancePriorities,
                AvoidanceWeights = AvoidanceWeights,
                Layers = Layers,
                CollisionMasks = CollisionMasks,
                Active = Active,
                ImmediateVelocity = ImmediateVelocity,
                DirectControl = DirectControl,
                StableContactResolution = StableContactResolution,
#if ENABLE_DIAGNOSTICS_LOG
                DiagnosticDesiredBeforeConstraint = diagnostics.DesiredBeforeConstraint,
                DiagnosticDesiredAfterConstraint = diagnostics.DesiredAfterConstraint,
                DiagnosticPreviousConstraintNormal = diagnostics.PreviousConstraintNormal,
                DiagnosticPreviousAllowedNormalSpeed = diagnostics.PreviousAllowedNormalSpeed,
                DiagnosticConstraintApplied = diagnostics.ConstraintApplied,
                DiagnosticCandidateChecks = diagnostics.CandidateChecks,
                DiagnosticRetainedNeighborCounts = diagnostics.RetainedNeighborCounts,
                DiagnosticSameCellCandidateChecks = diagnostics.SameCellCandidateChecks,
                DiagnosticCandidateLimitReached = diagnostics.CandidateLimitReached,
                DiagnosticCachedNeighbors = diagnostics.CachedNeighbors,
#endif
                PreviousContacts = Contacts,
                Obstacles = Obstacles,
                ObstacleCount = obstacleCount,
                Grid = _agentGrid.AsReadOnly(),
                LargeAgents = _largeAgents,
                RequiredAgents = _requiredAgents,
                NeighborCache = _neighborCache,
                ObstacleAvoidanceSides = _obstacleAvoidanceSides,
                ObstacleAvoidanceRetentionTimes = _obstacleAvoidanceRetentionTimes,
                ResolvedPositions = MovedPositions,
                ResolvedVelocities = ResolvedVelocities,
                InverseCellSize = 1f / settings.CellSize,
                NeighborDistance = settings.NeighborDistance,
                CollisionPredictionTime = settings.CollisionPredictionTime,
                LargeRadiusThreshold = settings.CellSize * .5f,
                VelocityResponse = settings.VelocityResponse,
                PreferredSeparationMultiplier = settings.PreferredSeparationMultiplier,
                SeparationSpeedRatio = settings.SeparationSpeedRatio,
                LateralSpeedRatio = settings.LateralSpeedRatio,
                LateralFlowFollowing = settings.LateralFlowFollowing,
                ContactSlowdown = settings.ContactSlowdown,
                ContactsForMaximumSlowdown = settings.ContactsForMaximumSlowdown,
                MinimumSpacingRatio = settings.MinimumSpacingRatio,
                MaximumNeighbors = settings.MaximumNeighbors,
                MaximumCandidateChecks = settings.MaximumCandidateChecks,
                DeltaTime = math.max(0f, deltaTime)
            }.Schedule(agentCount, settings.InnerLoopBatchCount, build);

            var input = MovedPositions;
            var output = _scratchPositions;
            // Constraint jobs read neighboring velocities. Snapshot them first so parallel
            // writes cannot make the result order-dependent.
            var handle = new CopyPositionsJob
            {
                Source = ResolvedVelocities,
                Destination = _constraintInputVelocities
            }.Schedule(agentCount, settings.InnerLoopBatchCount, move);
            for (var iteration = 0; iteration < settings.SolverIterations; iteration++)
            {
                var finalIteration = iteration == settings.SolverIterations - 1;
                if (finalIteration) output = ResolvedPositions;
                else if (iteration > 0) output = input.Equals(ResolvedPositions) ? _scratchPositions : ResolvedPositions;
                // First iteration cannot read and write ResolvedPositions.
                if (input.Equals(output)) output = _scratchPositions;
                handle = new ConstraintJob
                {
                    Positions = input,
                    Radii = Radii,
                    Masses = Masses,
                    AvoidancePriorities = AvoidancePriorities,
                    CorrectionVelocityWeights = CorrectionVelocityWeights,
                    MaximumCorrectionSpeeds = MaximumCorrectionSpeeds,
                    Layers = Layers,
                    CollisionMasks = CollisionMasks,
                    ContactEventMasks = ContactEventMasks,
                    Active = Active,
                    DirectControl = DirectControl,
                    StableContactResolution = StableContactResolution,
                    SolverIteration = iteration,
                    SolverIterationCount = settings.SolverIterations,
#if ENABLE_DIAGNOSTICS_LOG
                    DiagnosticSolverPositions = diagnostics.SolverPositions,
                    DiagnosticSolverCorrections = diagnostics.SolverCorrections,
                    DiagnosticCapacity = Capacity,
#endif
                    Obstacles = Obstacles,
                    ObstacleCount = obstacleCount,
                    NeighborCache = _neighborCache,
                    CorrectedPositions = output,
                    Velocities = ResolvedVelocities,
                    DesiredVelocities = DesiredVelocities,
                    InputVelocities = _constraintInputVelocities,
                    Contacts = Contacts,
                    ContactPairs = _currentContactPairs.AsParallelWriter(),
                    WriteContacts = (byte)(finalIteration ? 1 : 0),
                    MinimumSpacingRatio = settings.MinimumSpacingRatio,
                    MaximumCorrectionRatio = settings.MaximumCorrectionRatio,
                    ContactSkinRatio = settings.ContactSkinRatio,
                    ContactRetentionSkinMultiplier = settings.ContactRetentionSkinMultiplier,
                    DominantMassRatioThreshold = settings.DominantMassRatioThreshold,
                    DeltaTime = math.max(0f, deltaTime),
                    CorrectionVelocityInfluence = settings.CorrectionVelocityInfluence,
                    InverseSolverIterations = 1f / settings.SolverIterations
                }.Schedule(agentCount, settings.InnerLoopBatchCount, handle);
                input = output;
            }

            handle = new FinalizeContactPairsJob
            {
                Current = _currentContactPairs,
                Previous = _previousContactPairs,
                Entered = EnteredContacts,
                Exited = ExitedContacts
            }.Schedule(handle);

            if (!input.Equals(ResolvedPositions))
                handle = new CopyPositionsJob { Source = input, Destination = ResolvedPositions }
                    .Schedule(agentCount, settings.InnerLoopBatchCount, handle);
            _lastHandle = handle;
            return handle;
        }

        /// <summary>
        /// Updates every agent and obstacle slot in the allocated capacities synchronously.
        /// Prefer the count overload when capacity is larger than the currently used range.
        /// </summary>
        public void Step(float deltaTime) => Schedule(deltaTime).Complete();

        public void Step(float deltaTime, int agentCount, int obstacleCount = 0) =>
            Schedule(deltaTime, agentCount, obstacleCount).Complete();

        public void Step(float deltaTime, int agentCount, int obstacleCount,
            LocalAvoidanceDiagnostics diagnostics) =>
            Schedule(deltaTime, agentCount, obstacleCount, diagnostics).Complete();

        /// <summary>
        /// Performs a non-allocating swept-circle query against the current agent positions.
        /// Results are sorted by distance. The caller owns ray identity and hit deduplication.
        /// </summary>
        public int Raycast(float2 origin, float2 direction, float distance, float radius,
            uint layer, uint collisionMask, NativeArray<RaycastHit> results)
        {
            if (!results.IsCreated || results.Length == 0 || distance <= 0f || radius < 0f)
                return 0;
            _lastHandle.Complete();
            direction = math.normalizesafe(direction);
            if (math.lengthsq(direction) <= 1e-8f) return 0;

            var count = 0;
            for (var i = 0; i < Capacity; i++)
            {
                if (Active[i] == 0 || (collisionMask & Layers[i]) == 0 ||
                    (CollisionMasks[i] & layer) == 0) continue;
                var relative = Positions[i] - origin;
                var along = math.dot(relative, direction);
                if (along < 0f || along > distance) continue;
                var lateral = math.abs(relative.x * direction.y - relative.y * direction.x);
                if (lateral > radius + Radii[i]) continue;

                var insert = count;
                while (insert > 0 && results[insert - 1].Distance > along) insert--;
                if (insert >= results.Length) continue;
                var last = math.min(count, results.Length - 1);
                for (var j = last; j > insert; j--) results[j] = results[j - 1];
                results[insert] = new RaycastHit { AgentIndex = i, Distance = along, Position = Positions[i] };
                if (count < results.Length) count++;
            }
            return count;
        }

        public void Dispose()
        {
            _lastHandle.Complete();
#if ENABLE_DIAGNOSTICS_LOG
            _fallbackDiagnostics?.Dispose();
            _fallbackDiagnostics = null;
#endif
            if (_agentGrid.IsCreated) _agentGrid.Dispose();
            if (_largeAgents.IsCreated) _largeAgents.Dispose();
            if (_requiredAgents.IsCreated) _requiredAgents.Dispose();
            if (_previousContactPairs.IsCreated) _previousContactPairs.Dispose();
            if (_currentContactPairs.IsCreated) _currentContactPairs.Dispose();
            if (EnteredContacts.IsCreated) EnteredContacts.Dispose();
            if (ExitedContacts.IsCreated) ExitedContacts.Dispose();
            if (_neighborCache.IsCreated) _neighborCache.Dispose();
            if (_obstacleAvoidanceSides.IsCreated) _obstacleAvoidanceSides.Dispose();
            if (_obstacleAvoidanceRetentionTimes.IsCreated) _obstacleAvoidanceRetentionTimes.Dispose();
            if (_scratchPositions.IsCreated) _scratchPositions.Dispose();
            if (_constraintInputVelocities.IsCreated) _constraintInputVelocities.Dispose();
            if (Contacts.IsCreated) Contacts.Dispose();
            if (ResolvedVelocities.IsCreated) ResolvedVelocities.Dispose();
            if (ResolvedPositions.IsCreated) ResolvedPositions.Dispose();
            if (MovedPositions.IsCreated) MovedPositions.Dispose();
            if (Obstacles.IsCreated) Obstacles.Dispose();
            if (ImmediateVelocity.IsCreated) ImmediateVelocity.Dispose();
            if (DirectControl.IsCreated) DirectControl.Dispose();
            if (StableContactResolution.IsCreated) StableContactResolution.Dispose();
            if (Active.IsCreated) Active.Dispose();
            if (CollisionMasks.IsCreated) CollisionMasks.Dispose();
            if (ContactEventMasks.IsCreated) ContactEventMasks.Dispose();
            if (Layers.IsCreated) Layers.Dispose();
            if (Radii.IsCreated) Radii.Dispose();
            if (AvoidanceWeights.IsCreated) AvoidanceWeights.Dispose();
            if (CorrectionVelocityWeights.IsCreated) CorrectionVelocityWeights.Dispose();
            if (MaximumCorrectionSpeeds.IsCreated) MaximumCorrectionSpeeds.Dispose();
            if (Masses.IsCreated) Masses.Dispose();
            if (AvoidancePriorities.IsCreated) AvoidancePriorities.Dispose();
            if (CurrentVelocities.IsCreated) CurrentVelocities.Dispose();
            if (DesiredVelocities.IsCreated) DesiredVelocities.Dispose();
            if (Positions.IsCreated) Positions.Dispose();
        }

        [BurstCompile]
        private struct ClearGridJob : IJob
        {
            public NativeParallelMultiHashMap<int, int> Grid;
            public NativeList<int> LargeAgents;
            public NativeList<int> RequiredAgents;
            public void Execute()
            {
                Grid.Clear();
                LargeAgents.Clear();
                RequiredAgents.Clear();
            }
        }

        [BurstCompile]
        private struct ClearContactPairsJob : IJob
        {
            public NativeParallelHashSet<ulong> Current;
            public NativeList<AgentContactPair> Entered;
            public NativeList<AgentContactPair> Exited;
            public void Execute()
            {
                Current.Clear();
                Entered.Clear();
                Exited.Clear();
            }
        }

        [BurstCompile]
        private struct FinalizeContactPairsJob : IJob
        {
            [ReadOnly] public NativeParallelHashSet<ulong> Current;
            public NativeParallelHashSet<ulong> Previous;
            public NativeList<AgentContactPair> Entered;
            public NativeList<AgentContactPair> Exited;

            public void Execute()
            {
                foreach (var key in Current)
                {
                    if (!Previous.Contains(key))
                        Entered.Add(new AgentContactPair
                        {
                            AgentA = (int)(key >> 32),
                            AgentB = (int)(key & uint.MaxValue)
                        });
                }
                foreach (var key in Previous)
                {
                    if (!Current.Contains(key))
                        Exited.Add(new AgentContactPair
                        {
                            AgentA = (int)(key >> 32),
                            AgentB = (int)(key & uint.MaxValue)
                        });
                }
                Previous.Clear();
                foreach (var key in Current) Previous.Add(key);
            }
        }

        [BurstCompile]
        private struct BuildGridJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float2> Positions;
            [ReadOnly] public NativeArray<float> Radii;
            [ReadOnly] public NativeArray<byte> Active;
            [ReadOnly] public NativeArray<byte> DirectControl, StableContactResolution;
            public float InverseCellSize, LargeRadiusThreshold;
            public NativeParallelMultiHashMap<int, int>.ParallelWriter Grid;
            public NativeList<int>.ParallelWriter LargeAgents;
            public NativeList<int>.ParallelWriter RequiredAgents;
            public void Execute(int index)
            {
                if (Active[index] == 0) return;
                if (DirectControl[index] != 0 || StableContactResolution[index] != 0)
                    RequiredAgents.AddNoResize(index);
                if (Radii[index] > LargeRadiusThreshold) LargeAgents.AddNoResize(index);
                else Grid.Add(Key(Cell(Positions[index], InverseCellSize)), index);
            }
        }

        private struct Neighbor
        {
            public int Index;
            public float SurfaceDistance;
            public byte Required;
        }

        private struct NeighborCacheEntry
        {
            public FixedList128Bytes<int> Indices;
        }

        [BurstCompile]
        private struct MoveJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float2> Positions, DesiredVelocities, CurrentVelocities;
            [ReadOnly] public NativeArray<float> Radii;
            [ReadOnly] public NativeArray<byte> AvoidancePriorities;
            [ReadOnly] public NativeArray<float> AvoidanceWeights;
            [ReadOnly] public NativeArray<uint> Layers, CollisionMasks;
            [ReadOnly] public NativeArray<byte> Active, ImmediateVelocity, DirectControl, StableContactResolution;
#if ENABLE_DIAGNOSTICS_LOG
            [WriteOnly] public NativeArray<float2> DiagnosticDesiredBeforeConstraint;
            [WriteOnly] public NativeArray<float2> DiagnosticDesiredAfterConstraint;
            [WriteOnly] public NativeArray<float2> DiagnosticPreviousConstraintNormal;
            [WriteOnly] public NativeArray<float> DiagnosticPreviousAllowedNormalSpeed;
            // Keep this read/write so the job's byte-array safety metadata remains compatible
            // with the neighboring priority reads performed by the same job.
            public NativeArray<byte> DiagnosticConstraintApplied;
            [WriteOnly] public NativeArray<int> DiagnosticCandidateChecks;
            [WriteOnly] public NativeArray<int> DiagnosticRetainedNeighborCounts;
            [WriteOnly] public NativeArray<int> DiagnosticSameCellCandidateChecks;
            [WriteOnly] public NativeArray<byte> DiagnosticCandidateLimitReached;
            [WriteOnly] public NativeArray<FixedList128Bytes<int>> DiagnosticCachedNeighbors;
#endif
            [ReadOnly] public NativeArray<AgentContactState> PreviousContacts;
            [ReadOnly] public NativeArray<Obstacle> Obstacles;
            [ReadOnly] public NativeParallelMultiHashMap<int, int>.ReadOnly Grid;
            [ReadOnly] public NativeList<int> LargeAgents;
            [ReadOnly] public NativeList<int> RequiredAgents;
            [WriteOnly] public NativeArray<NeighborCacheEntry> NeighborCache;
            public NativeArray<sbyte> ObstacleAvoidanceSides;
            public NativeArray<float> ObstacleAvoidanceRetentionTimes;
            [WriteOnly] public NativeArray<float2> ResolvedPositions, ResolvedVelocities;
            public int ObstacleCount;
            public float InverseCellSize, NeighborDistance, LargeRadiusThreshold, VelocityResponse;
            public float CollisionPredictionTime;
            public float PreferredSeparationMultiplier;
            public float SeparationSpeedRatio, LateralSpeedRatio, DeltaTime;
            public float LateralFlowFollowing;
            public float ContactSlowdown, ContactsForMaximumSlowdown;
            public float MinimumSpacingRatio;
            public int MaximumNeighbors, MaximumCandidateChecks;

            public void Execute(int index)
            {
                if (Active[index] == 0) return;
                var position = Positions[index];
                var desired = DesiredVelocities[index];
                var priority = AvoidancePriorities[index];
                var desiredBeforeConstraint = desired;
                var immediate = ImmediateVelocity[index] != 0;
                var directControl = immediate || DirectControl[index] != 0;
                var stableContactResolution = StableContactResolution[index] != 0;
                var idleStable = stableContactResolution && !directControl &&
                                 math.lengthsq(desiredBeforeConstraint) <= 1e-8f;
                var previousContact = PreviousContacts[index];
                if ((directControl || stableContactResolution) && previousContact.HasConstraint != 0 &&
                    previousContact.ConstraintBlocksMovement != 0)
                {
                    var contactNormal = math.normalizesafe(previousContact.ConstraintNormal);
                    var desiredNormalSpeed = math.dot(desired, contactNormal);
                    if (math.lengthsq(contactNormal) > 1e-8f &&
                        desiredNormalSpeed < previousContact.AllowedNormalSpeed)
                        desired += contactNormal *
                                   (previousContact.AllowedNormalSpeed - desiredNormalSpeed);
                }
#if ENABLE_DIAGNOSTICS_LOG
                DiagnosticDesiredBeforeConstraint[index] = desiredBeforeConstraint;
                DiagnosticDesiredAfterConstraint[index] = desired;
                DiagnosticPreviousConstraintNormal[index] = previousContact.ConstraintNormal;
                DiagnosticPreviousAllowedNormalSpeed[index] = previousContact.AllowedNormalSpeed;
                DiagnosticConstraintApplied[index] =
                    (byte)(math.lengthsq(desired - desiredBeforeConstraint) > 1e-10f ? 1 : 0);
#endif
                var blockingPressure = math.saturate(
                    previousContact.BlockingAgentContactCount / ContactsForMaximumSlowdown);
                // Deep overlap only adds pressure when it lies ahead. Side and rear overlap must
                // not strand outer agents after room opens toward their destination.
                var penetrationPressure = previousContact.ForwardPenetrationPressure;
                var pressure = directControl
                    ? 0f
                    : math.max(blockingPressure, penetrationPressure);
                desired *= 1f - pressure * ContactSlowdown;
                var speed = math.length(desired);
                var direction = speed > 1e-5f ? desired / speed : float2.zero;
                // Direct/manual control owns the intended velocity. Collision constraints still
                // resolve overlap and mass, but predictive steering must not swallow the input.
                var avoidanceWeight = directControl ? 0f : math.max(0f, AvoidanceWeights[index]);
                FixedList128Bytes<Neighbor> neighbors = default;
                var regularSearchDistance = Radii[index] > LargeRadiusThreshold
                    ? math.max(NeighborDistance,
                        (Radii[index] + LargeRadiusThreshold) * PreferredSeparationMultiplier)
                    : NeighborDistance;
                Collect(index, position, regularSearchDistance, InverseCellSize, Positions, Radii, Active,
                    DirectControl, StableContactResolution, Layers, CollisionMasks, Grid,
                    MaximumNeighbors, MaximumCandidateChecks, ref neighbors,
                    out var candidateChecks, out var sameCellCandidateChecks,
                    out var candidateLimitReached);
                CollectLarge(index, position, NeighborDistance, PreferredSeparationMultiplier,
                    Positions, Radii, Active, DirectControl, StableContactResolution,
                    Layers, CollisionMasks, LargeAgents, MaximumNeighbors, ref neighbors);
                CollectRequired(index, position, NeighborDistance, PreferredSeparationMultiplier,
                    Positions, Radii, Active, Layers, CollisionMasks, RequiredAgents,
                    MaximumNeighbors, ref neighbors);
#if ENABLE_DIAGNOSTICS_LOG
                DiagnosticCandidateChecks[index] = candidateChecks;
                DiagnosticRetainedNeighborCounts[index] = neighbors.Length;
                DiagnosticSameCellCandidateChecks[index] = sameCellCandidateChecks;
                DiagnosticCandidateLimitReached[index] = candidateLimitReached;
#endif
                var retainedContact = PreviousContacts[index];
                if (retainedContact.HasConstraint != 0 && retainedContact.ConstraintIsDominant != 0 &&
                    (uint)retainedContact.ConstraintAgentIndex < (uint)Active.Length &&
                    Active[retainedContact.ConstraintAgentIndex] != 0)
                {
                    var retained = retainedContact.ConstraintAgentIndex;
                    var retainedDistance = math.distance(position, Positions[retained]);
                    InsertRequired(ref neighbors, new Neighbor
                    {
                        Index = retained,
                        SurfaceDistance = retainedDistance - Radii[index] - Radii[retained]
                    }, MaximumNeighbors);
                }
                var cache = default(NeighborCacheEntry);
                for (var i = 0; i < neighbors.Length; i++) cache.Indices.Add(neighbors[i].Index);
                NeighborCache[index] = cache;
#if ENABLE_DIAGNOSTICS_LOG
                DiagnosticCachedNeighbors[index] = cache.Indices;
#endif
                var separation = float2.zero;
                var lateralFlow = float2.zero;
                var lateralFlowWeight = 0f;
                var nearestAgentCollisionTime = float.PositiveInfinity;
                var nearestObstacleCollisionTime = float.PositiveInfinity;
                var nearestObstacleCandidateSide = 1f;
                for (var i = 0; i < neighbors.Length; i++)
                {
                    var n = neighbors[i];
                    // A breakthrough agent must retain its intended speed through the regular
                    // crowd. Keep the neighbor cached for contact/depenetration so the lower
                    // priority body is pushed aside, but do not steer or brake for it here.
                    var otherPriority = DirectControl[n.Index] != 0 || StableContactResolution[n.Index] != 0
                        ? (byte)2
                        : AvoidancePriorities[n.Index];
                    if (priority > otherPriority) continue;
                    var delta = position - Positions[n.Index];
                    var distanceSqr = math.lengthsq(delta);
                    var distance = math.sqrt(math.max(distanceSqr, 1e-8f));
                    var normal = distanceSqr > 1e-8f ? delta / distance : StableDirection(index, n.Index);
                    var preferred = (Radii[index] + Radii[n.Index]) * PreferredSeparationMultiplier;
                    if (distance < preferred) separation += normal * math.saturate((preferred - distance) / preferred);
                    var selfVelocity = math.lengthsq(CurrentVelocities[index]) > 1e-6f
                        ? CurrentVelocities[index]
                        : desired;
                    var otherVelocity = math.lengthsq(CurrentVelocities[n.Index]) > 1e-6f
                        ? CurrentVelocities[n.Index]
                        : DesiredVelocities[n.Index];
                    var relativeVelocity = selfVelocity - otherVelocity;
                    var closingSpeed = -math.dot(normal, relativeVelocity);
                    var isClosing = closingSpeed > 1e-4f;
                    var forwardDot = math.dot(-normal, direction);
                    if (speed > 1e-5f && isClosing && forwardDot > .25f)
                    {
                        var surfaceDistance = math.max(0f,
                            distance - Radii[index] - Radii[n.Index]);
                        nearestAgentCollisionTime = math.min(nearestAgentCollisionTime,
                            surfaceDistance / closingSpeed);
                    }

                    var otherDirection = math.normalizesafe(DesiredVelocities[n.Index]);
                    if (speed > 1e-5f && forwardDot > .25f &&
                        math.dot(direction, otherDirection) > .75f)
                    {
                        var weight = math.saturate(1f - distance / regularSearchDistance);
                        lateralFlow += (otherVelocity - direction * math.dot(otherVelocity, direction)) * weight;
                        lateralFlowWeight += weight;
                    }
                }
                for (var obstacleIndex = 0; obstacleIndex < ObstacleCount; obstacleIndex++)
                {
                    var obstacle = Obstacles[obstacleIndex];
                    if ((CollisionMasks[index] & obstacle.Layer) == 0 ||
                        (obstacle.CollidesWith & Layers[index]) == 0) continue;
                    var closest = obstacle.Shape == ObstacleShape.Circle
                        ? obstacle.PointA
                        : ClosestPoint(position, obstacle.PointA, obstacle.PointB);
                    var delta = position - closest;
                    var distanceSqr = math.lengthsq(delta);
                    var normal = distanceSqr > 1e-8f
                        ? delta * math.rsqrt(distanceSqr)
                        : StableDirection(index, -obstacleIndex - 1);
                    var forwardDot = math.dot(-normal, direction);
                    var closingSpeed = -math.dot(normal, desired);
                    if (speed <= 1e-5f || forwardDot <= .25f || closingSpeed <= 1e-4f) continue;
                    var surfaceDistance = math.max(0f,
                        math.sqrt(math.max(distanceSqr, 1e-8f)) - Radii[index] - obstacle.Radius);
                    var collisionTime = surfaceDistance / closingSpeed;
                    if (collisionTime >= nearestObstacleCollisionTime) continue;
                    nearestObstacleCollisionTime = collisionTime;
                    var perpendicular = new float2(-direction.y, direction.x);
                    float lateralDot;
                    if (obstacle.Shape == ObstacleShape.Segment)
                    {
                        var pointAOffset = obstacle.PointA - position;
                        var pointBOffset = obstacle.PointB - position;
                        var nearerEndpoint = math.lengthsq(pointAOffset) <= math.lengthsq(pointBOffset)
                            ? pointAOffset
                            : pointBOffset;
                        lateralDot = math.dot(nearerEndpoint, perpendicular);
                    }
                    else
                    {
                        lateralDot = math.dot(position - obstacle.PointA, perpendicular);
                    }
                    if (math.abs(lateralDot) <= 1e-5f)
                        lateralDot = math.dot(StableDirection(index, -obstacleIndex - 1), perpendicular);
                    nearestObstacleCandidateSide = lateralDot >= 0f ? 1f : -1f;
                }
                var retainedObstacleSide = ObstacleAvoidanceSides[index];
                var obstacleRetentionTime = math.max(0f,
                    ObstacleAvoidanceRetentionTimes[index] - DeltaTime);
                var nearestObstacleSide = nearestObstacleCandidateSide;
                if (!float.IsPositiveInfinity(nearestObstacleCollisionTime))
                {
                    if (retainedObstacleSide != 0 && obstacleRetentionTime > 0f)
                        nearestObstacleSide = retainedObstacleSide;
                    else
                        retainedObstacleSide = (sbyte)(nearestObstacleCandidateSide >= 0f ? 1 : -1);
                    obstacleRetentionTime = CollisionPredictionTime + .25f;
                }
                else if (obstacleRetentionTime <= 0f)
                {
                    retainedObstacleSide = 0;
                }
                ObstacleAvoidanceSides[index] = retainedObstacleSide;
                ObstacleAvoidanceRetentionTimes[index] = obstacleRetentionTime;
                var agentDetectedScale = float.IsPositiveInfinity(nearestAgentCollisionTime) ? 1f :
                    math.saturate(nearestAgentCollisionTime / CollisionPredictionTime);
                var obstacleDetectedScale = float.IsPositiveInfinity(nearestObstacleCollisionTime) ? 1f :
                    math.saturate(nearestObstacleCollisionTime / CollisionPredictionTime);
                var detectedScale = math.min(agentDetectedScale, obstacleDetectedScale);
                var scale = math.lerp(1f, detectedScale, math.saturate(avoidanceWeight));
                // Preserve the penetration ratio accumulated in separation. Normalizing the
                // vector unconditionally turns even floating-point noise at the preferred
                // separation boundary into a full-strength separation impulse.
                var separationMagnitude = math.length(separation);
                var avoidance = math.normalizesafe(separation) * math.saturate(separationMagnitude) *
                                (speed * SeparationSpeedRatio * avoidanceWeight);
                if (speed > 1e-5f && agentDetectedScale < 1f && avoidanceWeight > 0f)
                {
                    // Use the same relative passing side for every agent. Opposing directions
                    // then produce opposite world-space lateral velocities and can pass each
                    // other. Index parity can make both agents sidestep in the same world-space
                    // direction, which deadlocks a head-on stream.
                    // While routing around an obstacle, keep agent-agent passing on the same
                    // side. Otherwise the global passing convention can cancel lower-side
                    // obstacle steering while reinforcing upper-side steering.
                    var side = retainedObstacleSide != 0 && obstacleRetentionTime > 0f
                        ? retainedObstacleSide
                        : 1f;
                    avoidance += new float2(-direction.y, direction.x) *
                                 (side * speed * LateralSpeedRatio *
                                  (1f - agentDetectedScale) * avoidanceWeight);
                }
                if (speed > 1e-5f && obstacleDetectedScale < 1f && avoidanceWeight > 0f)
                    avoidance += new float2(-direction.y, direction.x) *
                                 (nearestObstacleSide * speed * LateralSpeedRatio *
                                  (1f - obstacleDetectedScale) * avoidanceWeight);
                if (lateralFlowWeight > 1e-5f && avoidanceWeight > 0f)
                    avoidance += lateralFlow / lateralFlowWeight *
                                 (LateralFlowFollowing * avoidanceWeight);
                var targetVelocity = desired * scale + avoidance;
                if (directControl && DeltaTime > 1e-6f)
                {
                    // Immediate input normally resets velocity every frame. Project that input
                    // against every nearby blocking agent before integration; positional
                    // depenetration is capped and cannot undo a large frame displacement after
                    // the controlled body has already crossed a crowd member.
                    for (var projectionPass = 0; projectionPass < 4; projectionPass++)
                    {
                        for (var neighborIndex = 0; neighborIndex < neighbors.Length; neighborIndex++)
                        {
                            var other = neighbors[neighborIndex].Index;
                            if (AvoidancePriorities[other] < priority) continue;
                            var delta = position - Positions[other];
                            var distanceSqr = math.lengthsq(delta);
                            var distance = math.sqrt(math.max(distanceSqr, 1e-8f));
                            var normal = distanceSqr > 1e-8f
                                ? delta / distance
                                : StableDirection(index, other);
                            var otherVelocity = math.lengthsq(CurrentVelocities[other]) > 1e-6f
                                ? CurrentVelocities[other]
                                : DesiredVelocities[other];
                            // Incoming crowd motion is resolved on that crowd member below; it
                            // must not carry the controlled body backward before integration.
                            // Motion away from the body may still extend the available movement.
                            var otherNormalSpeed = math.min(0f, math.dot(otherVelocity, normal));
                            var minimumDistance = (Radii[index] + Radii[other]) *
                                                  MinimumSpacingRatio;
                            var minimumNormalSpeed = otherNormalSpeed +
                                                     (math.min(minimumDistance, distance) - distance) /
                                                     DeltaTime;
                            var normalSpeed = math.dot(targetVelocity, normal);
                            if (normalSpeed < minimumNormalSpeed)
                                targetVelocity += normal * (minimumNormalSpeed - normalSpeed);
                        }
                    }
                }
                var response = immediate ? 1f : 1f - math.exp(-VelocityResponse * DeltaTime);
                // Do not let an idle stable body coast on contact velocity inherited from a
                // previous frame. Surrounding agents would otherwise carry it in alternating
                // directions before positional correction pulls it back.
                var currentVelocity = idleStable ? float2.zero : CurrentVelocities[index];
                var velocity = math.lerp(currentVelocity, targetVelocity, response);
                var resolved = position + velocity * DeltaTime;
                ConstrainObstacles(index, ref resolved, ref velocity, Radii[index], Layers[index],
                    CollisionMasks[index], Obstacles, ObstacleCount, false, out _);
                ResolvedPositions[index] = resolved;
                ResolvedVelocities[index] = velocity;
            }
        }

        [BurstCompile]
        private struct ConstraintJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float2> Positions;
            [ReadOnly] public NativeArray<float> Radii;
            [ReadOnly] public NativeArray<float> Masses;
            [ReadOnly] public NativeArray<byte> AvoidancePriorities;
            [ReadOnly] public NativeArray<float> CorrectionVelocityWeights;
            [ReadOnly] public NativeArray<float> MaximumCorrectionSpeeds;
            [ReadOnly] public NativeArray<uint> Layers, CollisionMasks;
            [ReadOnly] public NativeArray<uint> ContactEventMasks;
            [ReadOnly] public NativeArray<byte> Active;
            [ReadOnly] public NativeArray<byte> DirectControl;
            [ReadOnly] public NativeArray<byte> StableContactResolution;
            [ReadOnly] public NativeArray<Obstacle> Obstacles;
            [ReadOnly] public NativeArray<NeighborCacheEntry> NeighborCache;
            [WriteOnly] public NativeArray<float2> CorrectedPositions;
            [ReadOnly] public NativeArray<float2> DesiredVelocities;
            [ReadOnly] public NativeArray<float2> InputVelocities;
            public NativeArray<float2> Velocities;
            public NativeArray<AgentContactState> Contacts;
            public NativeParallelHashSet<ulong>.ParallelWriter ContactPairs;
            public int ObstacleCount;
            public byte WriteContacts;
            public float MinimumSpacingRatio, MaximumCorrectionRatio;
            public float ContactSkinRatio, ContactRetentionSkinMultiplier;
            public float DominantMassRatioThreshold;
            public float DeltaTime, CorrectionVelocityInfluence;
            public float InverseSolverIterations;
            public int SolverIteration, SolverIterationCount;
#if ENABLE_DIAGNOSTICS_LOG
            // Each iteration owns a disjoint [iteration * capacity, capacity] slice.
            // The job safety system cannot infer that mapping from the parallel index.
            [WriteOnly, NativeDisableParallelForRestriction]
            public NativeArray<float2> DiagnosticSolverPositions;
            [WriteOnly, NativeDisableParallelForRestriction]
            public NativeArray<float2> DiagnosticSolverCorrections;
            public int DiagnosticCapacity;
#endif

            public void Execute(int index)
            {
                if (Active[index] == 0) return;
                var position = Positions[index];
                var directControl = DirectControl[index] != 0;
                var stableContactResolution = StableContactResolution[index] != 0;
                var neighbors = NeighborCache[index].Indices;
                var correction = float2.zero;
                var strongestCorrection = float2.zero;
                var strongestPenetration = 0f;
                var strongestNeighbor = int.MaxValue;
                var strongestOtherMass = 0f;
                var strongestNormal = float2.zero;
                var strongestOtherNormalSpeed = 0f;
                var strongestBlocksMovement = false;
                var retainedCorrection = float2.zero;
                var retainedPenetration = 0f;
                var retainedNeighbor = int.MaxValue;
                var retainedOtherMass = 0f;
                var retainedNormal = float2.zero;
                var retainedOtherNormalSpeed = 0f;
                var retainedBlocksMovement = false;
                var penetrationContactCount = 0;
                var dominantCorrection = float2.zero;
                var dominantMassRatio = 0f;
                var dominantPenetration = 0f;
                var dominantNeighbor = int.MaxValue;
                var dominantOtherMass = 0f;
                var dominantNormal = float2.zero;
                var dominantOtherNormalSpeed = 0f;
                var dominantBlocksMovement = false;
                var dominantIsRetained = false;
                var contact = default(AgentContactState);
                var previousContact = Contacts[index];
                var idleStable = stableContactResolution && !directControl &&
                                 math.lengthsq(DesiredVelocities[index]) <= 1e-8f;
                var constrainedVelocity = idleStable ? float2.zero : InputVelocities[index];
                var movementDirection = math.normalizesafe(DesiredVelocities[index]);
                var peerVelocityCorrection = float2.zero;
                var peerVelocityConstraintCount = 0;
                var priority = AvoidancePriorities[index];
                for (var i = 0; i < neighbors.Length; i++)
                {
                    var other = neighbors[i];
                    if (Active[other] == 0 || (CollisionMasks[index] & Layers[other]) == 0 ||
                        (CollisionMasks[other] & Layers[index]) == 0) continue;
                    var delta = position - Positions[other];
                    var distanceSqr = math.lengthsq(delta);
                    var combinedRadius = Radii[index] + Radii[other];
                    var minimum = combinedRadius * MinimumSpacingRatio;
                    var contactDistance = minimum + combinedRadius * ContactSkinRatio;
                    var distance = math.sqrt(math.max(distanceSqr, 1e-8f));
                    var retainedDominant = previousContact.HasConstraint != 0 &&
                                           previousContact.ConstraintIsDominant != 0 &&
                                           previousContact.ConstraintAgentIndex == other;
                    var retentionDistance = minimum +
                                            combinedRadius * ContactSkinRatio *
                                            ContactRetentionSkinMultiplier;
                    var actualContact = distance <= contactDistance;
                    if (!actualContact && (!retainedDominant || distance > retentionDistance)) continue;
                    var normal = distanceSqr > 1e-8f
                        ? delta / distance
                        : StableDirection(index, other);
                    var otherPriority = AvoidancePriorities[other];
                    var blocksMovement = otherPriority >= priority;
                    if (actualContact)
                    {
                        contact.AgentContactCount++;
                        if (blocksMovement && math.dot(-normal, movementDirection) > .25f)
                        {
                            contact.BlockingAgentContactCount++;
                            contact.ForwardPenetrationPressure = math.max(
                                contact.ForwardPenetrationPressure,
                                math.saturate(math.max(0f, minimum - distance) /
                                              (combinedRadius * .25f)));
                        }
                        if (otherPriority == 0) contact.Priority0ContactCount++;
                        else if (otherPriority == 1) contact.Priority1ContactCount++;
                        else contact.Priority2ContactCount++;
                        contact.CombinedNormal += normal;
                    }
                    if (actualContact && WriteContacts != 0 &&
                        ((ContactEventMasks[index] & Layers[other]) != 0 ||
                         (ContactEventMasks[other] & Layers[index]) != 0))
                    {
                        var first = math.min(index, other);
                        var second = math.max(index, other);
                        ContactPairs.Add(((ulong)(uint)first << 32) | (uint)second);
                    }
                    // Priority is resolved before mass: stable gameplay-controlled bodies (2)
                    // outrank breakthrough agents (1), which outrank the regular crowd (0).
                    // Equal-priority dynamic bodies retain normal mass sharing.
                    float correctionShare;
                    if (priority > otherPriority) correctionShare = 0f;
                    else if (priority < otherPriority) correctionShare = 1f;
                    else if (priority >= 2) correctionShare = .5f;
                    else
                    {
                        var inverseMass = 1f / math.max(Masses[index], .0001f);
                        var otherInverseMass = 1f / math.max(Masses[other], .0001f);
                        correctionShare = inverseMass / math.max(.0001f, inverseMass + otherInverseMass);
                    }
                    var otherIdleStable = StableContactResolution[other] != 0 &&
                                          DirectControl[other] == 0 &&
                                          math.lengthsq(DesiredVelocities[other]) <= 1e-8f;
                    if (idleStable != otherIdleStable)
                    {
                        // An idle stable body is a static positional boundary. Correcting it
                        // against one deepest neighbor makes a surrounded body alternate between
                        // opposing normals; correcting the dynamic neighbor removes penetration
                        // without moving the idle body. Direct input immediately disables this.
                        correctionShare = idleStable ? 0f : 1f;
                    }
                    else
                    {
                        var stableBody = directControl || stableContactResolution;
                        var otherStableBody = DirectControl[other] != 0 ||
                                              StableContactResolution[other] != 0;
                        if (stableBody != otherStableBody)
                        {
                            // During direct control, priority alone assigns every correction to
                            // the controlled body even when the crowd moved into it. Attribute
                            // positional correction to the velocity that actually closed the pair:
                            // player motion moves the player back; crowd motion moves that crowd
                            // member back. This prevents both crowd pushing and retained overlap.
                            var selfApproach = math.max(0f,
                                -math.dot(InputVelocities[index], normal));
                            var otherApproach = math.max(0f,
                                math.dot(InputVelocities[other], normal));
                            var totalApproach = selfApproach + otherApproach;
                            if (totalApproach > 1e-5f)
                                correctionShare = selfApproach / totalApproach;
                        }
                    }
                    var otherMassRatio = Masses[other] / math.max(Masses[index], .0001f);
                    var dominantStableContact = (directControl || stableContactResolution) &&
                                                otherMassRatio >= DominantMassRatioThreshold;
                    // Constrain inward velocity throughout the contact skin, not only after
                    // penetration. This prevents direct-controlled agents from repeatedly
                    // entering and being corrected out of the same contact each frame.
                    var relativeNormalSpeed = math.dot(
                        constrainedVelocity - InputVelocities[other], normal);
                    // A dominant body must not remotely carry a stable agent while there is still
                    // a visible gap. Its velocity is transferred only at the geometric boundary.
                    if (!idleStable && blocksMovement &&
                        (!dominantStableContact || distance <= minimum))
                    {
                        if (priority == 1 && otherPriority == 1)
                        {
                            // Sequential projection against several peer normals is order
                            // dependent and can reverse the resulting velocity each frame. Build
                            // one Jacobi-style average from the immutable velocity snapshot.
                            var peerRelativeNormalSpeed = math.dot(
                                InputVelocities[index] - InputVelocities[other], normal);
                            if (peerRelativeNormalSpeed < 0f)
                            {
                                peerVelocityCorrection -= normal *
                                                          (peerRelativeNormalSpeed * correctionShare);
                                peerVelocityConstraintCount++;
                            }
                        }
                        else if (relativeNormalSpeed < 0f)
                            constrainedVelocity -= normal * (relativeNormalSpeed * correctionShare);
                    }
                    var rawOtherNormalSpeed = math.dot(InputVelocities[other], normal);
                    var otherNormalSpeed = dominantStableContact && distance > minimum
                        ? math.min(0f, rawOtherNormalSpeed)
                        : rawOtherNormalSpeed;
                    var penetration = math.max(0f, minimum - distance);
                    var pairCorrection = normal * (penetration * correctionShare);
                    if (penetration > 0f)
                    {
                        correction += pairCorrection;
                        penetrationContactCount++;
                    }
                    if (previousContact.HasConstraint != 0 &&
                        previousContact.ConstraintAgentIndex == other && penetration > 0f)
                    {
                        retainedCorrection = pairCorrection;
                        retainedPenetration = penetration;
                        retainedNeighbor = other;
                        retainedOtherMass = Masses[other];
                        retainedNormal = normal;
                        retainedOtherNormalSpeed = otherNormalSpeed;
                        retainedBlocksMovement = blocksMovement;
                    }
                    if (otherMassRatio >= DominantMassRatioThreshold &&
                        (retainedDominant && !dominantIsRetained ||
                         retainedDominant == dominantIsRetained &&
                        (otherMassRatio > dominantMassRatio + 1e-6f ||
                         math.abs(otherMassRatio - dominantMassRatio) <= 1e-6f &&
                         (penetration > dominantPenetration + 1e-6f ||
                          math.abs(penetration - dominantPenetration) <= 1e-6f &&
                          other < dominantNeighbor))))
                    {
                        dominantIsRetained = retainedDominant;
                        dominantMassRatio = otherMassRatio;
                        dominantPenetration = penetration;
                        dominantNeighbor = other;
                        dominantOtherMass = Masses[other];
                        dominantNormal = normal;
                        dominantOtherNormalSpeed = otherNormalSpeed;
                        dominantBlocksMovement = blocksMovement;
                        dominantCorrection = pairCorrection;
                    }
                    // At the center of a symmetric crowd the summed vectors cancel out even though
                    // the agent is deeply overlapping. Keep one deterministic escape direction.
                    if (penetration > strongestPenetration + 1e-6f ||
                        math.abs(penetration - strongestPenetration) <= 1e-6f && other < strongestNeighbor)
                    {
                        strongestPenetration = penetration;
                        strongestNeighbor = other;
                        strongestOtherMass = Masses[other];
                        strongestNormal = normal;
                        strongestOtherNormalSpeed = otherNormalSpeed;
                        strongestBlocksMovement = blocksMovement;
                        strongestCorrection = pairCorrection;
                    }
                }
                if (peerVelocityConstraintCount > 0)
                    constrainedVelocity += peerVelocityCorrection / peerVelocityConstraintCount;

                // A parallel Jacobi iteration applies every neighbor's response at once. Adding
                // all responses makes dense agents overshoot, after which the next iteration
                // applies almost the exact opposite correction. Averaging keeps the response
                // inside the local constraint set and lets successive iterations converge.
                if (!directControl && !stableContactResolution && penetrationContactCount > 1)
                    correction /= penetrationContactCount;

                // A manually controlled agent must not alternate between the summed normals of
                // different crowd members on consecutive frames. Resolve its deepest overlap;
                // the velocity constraints above still remove every inward normal component and
                // retain tangential input. Autonomous agents keep the aggregate crowd correction.
                if (directControl || stableContactResolution)
                {
                    if (dominantMassRatio >= DominantMassRatioThreshold)
                    {
                        correction = dominantCorrection;
                        strongestOtherMass = dominantOtherMass;
                        strongestNormal = dominantNormal;
                        strongestOtherNormalSpeed = dominantOtherNormalSpeed;
                        strongestBlocksMovement = dominantBlocksMovement;
                    }
                    else if (retainedNeighbor != int.MaxValue &&
                             retainedPenetration >= strongestPenetration * .75f)
                    {
                        // Several similarly deep contacts frequently exchange first place in a
                        // dense crowd. Retain last frame's constraint until it is materially
                        // weaker, otherwise the escape normal alternates every frame.
                        correction = retainedCorrection;
                        strongestPenetration = retainedPenetration;
                        strongestNeighbor = retainedNeighbor;
                        strongestOtherMass = retainedOtherMass;
                        strongestNormal = retainedNormal;
                        strongestOtherNormalSpeed = retainedOtherNormalSpeed;
                        strongestBlocksMovement = retainedBlocksMovement;
                    }
                    else correction = strongestCorrection;
                }
                var maxCorrection = Radii[index] * MaximumCorrectionRatio;
                var maximumCorrectionSpeed = MaximumCorrectionSpeeds[index];
                if (maximumCorrectionSpeed > 0f)
                {
                    // A dominant agent must be able to carry a lighter stable agent out of its
                    // path. Keep the normal crowd cap low, but scale it for a large mass gap.
                    var massRatio = strongestOtherMass / math.max(Masses[index], .0001f);
                    if (massRatio >= DominantMassRatioThreshold)
                        maximumCorrectionSpeed *= math.min(5f, math.sqrt(massRatio));
                    // A frame-time spike must not multiply the visible depenetration jump.
                    // Movement still integrates the full DeltaTime; only the solver impulse is capped.
                    var correctionDeltaTime = math.min(DeltaTime, 1f / 30f);
                    maxCorrection = math.min(maxCorrection,
                        maximumCorrectionSpeed * correctionDeltaTime * InverseSolverIterations);
                }
                var lengthSqr = math.lengthsq(correction);
                if (contact.AgentContactCount > 0 && lengthSqr < 1e-8f)
                {
                    correction = strongestCorrection;
                    lengthSqr = math.lengthsq(correction);
                }
                if (lengthSqr > maxCorrection * maxCorrection)
                    correction *= maxCorrection * math.rsqrt(lengthSqr);
                // Stable/direct-controlled bodies use a temporally retained single constraint.
                // Apply this after the zero-correction fallback above; otherwise that fallback
                // restores strongestCorrection and makes the second iteration move again.
                if ((directControl || stableContactResolution) && SolverIteration > 0)
                    correction = float2.zero;
#if ENABLE_DIAGNOSTICS_LOG
                if (SolverIteration < LocalAvoidanceDiagnostics.MaximumSolverIterations)
                {
                    var diagnosticIndex = SolverIteration * DiagnosticCapacity + index;
                    DiagnosticSolverCorrections[diagnosticIndex] = correction;
                    DiagnosticSolverPositions[diagnosticIndex] = position + correction;
                }
#endif
                position += correction;
                var positionBeforeObstacle = position;
                var dummyVelocity = float2.zero;
                ConstrainObstacles(index, ref position, ref dummyVelocity, Radii[index], Layers[index],
                    CollisionMasks[index], Obstacles, ObstacleCount, true, out var obstacleContact);
                contact.ObstacleContactCount = obstacleContact.ObstacleContactCount;
                contact.CombinedNormal += obstacleContact.CombinedNormal;
                contact.IsTouching = (byte)((contact.AgentContactCount + contact.ObstacleContactCount) > 0 ? 1 : 0);
                var correctionVelocityWeight = math.max(0f, CorrectionVelocityWeights[index]);
                if (!directControl && correctionVelocityWeight > 0f &&
                    DeltaTime > 1e-6f && CorrectionVelocityInfluence > 0f)
                {
                    var totalCorrection = correction + (position - positionBeforeObstacle);
                    constrainedVelocity += totalCorrection *
                                           (CorrectionVelocityInfluence * correctionVelocityWeight / DeltaTime);
                }
                if (!directControl)
                {
                    // Contact projection and retained depenetration velocity must not accelerate
                    // an autonomous agent beyond its authored movement speed. In a compressed
                    // crowd that feedback otherwise raises a 4.8-unit target to 7-8 units and
                    // drives still more agents into the center.
                    var maximumSpeed = math.length(DesiredVelocities[index]);
                    var velocityLengthSq = math.lengthsq(constrainedVelocity);
                    if (maximumSpeed <= 1e-5f) constrainedVelocity = float2.zero;
                    else if (velocityLengthSq > maximumSpeed * maximumSpeed)
                        constrainedVelocity *= maximumSpeed * math.rsqrt(velocityLengthSq);
                }
                Velocities[index] = constrainedVelocity;
                if (WriteContacts != 0)
                {
                    contact.ConstraintNormal = strongestNormal;
                    contact.AllowedNormalSpeed = strongestOtherNormalSpeed;
                    contact.ConstraintAgentIndex = dominantMassRatio >= DominantMassRatioThreshold
                        ? dominantNeighbor
                        : strongestNeighbor;
                    contact.ConstraintOtherMass = strongestOtherMass;
                    contact.ConstraintOtherRadius = contact.ConstraintAgentIndex >= 0 &&
                                                    contact.ConstraintAgentIndex < Radii.Length
                        ? Radii[contact.ConstraintAgentIndex]
                        : 0f;
                    contact.ConstraintPenetration = dominantMassRatio >= DominantMassRatioThreshold
                        ? dominantPenetration
                        : strongestPenetration;
                    contact.CorrectionLimit = maxCorrection;
                    contact.HasConstraint = (byte)(contact.ConstraintAgentIndex != int.MaxValue ? 1 : 0);
                    contact.ConstraintBlocksMovement = (byte)(strongestBlocksMovement ? 1 : 0);
                    contact.ConstraintIsDominant = (byte)(dominantMassRatio >= DominantMassRatioThreshold ? 1 : 0);
                    Contacts[index] = contact;
                }
                CorrectedPositions[index] = position;
            }
        }

        [BurstCompile]
        private struct CopyPositionsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float2> Source;
            [WriteOnly] public NativeArray<float2> Destination;
            public void Execute(int index) => Destination[index] = Source[index];
        }

        private static void Collect(int index, float2 position, float distance, float inverseCellSize,
            NativeArray<float2> positions, NativeArray<float> radii, NativeArray<byte> active,
            NativeArray<byte> directControl, NativeArray<byte> stableContactResolution,
            NativeArray<uint> layers, NativeArray<uint> masks,
            NativeParallelMultiHashMap<int, int>.ReadOnly grid,
            int maximumNeighbors, int maximumCandidateChecks,
            ref FixedList128Bytes<Neighbor> result, out int candidateChecks,
            out int sameCellCandidateChecks, out byte candidateLimitReached)
        {
            var cell = Cell(position, inverseCellSize);
            var range = math.max(1, (int)math.ceil(distance * inverseCellSize));
            var distanceSqrLimit = distance * distance;
            candidateChecks = 0;
            sameCellCandidateChecks = 0;
            candidateLimitReached = 0;
            // A stable per-agent D4 transform distributes the bounded traversal's directional
            // bias without changing order between frames. Using one global lower-left-first
            // order makes every saturated agent omit the same side of the grid and produces
            // world-axis-aligned seams in the crowd.
            var traversalVariant = (int)(((uint)index * 747796405u + 2891336453u) >> 29);
            // Visit the agent's own cell first, then expand in square rings. With a bounded
            // candidate budget, scanning from the lower-left corner can exhaust the entire
            // budget in a remote dense cell and never inspect agents sharing this cell.
            for (var ring = 0; ring <= range; ring++)
            for (var offsetX = -ring; offsetX <= ring; offsetX++)
            for (var offsetY = -ring; offsetY <= ring; offsetY++)
            {
                if (math.max(math.abs(offsetX), math.abs(offsetY)) != ring) continue;
                var transformedOffset = TransformTraversalOffset(
                    new int2(offsetX, offsetY), traversalVariant);
                var x = cell.x + transformedOffset.x;
                var y = cell.y + transformedOffset.y;
                if (!grid.TryGetFirstValue(Key(new int2(x, y)), out var other, out var iterator)) continue;
                do
                {
                    if (candidateChecks >= maximumCandidateChecks)
                    {
                        candidateLimitReached = 1;
                        return;
                    }
                    candidateChecks++;
                    if (ring == 0) sameCellCandidateChecks++;
                    if (other == index || active[other] == 0 ||
                        (masks[index] & layers[other]) == 0 || (masks[other] & layers[index]) == 0) continue;
                    var distanceSqr = math.lengthsq(position - positions[other]);
                    if (distanceSqr >= distanceSqrLimit) continue;
                    InsertNearest(ref result, new Neighbor
                    {
                        Index = other,
                        SurfaceDistance = math.sqrt(distanceSqr) - radii[index] - radii[other],
                        Required = (byte)(directControl[other] != 0 ||
                                          stableContactResolution[other] != 0 ? 1 : 0)
                    }, maximumNeighbors);
                } while (grid.TryGetNextValue(out other, ref iterator));
            }
        }

        private static int2 TransformTraversalOffset(int2 offset, int variant)
        {
            // Reflection followed by quarter turns spans all eight symmetries of a square.
            if ((variant & 4) != 0) offset.x = -offset.x;
            return (variant & 3) switch
            {
                1 => new int2(-offset.y, offset.x),
                2 => -offset,
                3 => new int2(offset.y, -offset.x),
                _ => offset
            };
        }

        private static void CollectRequired(int index, float2 position, float distance,
            float preferredSeparationMultiplier,
            NativeArray<float2> positions, NativeArray<float> radii, NativeArray<byte> active,
            NativeArray<uint> layers, NativeArray<uint> masks, NativeList<int> requiredAgents,
            int maximumNeighbors, ref FixedList128Bytes<Neighbor> result)
        {
            for (var i = 0; i < requiredAgents.Length; i++)
            {
                var other = requiredAgents[i];
                if (other == index || active[other] == 0 ||
                    (masks[index] & layers[other]) == 0 ||
                    (masks[other] & layers[index]) == 0) continue;
                var combinedRadius = radii[index] + radii[other];
                var searchDistance = math.max(distance,
                    combinedRadius * preferredSeparationMultiplier);
                var distanceSqr = math.lengthsq(position - positions[other]);
                if (distanceSqr >= searchDistance * searchDistance) continue;
                InsertRequired(ref result, new Neighbor
                {
                    Index = other,
                    SurfaceDistance = math.sqrt(distanceSqr) - combinedRadius,
                    Required = 1
                }, maximumNeighbors);
            }
        }

        private static void CollectLarge(int index, float2 position, float distance,
            float preferredSeparationMultiplier,
            NativeArray<float2> positions, NativeArray<float> radii, NativeArray<byte> active,
            NativeArray<byte> directControl, NativeArray<byte> stableContactResolution,
            NativeArray<uint> layers, NativeArray<uint> masks, NativeList<int> largeAgents,
            int maximumNeighbors, ref FixedList128Bytes<Neighbor> result)
        {
            for (var i = 0; i < largeAgents.Length; i++)
            {
                var other = largeAgents[i];
                if (other == index || active[other] == 0 ||
                    (masks[index] & layers[other]) == 0 || (masks[other] & layers[index]) == 0) continue;

                var combinedRadius = radii[index] + radii[other];
                // Large agents must remain discoverable even when the regular grid uses a much
                // shorter neighbor distance. Match the same preferred separation policy used by
                // steering and regular-agent searches.
                var searchDistance = math.max(distance, combinedRadius * preferredSeparationMultiplier);
                var distanceSqr = math.lengthsq(position - positions[other]);
                if (distanceSqr >= searchDistance * searchDistance) continue;
                InsertNearest(ref result, new Neighbor
                {
                    Index = other,
                    SurfaceDistance = math.sqrt(distanceSqr) - combinedRadius,
                    Required = (byte)(directControl[other] != 0 ||
                                      stableContactResolution[other] != 0 ? 1 : 0)
                }, maximumNeighbors);
            }
        }

        private static void InsertNearest(ref FixedList128Bytes<Neighbor> values, Neighbor value,
            int maximumNeighbors)
        {
            if (values.Length < maximumNeighbors) values.Add(value);
            else if (ComesBefore(value, values[^1])) values[^1] = value;
            else return;
            for (var i = values.Length - 1; i > 0 && ComesBefore(values[i], values[i - 1]); i--)
            {
                (values[i - 1], values[i]) = (values[i], values[i - 1]);
            }
        }

        private static void InsertRequired(ref FixedList128Bytes<Neighbor> values, Neighbor value,
            int maximumNeighbors)
        {
            for (var i = 0; i < values.Length; i++)
                if (values[i].Index == value.Index)
                    return;
            if (values.Length < maximumNeighbors) values.Add(value);
            else values[^1] = value;
        }

        private static bool ComesBefore(Neighbor a, Neighbor b) =>
            a.Required > b.Required || a.Required == b.Required &&
            (a.SurfaceDistance < b.SurfaceDistance - 1e-6f ||
             math.abs(a.SurfaceDistance - b.SurfaceDistance) <= 1e-6f && a.Index < b.Index);

        private static void ConstrainObstacles(int index, ref float2 position, ref float2 velocity,
            float agentRadius, uint layer, uint mask, NativeArray<Obstacle> obstacles, int count,
            bool collectContact, out AgentContactState contact)
        {
            contact = default;
            for (var i = 0; i < count; i++)
            {
                var obstacle = obstacles[i];
                if ((mask & obstacle.Layer) == 0 || (obstacle.CollidesWith & layer) == 0) continue;
                var closest = obstacle.Shape == ObstacleShape.Circle
                    ? obstacle.PointA
                    : ClosestPoint(position, obstacle.PointA, obstacle.PointB);
                var delta = position - closest;
                var minimum = agentRadius + obstacle.Radius;
                var distanceSqr = math.lengthsq(delta);
                var contactDistance = minimum + (collectContact ? .0001f : 0f);
                if (distanceSqr > contactDistance * contactDistance) continue;
                var normal = distanceSqr > 1e-8f ? delta * math.rsqrt(distanceSqr) : StableDirection(index, -i - 1);
                if (distanceSqr < minimum * minimum)
                {
                    position = closest + normal * minimum;
                    var inward = math.dot(velocity, normal);
                    if (inward < 0f) velocity -= normal * inward;
                }
                if (!collectContact) continue;
                contact.ObstacleContactCount++;
                contact.CombinedNormal += normal;
            }
        }

        private static float2 ClosestPoint(float2 point, float2 a, float2 b)
        {
            var edge = b - a;
            var lengthSqr = math.lengthsq(edge);
            return lengthSqr <= 1e-8f ? a : a + edge * math.saturate(math.dot(point - a, edge) / lengthSqr);
        }

        private static float2 StableDirection(int first, int second)
        {
            var hash = math.hash(new int2(math.min(first, second), math.max(first, second)));
            var value = new float2(((hash & 1023u) / 511.5f) - 1f,
                (((hash >> 10) & 1023u) / 511.5f) - 1f);
            value = math.normalizesafe(value, new float2(1f, 0f));
            return first <= second ? value : -value;
        }

        private static int2 Cell(float2 position, float inverseCellSize) =>
            (int2)math.floor(position * inverseCellSize);
        private static int Key(int2 cell) => unchecked(cell.x * 73856093 ^ cell.y * 19349663);
    }
}
