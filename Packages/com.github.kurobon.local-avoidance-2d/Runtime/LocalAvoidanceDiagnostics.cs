using System;
using Unity.Collections;
using Unity.Mathematics;

namespace LocalAvoidance2D
{
    /// <summary>Optional diagnostic storage. Create it only while simulation diagnostics are required.</summary>
    public sealed class LocalAvoidanceDiagnostics : IDisposable
    {
        public const int MaximumSolverIterations = 8;

        public int Capacity { get; }
        public NativeArray<float2> DesiredBeforeConstraint { get; }
        public NativeArray<float2> DesiredAfterConstraint { get; }
        public NativeArray<float2> PreviousConstraintNormal { get; }
        public NativeArray<float> PreviousAllowedNormalSpeed { get; }
        public NativeArray<float2> SolverPositions { get; }
        public NativeArray<float2> SolverCorrections { get; }
        public NativeArray<byte> ConstraintApplied { get; }
        public NativeArray<int> CandidateChecks { get; }
        public NativeArray<int> RetainedNeighborCounts { get; }
        public NativeArray<int> SameCellCandidateChecks { get; }
        public NativeArray<byte> CandidateLimitReached { get; }
        internal NativeArray<FixedList128Bytes<int>> CachedNeighbors { get; }

        public LocalAvoidanceDiagnostics(int capacity, Allocator allocator = Allocator.Persistent)
        {
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));
            Capacity = capacity;
            DesiredBeforeConstraint = new NativeArray<float2>(capacity, allocator);
            DesiredAfterConstraint = new NativeArray<float2>(capacity, allocator);
            PreviousConstraintNormal = new NativeArray<float2>(capacity, allocator);
            PreviousAllowedNormalSpeed = new NativeArray<float>(capacity, allocator);
            SolverPositions = new NativeArray<float2>(capacity * MaximumSolverIterations, allocator);
            SolverCorrections = new NativeArray<float2>(capacity * MaximumSolverIterations, allocator);
            ConstraintApplied = new NativeArray<byte>(capacity, allocator);
            CandidateChecks = new NativeArray<int>(capacity, allocator);
            RetainedNeighborCounts = new NativeArray<int>(capacity, allocator);
            SameCellCandidateChecks = new NativeArray<int>(capacity, allocator);
            CandidateLimitReached = new NativeArray<byte>(capacity, allocator);
            CachedNeighbors = new NativeArray<FixedList128Bytes<int>>(capacity, allocator);
        }

        public float2 GetSolverPosition(int agentIndex, int iteration) =>
            SolverPositions[SolverIndex(agentIndex, iteration)];

        public float2 GetSolverCorrection(int agentIndex, int iteration) =>
            SolverCorrections[SolverIndex(agentIndex, iteration)];

        public int GetCachedNeighborCount(int agentIndex)
        {
            ValidateAgentIndex(agentIndex);
            return CachedNeighbors[agentIndex].Length;
        }

        public int GetCachedNeighbor(int agentIndex, int neighborIndex)
        {
            ValidateAgentIndex(agentIndex);
            var neighbors = CachedNeighbors[agentIndex];
            if ((uint)neighborIndex >= (uint)neighbors.Length)
                throw new ArgumentOutOfRangeException(nameof(neighborIndex));
            return neighbors[neighborIndex];
        }

        private int SolverIndex(int agentIndex, int iteration)
        {
            ValidateAgentIndex(agentIndex);
            if ((uint)iteration >= MaximumSolverIterations)
                throw new ArgumentOutOfRangeException(nameof(iteration));
            return iteration * Capacity + agentIndex;
        }

        private void ValidateAgentIndex(int agentIndex)
        {
            if ((uint)agentIndex >= (uint)Capacity)
                throw new ArgumentOutOfRangeException(nameof(agentIndex));
        }

        public void Dispose()
        {
            if (CachedNeighbors.IsCreated) CachedNeighbors.Dispose();
            if (CandidateLimitReached.IsCreated) CandidateLimitReached.Dispose();
            if (SameCellCandidateChecks.IsCreated) SameCellCandidateChecks.Dispose();
            if (RetainedNeighborCounts.IsCreated) RetainedNeighborCounts.Dispose();
            if (CandidateChecks.IsCreated) CandidateChecks.Dispose();
            if (ConstraintApplied.IsCreated) ConstraintApplied.Dispose();
            if (SolverCorrections.IsCreated) SolverCorrections.Dispose();
            if (SolverPositions.IsCreated) SolverPositions.Dispose();
            if (PreviousAllowedNormalSpeed.IsCreated) PreviousAllowedNormalSpeed.Dispose();
            if (PreviousConstraintNormal.IsCreated) PreviousConstraintNormal.Dispose();
            if (DesiredAfterConstraint.IsCreated) DesiredAfterConstraint.Dispose();
            if (DesiredBeforeConstraint.IsCreated) DesiredBeforeConstraint.Dispose();
        }
    }
}
