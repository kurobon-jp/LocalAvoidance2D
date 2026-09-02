using Unity.Mathematics;

namespace LocalAvoidance2D
{
    public enum ObstacleShape : byte
    {
        Circle,
        Segment
    }

    public struct Obstacle
    {
        public ObstacleShape Shape;
        public float2 PointA;
        public float2 PointB;
        public float Radius;
        public uint Layer;
        public uint CollidesWith;

        public static Obstacle Circle(float2 position, float radius,
            uint layer = uint.MaxValue, uint collidesWith = uint.MaxValue) => new()
        {
            Shape = ObstacleShape.Circle,
            PointA = position,
            Radius = math.max(0f, radius),
            Layer = layer,
            CollidesWith = collidesWith
        };

        public static Obstacle Segment(float2 pointA, float2 pointB, float radius = 0f,
            uint layer = uint.MaxValue, uint collidesWith = uint.MaxValue) => new()
        {
            Shape = ObstacleShape.Segment,
            PointA = pointA,
            PointB = pointB,
            Radius = math.max(0f, radius),
            Layer = layer,
            CollidesWith = collidesWith
        };
    }

    public struct AgentContactState
    {
        public int AgentContactCount;
        /// <summary>Contacts whose avoidance priority is equal to or higher than this agent.</summary>
        public int BlockingAgentContactCount;
        /// <summary>
        /// Maximum normalized penetration among contacts in the desired movement direction.
        /// Reaches one at 25% of the pair's combined radius.
        /// </summary>
        public float ForwardPenetrationPressure;
        public int Priority0ContactCount;
        public int Priority1ContactCount;
        public int Priority2ContactCount;
        public int ObstacleContactCount;
        public float2 CombinedNormal;
        /// <summary>Primary non-penetration normal selected by the previous solver step.</summary>
        public float2 ConstraintNormal;
        /// <summary>Minimum velocity allowed along ConstraintNormal on the next movement step.</summary>
        public float AllowedNormalSpeed;
        public int ConstraintAgentIndex;
        public float ConstraintOtherMass;
        public float ConstraintOtherRadius;
        public float ConstraintPenetration;
        public float CorrectionLimit;
        public byte HasConstraint;
        /// <summary>Whether the selected agent was allowed to constrain this agent's velocity.</summary>
        public byte ConstraintBlocksMovement;
        public byte ConstraintIsDominant;
        public byte IsTouching;
    }

    /// <summary>A newly established agent contact. Indices refer to simulation slots.</summary>
    public struct AgentContactPair
    {
        public int AgentA;
        public int AgentB;
    }

    /// <summary>One agent intersected by a non-allocating 2D ray/capsule query.</summary>
    public struct RaycastHit
    {
        public int AgentIndex;
        public float Distance;
        public float2 Position;
    }

    public struct LocalAvoidanceSettings
    {
        /// <summary>
        /// Spatial-grid cell width. This has a large performance impact: cells that are too
        /// small increase the number searched, while cells that are too large increase the
        /// number of candidates in each cell.
        /// </summary>
        public float CellSize;
        /// <summary>
        /// Agent search distance. This has a large performance impact when increasing it crosses
        /// a cell boundary and expands the searched grid area. Values below CellSize are clamped
        /// to CellSize.
        /// </summary>
        public float NeighborDistance;
        /// <summary>
        /// Maximum number of nearest agents retained per agent. This has a large performance
        /// impact in dense crowds. Values are clamped to the FixedList-backed range of 1 to 10.
        /// </summary>
        public int MaximumNeighbors;
        /// <summary>
        /// Maximum regular grid entries inspected per agent before selecting nearest neighbors.
        /// Direct-controlled and stable agents are inspected separately and do not consume this limit.
        /// </summary>
        public int MaximumCandidateChecks;
        /// <summary>Seconds ahead used to predict collisions with moving agents.</summary>
        public float CollisionPredictionTime;
        public float VelocityResponse;
        public float SeparationSpeedRatio;
        public float LateralSpeedRatio;
        /// <summary>How strongly agents follow the lateral motion of agents ahead.</summary>
        public float LateralFlowFollowing;
        public float MinimumSpacingRatio;
        public float MaximumCorrectionRatio;
        public float ContactSlowdown;
        public float ContactsForMaximumSlowdown;
        public float ContactSkinRatio;
        /// <summary>Preferred separation multiplier used for neighbor discovery and steering.</summary>
        public float PreferredSeparationMultiplier;
        /// <summary>Multiplier applied to the contact skin while retaining a dominant contact.</summary>
        public float ContactRetentionSkinMultiplier;
        /// <summary>Mass ratio at which the other agent is treated as dominant.</summary>
        public float DominantMassRatioThreshold;
        public float CorrectionVelocityInfluence;
        /// <summary>
        /// Number of contact-solver passes. This has a large performance impact because every
        /// active agent is processed once per iteration; cost is approximately proportional to it.
        /// </summary>
        public int SolverIterations;
        /// <summary>
        /// IJobParallelFor batch size. This can materially affect parallel efficiency and should
        /// be measured on the target hardware; it does not reduce the amount of simulation work.
        /// </summary>
        public int InnerLoopBatchCount;

        public static LocalAvoidanceSettings Default => new()
        {
            CellSize = 1.2f,
            NeighborDistance = 1.2f,
            MaximumNeighbors = 8,
            MaximumCandidateChecks = 64,
            CollisionPredictionTime = .5f,
            VelocityResponse = 10f,
            SeparationSpeedRatio = .4f,
            LateralSpeedRatio = .15f,
            LateralFlowFollowing = .65f,
            MinimumSpacingRatio = .95f,
            MaximumCorrectionRatio = .25f,
            ContactSlowdown = .8f,
            ContactsForMaximumSlowdown = 6f,
            ContactSkinRatio = .1f,
            PreferredSeparationMultiplier = 1.2f,
            ContactRetentionSkinMultiplier = 2f,
            DominantMassRatioThreshold = 4f,
            CorrectionVelocityInfluence = .15f,
            SolverIterations = 2,
            InnerLoopBatchCount = 128
        };

        internal LocalAvoidanceSettings Sanitized()
        {
            CellSize = math.max(.01f, CellSize);
            NeighborDistance = math.max(CellSize, NeighborDistance);
            MaximumNeighbors = MaximumNeighbors > 0
                ? math.clamp(MaximumNeighbors, 1, 10)
                : 8;
            MaximumCandidateChecks = MaximumCandidateChecks > 0
                ? math.max(MaximumNeighbors, MaximumCandidateChecks)
                : 64;
            CollisionPredictionTime = CollisionPredictionTime > 0f
                ? CollisionPredictionTime
                : .5f;
            VelocityResponse = math.max(0f, VelocityResponse);
            SeparationSpeedRatio = math.max(0f, SeparationSpeedRatio);
            LateralSpeedRatio = math.max(0f, LateralSpeedRatio);
            LateralFlowFollowing = math.saturate(LateralFlowFollowing);
            MinimumSpacingRatio = math.clamp(MinimumSpacingRatio, .01f, 1f);
            MaximumCorrectionRatio = math.max(0f, MaximumCorrectionRatio);
            ContactSlowdown = math.saturate(ContactSlowdown);
            ContactsForMaximumSlowdown = math.max(1f, ContactsForMaximumSlowdown);
            ContactSkinRatio = math.max(0f, ContactSkinRatio);
            // Zero is the default value for callers using a partial struct initializer;
            // preserve the historical behavior in that case.
            PreferredSeparationMultiplier = PreferredSeparationMultiplier > 0f
                ? math.max(1f, PreferredSeparationMultiplier)
                : 1.2f;
            ContactRetentionSkinMultiplier = ContactRetentionSkinMultiplier > 0f
                ? math.max(1f, ContactRetentionSkinMultiplier)
                : 2f;
            DominantMassRatioThreshold = DominantMassRatioThreshold > 0f
                ? math.max(1f, DominantMassRatioThreshold)
                : 4f;
            CorrectionVelocityInfluence = math.max(0f, CorrectionVelocityInfluence);
            SolverIterations = math.clamp(SolverIterations, 1, 8);
            InnerLoopBatchCount = math.max(1, InnerLoopBatchCount);
            return this;
        }
    }
}
