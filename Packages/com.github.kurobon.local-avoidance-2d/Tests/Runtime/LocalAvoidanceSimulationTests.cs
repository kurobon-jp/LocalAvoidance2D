using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace LocalAvoidance2D.Tests
{
    public sealed class LocalAvoidanceSimulationTests
    {
        [Test]
        public void NewAgentSlotsHaveStandardDefaultsAndRemainInactive()
        {
            using var simulation = Create(2);

            for (var i = 0; i < simulation.Capacity; i++)
            {
                Assert.That(simulation.Masses[i], Is.EqualTo(1f));
                Assert.That(simulation.AvoidanceWeights[i], Is.EqualTo(1f));
                Assert.That(simulation.CorrectionVelocityWeights[i], Is.EqualTo(1f));
                Assert.That(simulation.Layers[i], Is.EqualTo(1u));
                Assert.That(simulation.CollisionMasks[i], Is.EqualTo(1u));
                Assert.That(simulation.Active[i], Is.Zero);
            }
        }

        [Test]
        public void ActivateAgentSetsRequiredInputsAndActivatesSlot()
        {
            using var simulation = Create(1);
            var position = new float2(2f, 3f);
            var desiredVelocity = new float2(4f, 5f);

            simulation.ActivateAgent(0, position, desiredVelocity, .6f);

            Assert.That(simulation.Positions[0], Is.EqualTo(position));
            Assert.That(simulation.ResolvedPositions[0], Is.EqualTo(position));
            Assert.That(simulation.DesiredVelocities[0], Is.EqualTo(desiredVelocity));
            Assert.That(simulation.Radii[0], Is.EqualTo(.6f));
            Assert.That(simulation.Active[0], Is.EqualTo(1));
        }

        [Test]
        public void CapacityStepUsesAllocatedAgentAndObstacleRanges()
        {
            using var simulation = Create(2, 1);
            simulation.ActivateAgent(1, float2.zero, float2.zero, .5f);
            var obstacles = simulation.Obstacles;
            obstacles[0] = Obstacle.Circle(float2.zero, 1f, 1u, 1u);

            simulation.Step(.01f);

            Assert.That(math.length(simulation.ResolvedPositions[1]), Is.EqualTo(1.5f).Within(.001f));
            Assert.That(simulation.Contacts[1].ObstacleContactCount, Is.EqualTo(1));
        }

        [Test]
        public void TeleportSynchronizesPositionAndClearsTransientState()
        {
            using var simulation = Create(1);
            SetAgent(simulation, 0, float2.zero, new float2(1f, 0f), .25f);
            var currentVelocities = simulation.CurrentVelocities;
            currentVelocities[0] = new float2(2f, 3f);
            var contacts = simulation.Contacts;
            contacts[0] = new AgentContactState { IsTouching = 1, HasConstraint = 1 };
            var sides = simulation.ObstacleAvoidanceSides;
            sides[0] = -1;

            simulation.Teleport(0, new float2(5f, 7f));

            Assert.That(simulation.Positions[0], Is.EqualTo(new float2(5f, 7f)));
            Assert.That(simulation.ResolvedPositions[0], Is.EqualTo(new float2(5f, 7f)));
            Assert.That(simulation.MovedPositions[0], Is.EqualTo(new float2(5f, 7f)));
            Assert.That(simulation.CurrentVelocities[0], Is.EqualTo(float2.zero));
            Assert.That(simulation.ResolvedVelocities[0], Is.EqualTo(float2.zero));
            Assert.That(simulation.Contacts[0].IsTouching, Is.Zero);
            Assert.That(simulation.Contacts[0].HasConstraint, Is.Zero);
            Assert.That(simulation.ObstacleAvoidanceSides[0], Is.Zero);
        }

        [Test]
        public void RaycastUsesBidirectionalMasksWithoutRequiringSameLayer()
        {
            using var simulation = Create(1);
            SetAgent(simulation, 0, new float2(2f, 0f), float2.zero, .25f);
            var layers = simulation.Layers;
            var masks = simulation.CollisionMasks;
            layers[0] = 1u << 1;
            masks[0] = 1u << 0;
            using var hits = new NativeArray<RaycastHit>(1, Allocator.Temp);

            var count = simulation.Raycast(float2.zero, new float2(1f, 0f), 4f, 0f,
                1u << 0, 1u << 1, hits);

            Assert.That(count, Is.EqualTo(1));
            Assert.That(hits[0].AgentIndex, Is.Zero);
        }

        [Test]
        public void SingleAgentMovesAlongDesiredVelocity()
        {
            using var simulation = Create(1);
            SetAgent(simulation, 0, float2.zero, new float2(2f, 0f), .25f);
            var immediateVelocity = simulation.ImmediateVelocity;
            immediateVelocity[0] = 1;
            simulation.Step(.5f, 1);
            Assert.That(simulation.ResolvedPositions[0].x, Is.EqualTo(1f).Within(.001f));
        }

        [Test]
        public void CoincidentAgentsAreSeparatedWithoutNaN()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            SetAgent(simulation, 1, float2.zero, float2.zero, .5f);
            simulation.Step(1f / 60f, 2);
            Assert.That(math.all(math.isfinite(simulation.ResolvedPositions[0])), Is.True);
            Assert.That(math.distance(simulation.ResolvedPositions[0], simulation.ResolvedPositions[1]),
                Is.GreaterThan(0f));
        }

        [Test]
        public void HeadOnAgentsSteerToOppositeWorldSpaceSides()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(-.3f, 0f), new float2(1f, 0f), .25f);
            SetAgent(simulation, 1, new float2(.3f, 0f), new float2(-1f, 0f), .25f);
            var settings = simulation.Settings;
            settings.LateralSpeedRatio = 1f;
            simulation.Settings = settings;

            simulation.Step(.1f, 2);

            Assert.That(simulation.ResolvedVelocities[0].y * simulation.ResolvedVelocities[1].y,
                Is.LessThan(0f));
        }

        [Test]
        public void HeadOnAgentsSteerBeforeContactWithinPredictionTime()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(-.6f, 0f), new float2(1f, 0f), .25f);
            SetAgent(simulation, 1, new float2(.6f, 0f), new float2(-1f, 0f), .25f);
            var settings = simulation.Settings;
            settings.NeighborDistance = 2f;
            settings.CollisionPredictionTime = .5f;
            settings.LateralSpeedRatio = 1f;
            simulation.Settings = settings;

            simulation.Step(.1f, 2);

            Assert.That(math.abs(simulation.ResolvedVelocities[0].y), Is.GreaterThan(1e-5f));
            Assert.That(math.abs(simulation.ResolvedVelocities[1].y), Is.GreaterThan(1e-5f));
            Assert.That(simulation.Contacts[0].IsTouching, Is.Zero);
            Assert.That(simulation.Contacts[1].IsTouching, Is.Zero);
        }

        [Test]
        public void EqualVelocityFollowersDoNotSteerSideways()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(0f, 0f), new float2(1f, 0f), .2f);
            SetAgent(simulation, 1, new float2(.48f, 0f), new float2(1f, 0f), .2f);
            var settings = simulation.Settings;
            settings.LateralSpeedRatio = 1f;
            simulation.Settings = settings;

            simulation.Step(.1f, 2);

            Assert.That(simulation.ResolvedVelocities[0].y, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(simulation.ResolvedVelocities[1].y, Is.EqualTo(0f).Within(1e-5f));
        }

        [Test]
        public void TinyPreferredSeparationOverlapDoesNotProduceFullSeparationSpeed()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(1f, 0f), .2f);
            SetAgent(simulation, 1, new float2(.479999f, 0f), new float2(1f, 0f), .2f);
            var settings = simulation.Settings;
            settings.PreferredSeparationMultiplier = 1.2f;
            settings.SeparationSpeedRatio = 1f;
            simulation.Settings = settings;

            simulation.Step(.1f, 2);

            Assert.That(math.abs(simulation.ResolvedVelocities[0].x - 1f), Is.LessThan(1e-3f));
            Assert.That(math.abs(simulation.ResolvedVelocities[1].x - 1f), Is.LessThan(1e-3f));
        }

        [Test]
        public void FollowerAdoptsLateralFlowFromAgentAhead()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(0f, 0f), new float2(1f, 0f), .2f);
            SetAgent(simulation, 1, new float2(.8f, .1f), new float2(1f, 0f), .2f);
            var currentVelocities = simulation.CurrentVelocities;
            currentVelocities[0] = new float2(1f, 0f);
            currentVelocities[1] = new float2(1f, .5f);

            simulation.Step(.1f, 2);

            Assert.That(simulation.ResolvedVelocities[0].y, Is.GreaterThan(0f));
        }

        [Test]
        public void CircleObstacleIsImpenetrable()
        {
            using var simulation = Create(1, 1);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            var obstacles = simulation.Obstacles;
            obstacles[0] = Obstacle.Circle(float2.zero, 1f, 2u, 1u);
            simulation.Step(0f, 1, 1);
            Assert.That(math.length(simulation.ResolvedPositions[0]), Is.EqualTo(1.5f).Within(.001f));
            Assert.That(simulation.Contacts[0].ObstacleContactCount, Is.EqualTo(1));
        }

        [Test]
        public void SegmentObstaclePushesAgentToItsSurface()
        {
            using var simulation = Create(1, 1);
            SetAgent(simulation, 0, new float2(0f, .1f), float2.zero, .5f);
            var obstacles = simulation.Obstacles;
            obstacles[0] = Obstacle.Segment(new float2(-2f, 0f), new float2(2f, 0f), 0f, 2u, 1u);
            simulation.Step(0f, 1, 1);
            Assert.That(simulation.ResolvedPositions[0].y, Is.EqualTo(.5f).Within(.001f));
        }

        [TestCase(1f, 1f)]
        [TestCase(-1f, -1f)]
        public void AgentSteersTowardNearerEndOfSegmentObstacle(float y, float expectedSide)
        {
            using var simulation = Create(1, 1);
            SetAgent(simulation, 0, new float2(-1f, y), new float2(1f, 0f), .25f);
            var settings = simulation.Settings;
            settings.CollisionPredictionTime = 1f;
            settings.LateralSpeedRatio = 1f;
            simulation.Settings = settings;
            var obstacles = simulation.Obstacles;
            obstacles[0] = Obstacle.Segment(new float2(0f, -2f), new float2(0f, 2f), 0f, 1u, 1u);

            simulation.Step(.1f, 1, 1);

            Assert.That(simulation.ResolvedVelocities[0].y * expectedSide, Is.GreaterThan(0f));
            Assert.That(simulation.Contacts[0].ObstacleContactCount, Is.Zero);
        }

        [Test]
        public void LowerObstacleRouteAlsoAlignsAgentPassingDownward()
        {
            using var simulation = Create(2, 1);
            SetAgent(simulation, 0, new float2(-1f, -1f), new float2(1f, 0f), .25f);
            SetAgent(simulation, 1, new float2(-.55f, -1f), new float2(-1f, 0f), .2f);
            var settings = simulation.Settings;
            settings.CollisionPredictionTime = 1f;
            settings.LateralSpeedRatio = 1f;
            simulation.Settings = settings;
            var obstacles = simulation.Obstacles;
            obstacles[0] = Obstacle.Segment(new float2(0f, -2f), new float2(0f, 2f), 0f, 1u, 1u);

            simulation.Step(.1f, 2, 1);

            Assert.That(simulation.ResolvedVelocities[0].y, Is.LessThan(0f));
        }

        [Test]
        public void LighterAgentReceivesMoreOverlapCorrection()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(-.25f, 0f), float2.zero, .5f, 1f);
            SetAgent(simulation, 1, new float2(.25f, 0f), float2.zero, .5f, 3f);
            simulation.Step(0f, 2);
            var lightMovement = math.abs(simulation.ResolvedPositions[0].x + .25f);
            var heavyMovement = math.abs(simulation.ResolvedPositions[1].x - .25f);
            Assert.That(lightMovement, Is.GreaterThan(heavyMovement));
        }

        [Test]
        public void EnteredContactIsReportedOnceUntilAgentsSeparate()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, new float2(-.4f, 0f), float2.zero, .5f);
            SetAgent(simulation, 1, new float2(.4f, 0f), float2.zero, .5f);
            var eventMasks = simulation.ContactEventMasks;
            eventMasks[0] = uint.MaxValue;
            eventMasks[1] = uint.MaxValue;

            simulation.Step(0f, 2);
            Assert.That(simulation.EnteredContactCount, Is.EqualTo(1));

            simulation.Step(0f, 2);
            Assert.That(simulation.EnteredContactCount, Is.Zero);

            var positions = simulation.Positions;
            positions[1] = new float2(10f, 0f);
            simulation.Step(0f, 2);
            Assert.That(simulation.EnteredContactCount, Is.Zero);
            Assert.That(simulation.ExitedContactCount, Is.EqualTo(1));

            positions[1] = new float2(.4f, 0f);
            simulation.Step(0f, 2);
            Assert.That(simulation.EnteredContactCount, Is.EqualTo(1));
            Assert.That(simulation.ExitedContactCount, Is.Zero);
        }

        [Test]
        public void LargeAgentIsDetectedBeyondRegularNeighborDistance()
        {
            using var simulation = Create(2);
            simulation.Settings = new LocalAvoidanceSettings
            {
                CellSize = 1.5f,
                NeighborDistance = 1.5f,
                VelocityResponse = 10f,
                SeparationSpeedRatio = .4f,
                LateralSpeedRatio = .15f,
                MinimumSpacingRatio = 1f,
                MaximumCorrectionRatio = .65f,
                ContactSlowdown = 1f,
                ContactsForMaximumSlowdown = 6f,
                ContactSkinRatio = .1f,
                CorrectionVelocityInfluence = .25f,
                SolverIterations = 2,
                InnerLoopBatchCount = 1
            };
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            SetAgent(simulation, 1, new float2(1.9f, 0f), float2.zero, 1.5f);

            simulation.Step(0f, 2);

            Assert.That(simulation.Contacts[0].AgentContactCount, Is.EqualTo(1));
            Assert.That(simulation.Contacts[1].AgentContactCount, Is.EqualTo(1));
        }

        [Test]
        public void DirectControlOverlapCorrectionDoesNotBecomeVelocity()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f, .5f);
            SetAgent(simulation, 1, new float2(.8f, 0f), float2.zero, .5f);
            var directControl = simulation.DirectControl;
            var immediateVelocity = simulation.ImmediateVelocity;
            directControl[0] = 1;
            immediateVelocity[0] = 1;

            simulation.Step(1f / 60f, 2);

            Assert.That(math.length(simulation.ResolvedPositions[0]), Is.GreaterThan(0f));
            Assert.That(math.length(simulation.ResolvedVelocities[0]), Is.EqualTo(0f).Within(1e-5f));
        }

        [Test]
        public void DominantMassRaisesStableAgentCorrectionSpeedLimit()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f, .5f);
            SetAgent(simulation, 1, new float2(.1f, 0f), float2.zero, .5f, 100f);
            var stable = simulation.StableContactResolution;
            var maximumSpeeds = simulation.MaximumCorrectionSpeeds;
            stable[0] = 1;
            maximumSpeeds[0] = 4f;

            simulation.Step(.1f, 2);

            Assert.That(math.distance(simulation.ResolvedPositions[0], simulation.MovedPositions[0]),
                Is.GreaterThan(.4f));
        }

        [Test]
        public void StableAgentPrioritizesDominantMassOverDeeperLightContact()
        {
            using var simulation = Create(3);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f, .5f);
            SetAgent(simulation, 1, new float2(.1f, 0f), float2.zero, .5f, 1f);
            SetAgent(simulation, 2, new float2(-.4f, 0f), float2.zero, .5f, 100f);
            var stable = simulation.StableContactResolution;
            stable[0] = 1;

            simulation.Step(0f, 3);

            Assert.That(simulation.ResolvedPositions[0].x, Is.GreaterThan(0f));
        }

        [Test]
        public void StableAgentDoesNotReenterPreviousPrimaryContact()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(2f, 0f), .5f, .5f);
            SetAgent(simulation, 1, new float2(.8f, 0f), float2.zero, .5f, 100f);
            var stable = simulation.StableContactResolution;
            var immediate = simulation.ImmediateVelocity;
            stable[0] = 1;
            immediate[0] = 1;
            simulation.Step(1f / 60f, 2);

            var positions = simulation.Positions;
            var velocities = simulation.CurrentVelocities;
            positions[0] = simulation.ResolvedPositions[0];
            positions[1] = simulation.ResolvedPositions[1];
            velocities[0] = simulation.ResolvedVelocities[0];
            velocities[1] = simulation.ResolvedVelocities[1];
            var beforeSecondStep = positions[0];

            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.MovedPositions[0].x, Is.LessThanOrEqualTo(beforeSecondStep.x + 1e-5f));
        }

        [Test]
        public void HigherPriorityAgentDoesNotBrakeForRegularCrowd()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(6f, 0f), .5f);
            SetAgent(simulation, 1, new float2(1.1f, 0f), float2.zero, .5f);
            var priorities = simulation.AvoidancePriorities;
            var velocities = simulation.CurrentVelocities;
            priorities[0] = 1;
            velocities[0] = new float2(6f, 0f);

            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.MovedPositions[0].x, Is.EqualTo(.1f).Within(.001f));
            Assert.That(simulation.ResolvedVelocities[0].x, Is.EqualTo(6f).Within(.001f));
        }

        [Test]
        public void MovingNeighborDoesNotCarryIdleStableAgent()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            SetAgent(simulation, 1, new float2(1f, 0f), new float2(-2f, 0f), .5f);
            var priorities = simulation.AvoidancePriorities;
            var stable = simulation.StableContactResolution;
            var currentVelocities = simulation.CurrentVelocities;
            priorities[0] = 0;
            priorities[1] = 1;
            stable[0] = 1;
            currentVelocities[0] = new float2(1f, 0f);
            currentVelocities[1] = new float2(-2f, 0f);

            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.ResolvedPositions[0], Is.EqualTo(float2.zero));
            Assert.That(simulation.ResolvedVelocities[0], Is.EqualTo(float2.zero));
        }

        [Test]
        public void PenetratingNeighborMovesInsteadOfIdleStableAgent()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            SetAgent(simulation, 1, new float2(.8f, 0f), float2.zero, .5f);
            var priorities = simulation.AvoidancePriorities;
            var stable = simulation.StableContactResolution;
            priorities[0] = 0;
            priorities[1] = 1;
            stable[0] = 1;

            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.ResolvedPositions[0], Is.EqualTo(float2.zero));
            Assert.That(simulation.ResolvedPositions[1].x, Is.GreaterThan(.8f));
        }

        [Test]
        public void DirectControlDoesNotCrossBlockingAgentWithinSingleFrame()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(10f, 0f), .5f);
            SetAgent(simulation, 1, new float2(1.5f, 0f), float2.zero, .5f);
            var priorities = simulation.AvoidancePriorities;
            var directControl = simulation.DirectControl;
            var immediate = simulation.ImmediateVelocity;
            priorities[0] = 0;
            priorities[1] = 1;
            directControl[0] = 1;
            immediate[0] = 1;

            simulation.Step(.1f, 2);

            Assert.That(simulation.MovedPositions[0].x, Is.LessThanOrEqualTo(.5001f));
            Assert.That(simulation.ResolvedPositions[0].x, Is.LessThanOrEqualTo(.5001f));
        }

        [Test]
        public void RequiredAgentIsCollectedBeyondRegularCandidateLimit()
        {
            using var simulation = Create(34);
            var settings = simulation.Settings;
            settings.MaximumCandidateChecks = 8;
            simulation.Settings = settings;
            SetAgent(simulation, 0, float2.zero, float2.zero, .5f);
            for (var i = 1; i < 33; i++)
                SetAgent(simulation, i, new float2(.8f, (i - 16) * .001f), float2.zero, .5f);
            SetAgent(simulation, 33, float2.zero, float2.zero, .5f);
            var stable = simulation.StableContactResolution;
            stable[33] = 1;

            simulation.Step(1f / 60f, 34);

            Assert.That(simulation.Contacts[0].ConstraintAgentIndex, Is.EqualTo(33));
        }

        [Test]
        public void ApproachingCrowdAgentOwnsPenetrationAgainstDirectControlledAgent()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(0f, 1f), .5f);
            SetAgent(simulation, 1, new float2(.8f, 0f), new float2(-2f, 0f), .5f);
            var priorities = simulation.AvoidancePriorities;
            var directControl = simulation.DirectControl;
            var stable = simulation.StableContactResolution;
            var immediate = simulation.ImmediateVelocity;
            var currentVelocities = simulation.CurrentVelocities;
            priorities[0] = 0;
            priorities[1] = 1;
            directControl[0] = 1;
            stable[0] = 1;
            immediate[0] = 1;
            currentVelocities[1] = new float2(-2f, 0f);

            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.ResolvedPositions[0].x, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(simulation.ResolvedPositions[1].x, Is.GreaterThan(.8f));
        }

        [Test]
        public void DeepPenetrationStopsAutonomousDesiredMovementOnFollowingStep()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(4f, 0f), .5f);
            SetAgent(simulation, 1, new float2(.5f, 0f), float2.zero, .5f);
            var settings = simulation.Settings;
            settings.ContactSlowdown = 1f;
            simulation.Settings = settings;

            simulation.Step(.0001f, 2);
            var positions = simulation.Positions;
            var currentVelocities = simulation.CurrentVelocities;
            positions[0] = simulation.ResolvedPositions[0];
            positions[1] = simulation.ResolvedPositions[1];
            currentVelocities[0] = float2.zero;
            currentVelocities[1] = float2.zero;
            var beforeMovement = positions[0];
            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.MovedPositions[0], Is.EqualTo(beforeMovement));
        }

        [Test]
        public void DeepSidePenetrationDoesNotStopForwardMovement()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(4f, 0f), .5f);
            SetAgent(simulation, 1, new float2(0f, .5f), float2.zero, .5f);
            var settings = simulation.Settings;
            settings.ContactSlowdown = 1f;
            simulation.Settings = settings;

            simulation.Step(.0001f, 2);
            var positions = simulation.Positions;
            var currentVelocities = simulation.CurrentVelocities;
            positions[0] = simulation.ResolvedPositions[0];
            positions[1] = simulation.ResolvedPositions[1];
            currentVelocities[0] = float2.zero;
            currentVelocities[1] = float2.zero;
            var beforeMovement = positions[0];
            simulation.Step(1f / 60f, 2);

            Assert.That(simulation.MovedPositions[0].x, Is.GreaterThan(beforeMovement.x));
        }

        [Test]
        public void AutonomousContactVelocityDoesNotExceedDesiredSpeed()
        {
            using var simulation = Create(2);
            SetAgent(simulation, 0, float2.zero, new float2(1f, 0f), .5f);
            SetAgent(simulation, 1, new float2(.5f, 0f), new float2(-1f, 0f), .5f);
            var settings = simulation.Settings;
            settings.CorrectionVelocityInfluence = 1f;
            simulation.Settings = settings;

            simulation.Step(1f / 60f, 2);

            Assert.That(math.length(simulation.ResolvedVelocities[0]),
                Is.LessThanOrEqualTo(1.0001f));
        }

        private static LocalAvoidanceSimulation Create(int agents, int obstacles = 0)
        {
            var simulation = new LocalAvoidanceSimulation(agents, obstacles, Allocator.Persistent);
            simulation.Settings = LocalAvoidanceSettings.Default;
            return simulation;
        }

        private static void SetAgent(LocalAvoidanceSimulation simulation, int index, float2 position,
            float2 desiredVelocity, float radius, float mass = 1f, float avoidanceWeight = 1f)
        {
            var positions = simulation.Positions;
            var desiredVelocities = simulation.DesiredVelocities;
            var currentVelocities = simulation.CurrentVelocities;
            var radii = simulation.Radii;
            var masses = simulation.Masses;
            var avoidanceWeights = simulation.AvoidanceWeights;
            var layers = simulation.Layers;
            var collisionMasks = simulation.CollisionMasks;
            var active = simulation.Active;
            positions[index] = position;
            desiredVelocities[index] = desiredVelocity;
            currentVelocities[index] = float2.zero;
            radii[index] = radius;
            masses[index] = mass;
            avoidanceWeights[index] = avoidanceWeight;
            layers[index] = 1u;
            collisionMasks[index] = uint.MaxValue;
            active[index] = 1;
        }
    }
}
