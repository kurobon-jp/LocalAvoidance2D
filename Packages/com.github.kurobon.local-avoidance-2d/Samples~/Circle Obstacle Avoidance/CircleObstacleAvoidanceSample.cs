using System;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace LocalAvoidance2D.Samples
{
    /// <summary>Isolates a compact agent grid predictively avoiding one circular obstacle.</summary>
    public sealed class CircleObstacleAvoidanceSample : MonoBehaviour
    {
        [SerializeField, Range(1, 128)] private int agentCount = 25;
        [SerializeField, Min(.05f)] private float agentRadius = .22f;
        [SerializeField, Min(2f)] private float spacingMultiplier = 2.5f;
        [SerializeField, Min(.1f)] private float speed = 2.5f;
        [SerializeField] private Vector2 start = new(-5f, 0f);
        [SerializeField] private Vector2 goal = new(5f, 0f);
        [SerializeField] private Vector2 obstacleCenter = Vector2.zero;
        [SerializeField, Min(.05f)] private float obstacleRadius = 1f;
        [SerializeField, Min(.1f)] private float neighborDistance = 3f;
        [SerializeField, Min(.01f)] private float collisionPredictionTime = .75f;
        [SerializeField, Min(0f)] private float avoidanceWeight = 1f;
        [SerializeField, Min(.001f)] private float stalledSpeed = .05f;
        [SerializeField] private bool enableDiagnosticLog = true;
        [SerializeField, Range(1, 60)] private int diagnosticFrameInterval = 3;

        private LocalAvoidanceSimulation _simulation;
        private Transform[] _agentViews;
        private float2[] _spawnPositions;
        private float2[] _goals;
        private Material _agentMaterial;
        private Material _obstacleMaterial;
        private StreamWriter _diagnosticWriter;
        private float _nextDiagnosticFlushTime;
        private float[] _stalledTimes;
        private float[] _minimumClearances;
        private sbyte[] _previousNonZeroSides;
        private int[] _sideSwitches;
        private bool[] _teleportedThisFrame;
        private int[] _completedLaps;

        private void Start()
        {
            agentCount = Mathf.Max(1, agentCount);
            _simulation = new LocalAvoidanceSimulation(agentCount, 1, Allocator.Persistent);
            var settings = _simulation.Settings;
            settings.NeighborDistance = neighborDistance;
            settings.CollisionPredictionTime = collisionPredictionTime;
            _simulation.Settings = settings;

            var avoidanceWeights = _simulation.AvoidanceWeights;
            _agentViews = new Transform[agentCount];
            _spawnPositions = new float2[agentCount];
            _goals = new float2[agentCount];
            _stalledTimes = new float[agentCount];
            _minimumClearances = new float[agentCount];
            _previousNonZeroSides = new sbyte[agentCount];
            _sideSwitches = new int[agentCount];
            _teleportedThisFrame = new bool[agentCount];
            _completedLaps = new int[agentCount];
            for (var i = 0; i < agentCount; i++) _minimumClearances[i] = float.PositiveInfinity;
            var obstacles = _simulation.Obstacles;
            obstacles[0] = Obstacle.Circle(
                new float2(obstacleCenter.x, obstacleCenter.y), obstacleRadius, 1u, 1u);

            _agentMaterial = CreateMaterial(new Color(.15f, .75f, 1f));
            _obstacleMaterial = CreateMaterial(new Color(.15f, .17f, .2f));
            var columns = Mathf.Max(1, Mathf.CeilToInt(Mathf.Sqrt(agentCount)));
            var rows = Mathf.CeilToInt(agentCount / (float)columns);
            var spacing = agentRadius * spacingMultiplier;
            for (var i = 0; i < agentCount; i++)
            {
                var row = i / columns;
                var column = i % columns;
                var yOffset = (row - (rows - 1) * .5f) * spacing;
                var xOffset = column * spacing;
                var position = new float2(start.x - xOffset, start.y + yOffset);
                // Apply the same column offset at both ends so every lane has the same
                // travel distance and the grid does not stretch while agents loop.
                var goalPosition = new float2(goal.x - xOffset, goal.y + yOffset);
                _spawnPositions[i] = position;
                _goals[i] = goalPosition;
                var desiredVelocity = math.normalizesafe(goalPosition - position) * speed;
                _simulation.ActivateAgent(i, position, desiredVelocity, agentRadius);
                avoidanceWeights[i] = avoidanceWeight;
                _agentViews[i] = CreateDisc($"Agent {i}", position, agentRadius, _agentMaterial);
            }
            CreateDisc("Circle Obstacle", new float2(obstacleCenter.x, obstacleCenter.y),
                obstacleRadius, _obstacleMaterial);
            if (enableDiagnosticLog) OpenDiagnosticLog();
        }

        private void Update()
        {
            var inputPositions = _simulation.Positions;
            var desiredVelocities = _simulation.DesiredVelocities;
            for (var i = 0; i < agentCount; i++)
            {
                _teleportedThisFrame[i] = false;
                desiredVelocities[i] = math.normalizesafe(_goals[i] - inputPositions[i]) * speed;
            }
            _simulation.Step(Time.deltaTime, agentCount, 1);
            var resolvedPositions = _simulation.ResolvedPositions;
            var resolvedVelocities = _simulation.ResolvedVelocities;
            var positions = _simulation.Positions;
            var currentVelocities = _simulation.CurrentVelocities;
            for (var i = 0; i < agentCount; i++)
            {
                var position = resolvedPositions[i];
                var velocity = resolvedVelocities[i];
                var clearance = CircleClearance(position);
                _minimumClearances[i] = math.min(_minimumClearances[i], clearance);
                var currentSpeed = math.length(velocity);
                _stalledTimes[i] = currentSpeed < stalledSpeed
                    ? _stalledTimes[i] + Time.deltaTime
                    : 0f;
                var side = _simulation.ObstacleAvoidanceSides[i];
                if (side != 0)
                {
                    if (_previousNonZeroSides[i] != 0 && side != _previousNonZeroSides[i])
                        _sideSwitches[i]++;
                    _previousNonZeroSides[i] = side;
                }

                if (position.x >= _goals[i].x)
                {
                    _teleportedThisFrame[i] = true;
                    _completedLaps[i]++;
                    position = _spawnPositions[i];
                    _simulation.Teleport(i, position);
                }
                else
                {
                    positions[i] = position;
                    currentVelocities[i] = velocity;
                }
                _agentViews[i].position = new Vector3(position.x, position.y, 0f);
            }

            WriteDiagnosticFrame();
            for (var i = 0; i < agentCount; i++)
            {
                if (!_teleportedThisFrame[i]) continue;
                _stalledTimes[i] = 0f;
                _minimumClearances[i] = float.PositiveInfinity;
                _previousNonZeroSides[i] = 0;
                _sideSwitches[i] = 0;
            }
        }

        private void OpenDiagnosticLog()
        {
            var fileName = $"local-avoidance-circle-{DateTime.Now:yyyyMMdd-HHmmss}.csv";
            var path = Path.Combine(Application.persistentDataPath, fileName);
            _diagnosticWriter = new StreamWriter(path, false);
            _diagnosticWriter.WriteLine(
                "frame,time,index,pos_x,pos_y,goal_x,goal_y,desired_x,desired_y,resolved_vx,resolved_vy,speed," +
                "circle_clearance,minimum_clearance,retained_obstacle_side,side_retention_time," +
                "side_switches,obstacle_contacts,touching,constraint,stalled_time,teleported,completed_laps," +
                "agent_radius,obstacle_radius,collision_prediction_time,avoidance_weight");
            Debug.Log($"[LocalAvoidance.Circle] Diagnostic log: {path}");
        }

        private void WriteDiagnosticFrame()
        {
            if (_diagnosticWriter == null || Time.frameCount % diagnosticFrameInterval != 0) return;
            var positions = _simulation.ResolvedPositions;
            var desired = _simulation.DesiredVelocities;
            var velocities = _simulation.ResolvedVelocities;
            var contacts = _simulation.Contacts;
            var retainedSides = _simulation.ObstacleAvoidanceSides;
            var retentionTimes = _simulation.ObstacleAvoidanceRetentionTimes;
            var invariant = CultureInfo.InvariantCulture;
            for (var i = 0; i < agentCount; i++)
            {
                var position = positions[i];
                var velocity = velocities[i];
                var contact = contacts[i];
                _diagnosticWriter.Write(Time.frameCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(Time.time.ToString("F4", invariant));
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(i);
                WriteFloat(position.x, invariant);
                WriteFloat(position.y, invariant);
                WriteFloat(_goals[i].x, invariant);
                WriteFloat(_goals[i].y, invariant);
                WriteFloat(desired[i].x, invariant);
                WriteFloat(desired[i].y, invariant);
                WriteFloat(velocity.x, invariant);
                WriteFloat(velocity.y, invariant);
                WriteFloat(math.length(velocity), invariant);
                WriteFloat(CircleClearance(position), invariant);
                WriteFloat(_minimumClearances[i], invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(retainedSides[i]);
                WriteFloat(retentionTimes[i], invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(_sideSwitches[i]);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.ObstacleContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.IsTouching);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.HasConstraint);
                WriteFloat(_stalledTimes[i], invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(_teleportedThisFrame[i] ? 1 : 0);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(_completedLaps[i]);
                WriteFloat(agentRadius, invariant);
                WriteFloat(obstacleRadius, invariant);
                WriteFloat(collisionPredictionTime, invariant);
                WriteFloat(avoidanceWeight, invariant);
                _diagnosticWriter.WriteLine();
            }

            if (Time.unscaledTime < _nextDiagnosticFlushTime) return;
            _diagnosticWriter.Flush();
            _nextDiagnosticFlushTime = Time.unscaledTime + 1f;
        }

        private float CircleClearance(float2 position) =>
            math.distance(position, new float2(obstacleCenter.x, obstacleCenter.y)) -
            obstacleRadius - agentRadius;

        private void WriteFloat(float value, IFormatProvider provider)
        {
            _diagnosticWriter.Write(',');
            _diagnosticWriter.Write(value.ToString("F5", provider));
        }

        private Transform CreateDisc(string objectName, float2 position, float discRadius,
            Material material)
        {
            var disc = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            disc.name = objectName;
            disc.transform.SetParent(transform, false);
            disc.transform.position = new Vector3(position.x, position.y, 0f);
            disc.transform.localScale = new Vector3(discRadius * 2f, discRadius * 2f, .08f);
            disc.GetComponent<MeshRenderer>().sharedMaterial = material;
            Destroy(disc.GetComponent<Collider>());
            return disc.transform;
        }

        private static Material CreateMaterial(Color color)
        {
            var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
            var material = new Material(shader);
            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
            if (material.HasProperty("_Color")) material.SetColor("_Color", color);
            return material;
        }

        private void OnDestroy()
        {
            _diagnosticWriter?.Dispose();
            _simulation?.Dispose();
            if (_agentMaterial != null) Destroy(_agentMaterial);
            if (_obstacleMaterial != null) Destroy(_obstacleMaterial);
        }
    }
}
