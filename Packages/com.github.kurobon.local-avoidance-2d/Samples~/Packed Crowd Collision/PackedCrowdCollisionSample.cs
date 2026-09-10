using System;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace LocalAvoidance2D.Samples
{
    /// <summary>A moving crowd collides with an already packed destination crowd.</summary>
    public sealed class PackedCrowdCollisionSample : MonoBehaviour
    {
        [SerializeField, Range(0, 91)] private int packedAgentCount = 37;
        [SerializeField, Range(1, 64)] private int movingAgentCount = 24;
        [SerializeField, Min(.05f)] private float radius = .22f;
        [SerializeField, Min(.1f)] private float speed = 2.5f;
        [SerializeField] private float movingSpawnX = -8f;
        [SerializeField] private float movingGoalX = 8f;
        [SerializeField] private bool movingGroupJoinsPackedDestination;
        [SerializeField, Min(.1f)] private float slowingDistance = 2f;
        [SerializeField, Min(0f)] private float packingSpeedRatio = .6f;
        [SerializeField, Min(.1f)] private float neighborDistance = 3f;
        [SerializeField, Min(.1f)] private float obstacleRadius = 1f;
        [SerializeField, Min(.1f)] private float obstacleY = 3f;
        [SerializeField] private bool enableDiagnosticLog = true;
        [SerializeField, Range(1, 60)] private int diagnosticFrameInterval = 6;

        private LocalAvoidanceSimulation _simulation;
        private LocalAvoidanceDiagnostics _diagnostics;
        private Transform[] _views;
        private Material _packedMaterial;
        private Material _movingMaterial;
        private Material _obstacleMaterial;
        private StreamWriter _diagnosticWriter;
        private float _nextDiagnosticFlushTime;

        private void Start()
        {
            movingAgentCount = Mathf.Max(1, movingAgentCount);
            var agentCount = packedAgentCount + movingAgentCount;
            _simulation = new LocalAvoidanceSimulation(agentCount, 2, Allocator.Persistent);
            var settings = _simulation.Settings;
            settings.NeighborDistance = neighborDistance;
            _simulation.Settings = settings;
            if (enableDiagnosticLog)
            {
                _diagnostics = new LocalAvoidanceDiagnostics(agentCount);
                OpenDiagnosticLog();
            }

            _views = new Transform[agentCount];
            _packedMaterial = CreateMaterial(new Color(.15f, .75f, 1f));
            _movingMaterial = CreateMaterial(new Color(1f, .3f, .2f));
            _obstacleMaterial = CreateMaterial(new Color(.15f, .17f, .2f));
            CreateObstacles();
            CreatePackedCrowd();
            CreateMovingCrowd();
        }

        private void CreatePackedCrowd()
        {
            var spacing = radius * 2f * .98f;
            for (var i = 0; i < packedAgentCount; i++)
            {
                float2 position;
                if (i == 0) position = float2.zero;
                else
                {
                    var remaining = i - 1;
                    var ring = 1;
                    while (remaining >= ring * 6)
                    {
                        remaining -= ring * 6;
                        ring++;
                    }
                    var angle = remaining * (math.PI * 2f / (ring * 6f));
                    position = new float2(math.cos(angle), math.sin(angle)) * (ring * spacing);
                }

                _simulation.ActivateAgent(i, position, float2.zero, radius);
                _simulation.SetDestination(i, float2.zero, speed,
                    slowingDistance, packingSpeedRatio);
                _views[i] = CreateDisc($"Packed Agent {i}", position, _packedMaterial);
            }
        }

        private void CreateMovingCrowd()
        {
            var columns = Mathf.Max(1, Mathf.CeilToInt(Mathf.Sqrt(movingAgentCount)));
            var rows = Mathf.CeilToInt(movingAgentCount / (float)columns);
            var spacing = radius * 2.5f;
            var finalDestination = movingGroupJoinsPackedDestination
                ? float2.zero
                : new float2(movingGoalX, 0f);
            for (var groupIndex = 0; groupIndex < movingAgentCount; groupIndex++)
            {
                var index = packedAgentCount + groupIndex;
                var row = groupIndex / columns;
                var column = groupIndex % columns;
                var position = new float2(
                    movingSpawnX - column * spacing,
                    (row - (rows - 1) * .5f) * spacing);
                _simulation.ActivateAgent(index, position, float2.zero, radius);
                _simulation.SetDestination(index, finalDestination, speed,
                    slowingDistance, packingSpeedRatio);
                _views[index] = CreateDisc($"Moving Agent {groupIndex}", position, _movingMaterial);
            }
        }

        private void Update()
        {
            var count = packedAgentCount + movingAgentCount;
            _simulation.Step(Time.deltaTime, count, 2, _diagnostics);
            var positions = _simulation.Positions;
            var currentVelocities = _simulation.CurrentVelocities;
            var resolvedPositions = _simulation.ResolvedPositions;
            var resolvedVelocities = _simulation.ResolvedVelocities;
            for (var i = 0; i < count; i++)
            {
                positions[i] = resolvedPositions[i];
                currentVelocities[i] = resolvedVelocities[i];
                _views[i].position = new Vector3(resolvedPositions[i].x, resolvedPositions[i].y, 0f);
            }

            WriteDiagnosticFrame();
        }

        private void OpenDiagnosticLog()
        {
            var fileName = $"local-avoidance-packed-collision-{DateTime.Now:yyyyMMdd-HHmmss}.csv";
            var path = Path.Combine(Application.persistentDataPath, fileName);
            _diagnosticWriter = new StreamWriter(path, false);
            _diagnosticWriter.WriteLine(
                "frame,time,index,group,pos_x,pos_y,desired_x,desired_y,resolved_vx,resolved_vy," +
                "speed,contacts,blocking_contacts,touching,constraint,sleeping," +
                "retained_avoidance_side,side_retention_time," +
                "first_correction_x,first_correction_y,last_correction_x,last_correction_y," +
                "joins_packed_destination,neighbor_distance");
            Debug.Log($"[LocalAvoidance.PackedCollision] Diagnostic log: {path}");
        }

        private void CreateObstacles()
        {
            var obstacles = _simulation.Obstacles;
            obstacles[0] = Obstacle.Circle(new float2(0f, obstacleY), obstacleRadius, 1u, 1u);
            obstacles[1] = Obstacle.Circle(new float2(0f, -obstacleY), obstacleRadius, 1u, 1u);
            CreateObstacleView("Upper Obstacle", new float2(0f, obstacleY));
            CreateObstacleView("Lower Obstacle", new float2(0f, -obstacleY));
        }

        private void CreateObstacleView(string objectName, float2 position)
        {
            var obstacle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obstacle.name = objectName;
            obstacle.transform.SetParent(transform, false);
            obstacle.transform.position = new Vector3(position.x, position.y, 0f);
            obstacle.transform.localScale = new Vector3(
                obstacleRadius * 2f, obstacleRadius * 2f, .08f);
            obstacle.GetComponent<MeshRenderer>().sharedMaterial = _obstacleMaterial;
            Destroy(obstacle.GetComponent<Collider>());
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
            var lastIteration = math.min(_simulation.Settings.SolverIterations,
                LocalAvoidanceDiagnostics.MaximumSolverIterations) - 1;
            var invariant = CultureInfo.InvariantCulture;
            var count = packedAgentCount + movingAgentCount;
            for (var i = 0; i < count; i++)
            {
                var position = positions[i];
                var velocity = velocities[i];
                var contact = contacts[i];
                var firstCorrection = _diagnostics.GetSolverCorrection(i, 0);
                var lastCorrection = _diagnostics.GetSolverCorrection(i, lastIteration);
                _diagnosticWriter.Write(Time.frameCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(Time.time.ToString("F4", invariant));
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(i);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(i < packedAgentCount ? "packed" : "moving");
                WriteFloat(position.x, invariant);
                WriteFloat(position.y, invariant);
                WriteFloat(desired[i].x, invariant);
                WriteFloat(desired[i].y, invariant);
                WriteFloat(velocity.x, invariant);
                WriteFloat(velocity.y, invariant);
                WriteFloat(math.length(velocity), invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.AgentContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.BlockingAgentContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.IsTouching);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.HasConstraint);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(_simulation.IsDestinationSleeping(i) ? 1 : 0);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(retainedSides[i]);
                WriteFloat(retentionTimes[i], invariant);
                WriteFloat(firstCorrection.x, invariant);
                WriteFloat(firstCorrection.y, invariant);
                WriteFloat(lastCorrection.x, invariant);
                WriteFloat(lastCorrection.y, invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(movingGroupJoinsPackedDestination ? 1 : 0);
                WriteFloat(neighborDistance, invariant);
                _diagnosticWriter.WriteLine();
            }

            if (Time.unscaledTime < _nextDiagnosticFlushTime) return;
            _diagnosticWriter.Flush();
            _nextDiagnosticFlushTime = Time.unscaledTime + 1f;
        }

        private void WriteFloat(float value, IFormatProvider provider)
        {
            _diagnosticWriter.Write(',');
            _diagnosticWriter.Write(value.ToString("F5", provider));
        }

        private Transform CreateDisc(string objectName, float2 position, Material material)
        {
            var disc = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            disc.name = objectName;
            disc.transform.SetParent(transform, false);
            disc.transform.position = new Vector3(position.x, position.y, 0f);
            disc.transform.localScale = new Vector3(radius * 2f, radius * 2f, .08f);
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
            _diagnosticWriter = null;
            _diagnostics?.Dispose();
            _diagnostics = null;
            _simulation?.Dispose();
            if (_packedMaterial != null) Destroy(_packedMaterial);
            if (_movingMaterial != null) Destroy(_movingMaterial);
            if (_obstacleMaterial != null) Destroy(_obstacleMaterial);
        }
    }
}
