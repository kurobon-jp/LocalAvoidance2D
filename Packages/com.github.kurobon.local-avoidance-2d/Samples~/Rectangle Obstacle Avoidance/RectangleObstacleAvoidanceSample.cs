using System;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace LocalAvoidance2D.Samples
{
    /// <summary>Predictive local avoidance around a rectangle made from segment obstacles.</summary>
    public sealed class RectangleObstacleAvoidanceSample : MonoBehaviour
    {
        [SerializeField, Range(1, 128)] private int agentCount = 48;
        [SerializeField, Min(.05f)] private float radius = .22f;
        [SerializeField, Min(.1f)] private float speed = 2.5f;
        [SerializeField] private float spawnX = -8f;
        [SerializeField] private float goalX = 9f;
        [SerializeField] private Vector2 rectangleCenter = Vector2.zero;
        [SerializeField] private Vector2 rectangleSize = new(2f, 4f);
        [SerializeField, Min(.1f)] private float neighborDistance = 3f;
        [SerializeField, Min(.01f)] private float collisionPredictionTime = .75f;
        [SerializeField, Min(0f)] private float avoidanceWeight = 1f;
        [SerializeField, Min(0f)] private float correctionVelocityWeight = 1f;
        [SerializeField] private bool enableDiagnosticLog = true;
        [SerializeField, Min(.5f)] private float diagnosticRange = 5f;
        [SerializeField, Range(1, 60)] private int diagnosticFrameInterval = 6;

        private LocalAvoidanceSimulation _simulation;
        private LocalAvoidanceDiagnostics _diagnostics;
        private Transform[] _views;
        private float2[] _spawnPositions;
        private Material _agentMaterial;
        private Material _obstacleMaterial;
        private StreamWriter _diagnosticWriter;
        private float _nextDiagnosticFlushTime;

        private void Start()
        {
            agentCount = Mathf.Max(1, agentCount);
            _simulation = new LocalAvoidanceSimulation(agentCount, 4, Allocator.Persistent);
            var settings = _simulation.Settings;
            settings.NeighborDistance = neighborDistance;
            settings.CollisionPredictionTime = collisionPredictionTime;
            settings.LateralSpeedRatio = .8f;
            settings.LateralFlowFollowing = .45f;
            _simulation.Settings = settings;
            if (enableDiagnosticLog)
            {
                _diagnostics = new LocalAvoidanceDiagnostics(agentCount);
                OpenDiagnosticLog();
            }

            _views = new Transform[agentCount];
            _spawnPositions = new float2[agentCount];
            _agentMaterial = CreateMaterial(new Color(.15f, .75f, 1f));
            _obstacleMaterial = CreateMaterial(new Color(.15f, .17f, .2f));

            var columns = Mathf.Max(1, Mathf.CeilToInt(Mathf.Sqrt(agentCount)));
            var rows = Mathf.CeilToInt(agentCount / (float)columns);
            var spacing = radius * 2.5f;
            var avoidanceWeights = _simulation.AvoidanceWeights;
            var correctionWeights = _simulation.CorrectionVelocityWeights;
            for (var i = 0; i < agentCount; i++)
            {
                var row = i / columns;
                var column = i % columns;
                var y = (row - (rows - 1) * .5f) * spacing;
                var position = new float2(spawnX - column * spacing, y);
                _spawnPositions[i] = position;
                _simulation.ActivateAgent(i, position, new float2(speed, 0f), radius);
                avoidanceWeights[i] = avoidanceWeight;
                correctionWeights[i] = correctionVelocityWeight;
                _views[i] = CreateDisc($"Agent {i}", position, radius, _agentMaterial);
            }

            ConfigureRectangleObstacle();
            CreateRectangle();
        }

        private void Update()
        {
            var positions = _simulation.Positions;
            _simulation.Step(Time.deltaTime, agentCount, 4, _diagnostics);
            var resolvedPositions = _simulation.ResolvedPositions;
            var resolvedVelocities = _simulation.ResolvedVelocities;
            var currentVelocities = _simulation.CurrentVelocities;
            for (var i = 0; i < agentCount; i++)
            {
                var position = resolvedPositions[i];
                if (position.x >= goalX - .05f)
                {
                    position = _spawnPositions[i];
                    _simulation.Teleport(i, position);
                }
                else
                {
                    currentVelocities[i] = resolvedVelocities[i];
                    positions[i] = position;
                }
                _views[i].position = new Vector3(position.x, position.y, 0f);
            }

            WriteDiagnosticFrame();
        }

        private void OpenDiagnosticLog()
        {
            var fileName = $"local-avoidance-rectangle-{DateTime.Now:yyyyMMdd-HHmmss}.csv";
            var path = Path.Combine(Application.persistentDataPath, fileName);
            _diagnosticWriter = new StreamWriter(path, false);
            _diagnosticWriter.WriteLine(
                "frame,time,index,pos_x,pos_y,desired_x,desired_y,resolved_vx,resolved_vy,avoidance_side," +
                "retained_obstacle_side,side_retention_time,rectangle_clearance,contacts,obstacle_contacts,touching,constraint," +
                "first_correction_x,first_correction_y,last_correction_x,last_correction_y," +
                "neighbor_distance,collision_prediction_time,avoidance_weight,correction_velocity_weight");
            Debug.Log($"[LocalAvoidance.Rectangle] Diagnostic log: {path}");
        }

        private void WriteDiagnosticFrame()
        {
            if (_diagnosticWriter == null || Time.frameCount % diagnosticFrameInterval != 0) return;

            var positions = _simulation.ResolvedPositions;
            var desired = _simulation.DesiredVelocities;
            var velocities = _simulation.ResolvedVelocities;
            var contacts = _simulation.Contacts;
            var lastIteration = math.min(_simulation.Settings.SolverIterations,
                LocalAvoidanceDiagnostics.MaximumSolverIterations) - 1;
            var retainedSides = _simulation.ObstacleAvoidanceSides;
            var retentionTimes = _simulation.ObstacleAvoidanceRetentionTimes;
            var invariant = CultureInfo.InvariantCulture;
            for (var i = 0; i < agentCount; i++)
            {
                var position = positions[i];
                if (Mathf.Abs(position.x - rectangleCenter.x) > diagnosticRange) continue;
                var velocity = velocities[i];
                var contact = contacts[i];
                var firstCorrection = _diagnostics.GetSolverCorrection(i, 0);
                var lastCorrection = _diagnostics.GetSolverCorrection(i, lastIteration);
                _diagnosticWriter.Write(Time.frameCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(Time.time.ToString("F4", invariant));
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(i);
                WriteFloat(position.x, invariant);
                WriteFloat(position.y, invariant);
                WriteFloat(desired[i].x, invariant);
                WriteFloat(desired[i].y, invariant);
                WriteFloat(velocity.x, invariant);
                WriteFloat(velocity.y, invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(velocity.y > .01f ? "upper" : velocity.y < -.01f ? "lower" : "none");
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(retainedSides[i] > 0 ? "upper" : retainedSides[i] < 0 ? "lower" : "none");
                WriteFloat(retentionTimes[i], invariant);
                WriteFloat(RectangleClearance(position), invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.AgentContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.ObstacleContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.IsTouching);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.HasConstraint);
                WriteFloat(firstCorrection.x, invariant);
                WriteFloat(firstCorrection.y, invariant);
                WriteFloat(lastCorrection.x, invariant);
                WriteFloat(lastCorrection.y, invariant);
                WriteFloat(neighborDistance, invariant);
                WriteFloat(collisionPredictionTime, invariant);
                WriteFloat(avoidanceWeight, invariant);
                WriteFloat(correctionVelocityWeight, invariant);
                _diagnosticWriter.WriteLine();
            }

            if (Time.unscaledTime < _nextDiagnosticFlushTime) return;
            _diagnosticWriter.Flush();
            _nextDiagnosticFlushTime = Time.unscaledTime + 1f;
        }

        private float RectangleClearance(float2 position)
        {
            var center = new float2(rectangleCenter.x, rectangleCenter.y);
            var half = new float2(Mathf.Abs(rectangleSize.x), Mathf.Abs(rectangleSize.y)) * .5f;
            var clamped = math.clamp(position, center - half, center + half);
            var outsideDistance = math.distance(position, clamped);
            if (outsideDistance > 1e-5f) return outsideDistance - radius;
            var local = position - center;
            var insideDepth = math.min(half.x - math.abs(local.x), half.y - math.abs(local.y));
            return -insideDepth - radius;
        }

        private void WriteFloat(float value, IFormatProvider provider)
        {
            _diagnosticWriter.Write(',');
            _diagnosticWriter.Write(value.ToString("F5", provider));
        }

        private void ConfigureRectangleObstacle()
        {
            var center = new float2(rectangleCenter.x, rectangleCenter.y);
            var half = new float2(Mathf.Abs(rectangleSize.x), Mathf.Abs(rectangleSize.y)) * .5f;
            var bottomLeft = center - half;
            var bottomRight = center + new float2(half.x, -half.y);
            var topRight = center + half;
            var topLeft = center + new float2(-half.x, half.y);
            var obstacles = _simulation.Obstacles;
            obstacles[0] = Obstacle.Segment(bottomLeft, bottomRight, 0f, 1u, 1u);
            obstacles[1] = Obstacle.Segment(bottomRight, topRight, 0f, 1u, 1u);
            obstacles[2] = Obstacle.Segment(topRight, topLeft, 0f, 1u, 1u);
            obstacles[3] = Obstacle.Segment(topLeft, bottomLeft, 0f, 1u, 1u);
        }

        private Transform CreateDisc(string objectName, float2 position, float discRadius, Material material)
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

        private void CreateRectangle()
        {
            var rectangle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rectangle.name = "Rectangle Obstacle";
            rectangle.transform.SetParent(transform, false);
            rectangle.transform.position = new Vector3(rectangleCenter.x, rectangleCenter.y, 0f);
            rectangle.transform.localScale = new Vector3(
                Mathf.Abs(rectangleSize.x), Mathf.Abs(rectangleSize.y), .08f);
            rectangle.GetComponent<MeshRenderer>().sharedMaterial = _obstacleMaterial;
            Destroy(rectangle.GetComponent<Collider>());
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
            if (_agentMaterial != null) Destroy(_agentMaterial);
            if (_obstacleMaterial != null) Destroy(_obstacleMaterial);
        }
    }
}
