using System;
using System.Globalization;
using System.IO;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace LocalAvoidance2D.Samples
{
    /// <summary>Minimal gather, simulate and apply example for LocalAvoidanceSimulation.</summary>
    public sealed class BasicCrowdAvoidanceSample : MonoBehaviour
    {
        [SerializeField, Range(2, 256)] private int agentCount = 96;
        [SerializeField, Min(.05f)] private float radius = .22f;
        [SerializeField, Min(.1f)] private float speed = 2.5f;
        [SerializeField] private float spawnX = 8f;
        [SerializeField] private float goalX = 9f;
        [SerializeField, Min(.1f)] private float neighborDistance = 5f;
        [SerializeField, Min(.01f)] private float collisionPredictionTime = 1f;
        [SerializeField, Min(0f)] private float lateralSpeedRatio = 1f;
        [SerializeField, Range(0f, 1f)] private float lateralFlowFollowing = .3f;
        [SerializeField, Min(0f)] private float avoidanceWeight = 1f;
        [SerializeField] private bool enableDiagnosticLog = true;
        [SerializeField, Min(.5f)] private float diagnosticRange = 3.5f;
        [SerializeField, Min(0f)] private float diagnosticOpeningDuration = 4f;
        [SerializeField, Range(1, 60)] private int diagnosticFrameInterval = 6;

        private LocalAvoidanceSimulation _simulation;
        private LocalAvoidanceDiagnostics _diagnostics;
        private Transform[] _views;
        private Material _leftMaterial;
        private Material _rightMaterial;
        private StreamWriter _diagnosticWriter;
        private float _nextDiagnosticFlushTime;
        private float _diagnosticOpeningEndTime;

        private void Start()
        {
            agentCount = Mathf.Max(2, agentCount);
            _simulation = new LocalAvoidanceSimulation(agentCount, 0, Allocator.Persistent);
            var settings = _simulation.Settings;
            settings.NeighborDistance = neighborDistance;
            settings.CollisionPredictionTime = collisionPredictionTime;
            settings.LateralSpeedRatio = lateralSpeedRatio;
            settings.LateralFlowFollowing = lateralFlowFollowing;
            settings.SeparationSpeedRatio = .55f;
            settings.ContactSlowdown = .35f;
            _simulation.Settings = settings;
            if (enableDiagnosticLog)
            {
                _diagnostics = new LocalAvoidanceDiagnostics(agentCount);
                OpenDiagnosticLog();
            }
            _views = new Transform[agentCount];
            _leftMaterial = CreateMaterial(new Color(.15f, .75f, 1f));
            _rightMaterial = CreateMaterial(new Color(1f, .3f, .2f));

            var largerGroupCount = (agentCount + 1) / 2;
            var columns = Mathf.Max(1, Mathf.CeilToInt(Mathf.Sqrt(largerGroupCount)));
            var rows = Mathf.CeilToInt(largerGroupCount / (float)columns);
            // Keep the initial formation clear of the preferred-separation boundary. Placing
            // agents exactly on that boundary makes the setup sensitive to float rounding.
            var spacing = radius * 2.5f;
            var avoidanceWeights = _simulation.AvoidanceWeights;
            for (var i = 0; i < agentCount; i++)
            {
                var fromLeft = (i & 1) == 0;
                var groupIndex = i / 2;
                var row = groupIndex / columns;
                var column = groupIndex % columns;
                var groupCount = fromLeft ? (agentCount + 1) / 2 : agentCount / 2;
                var itemsInRow = Mathf.Min(columns, groupCount - row * columns);
                var centeredColumn = column + (columns - itemsInRow) * .5f;
                var position = new float2(
                    (fromLeft ? -spawnX : spawnX) + centeredColumn * spacing * (fromLeft ? -1f : 1f),
                    (row - (rows - 1) * .5f) * spacing);
                var direction = fromLeft ? 1f : -1f;
                _simulation.ActivateAgent(i, position, new float2(direction * speed, 0f), radius);
                avoidanceWeights[i] = avoidanceWeight;
                _views[i] = CreateDisc($"Agent {i}", position, radius,
                    fromLeft ? _leftMaterial : _rightMaterial);
            }

        }

        private void Update()
        {
            _simulation.Step(Time.deltaTime, agentCount, 0, _diagnostics);
            var resolvedPositions = _simulation.ResolvedPositions;
            var resolvedVelocities = _simulation.ResolvedVelocities;
            var positions = _simulation.Positions;
            var currentVelocities = _simulation.CurrentVelocities;
            var desiredVelocities = _simulation.DesiredVelocities;
            for (var i = 0; i < agentCount; i++)
            {
                var position = resolvedPositions[i];
                var direction = math.sign(desiredVelocities[i].x);
                if (direction > 0f && position.x > goalX ||
                    direction < 0f && position.x < -goalX)
                {
                    position.x = -direction * spawnX;
                    position.y = Mathf.Repeat(position.y + 3.7f, 7.4f) - 3.7f;
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
            var fileName = $"local-avoidance-sample-{DateTime.Now:yyyyMMdd-HHmmss}.csv";
            var path = Path.Combine(Application.persistentDataPath, fileName);
            _diagnosticWriter = new StreamWriter(path, false);
            // Existing sample scenes can deserialize a newly added serialized field as zero.
            // Always retain a useful opening capture window for diagnosing spawn behavior.
            _diagnosticOpeningEndTime = Time.time + Mathf.Max(4f, diagnosticOpeningDuration);
            _diagnosticWriter.WriteLine(
                "frame,time,index,group,pos_x,pos_y,desired_x,desired_y,resolved_vx,resolved_vy,lateral_v," +
                "speed_ratio,contacts,blocking_contacts,touching,constraint,constraint_nx,constraint_ny," +
                "allowed_normal_speed,first_correction_x,first_correction_y,last_correction_x,last_correction_y," +
                "neighbor_distance,collision_prediction_time,lateral_speed_ratio,lateral_flow_following," +
                "avoidance_weight");
            Debug.Log($"[LocalAvoidance.Sample] Diagnostic log: {path}");
        }

        private void WriteDiagnosticFrame()
        {
            if (_diagnosticWriter == null || Time.frameCount % diagnosticFrameInterval != 0) return;

            var positions = _simulation.ResolvedPositions;
            var desired = _simulation.DesiredVelocities;
            var velocities = _simulation.ResolvedVelocities;
            var contacts = _simulation.Contacts;
            var constraintNormals = _diagnostics.PreviousConstraintNormal;
            var allowedNormalSpeeds = _diagnostics.PreviousAllowedNormalSpeed;
            var lastIteration = math.min(_simulation.Settings.SolverIterations,
                LocalAvoidanceDiagnostics.MaximumSolverIterations) - 1;
            var invariant = CultureInfo.InvariantCulture;
            var captureOpening = Time.time <= _diagnosticOpeningEndTime;
            for (var i = 0; i < agentCount; i++)
            {
                var position = positions[i];
                if (!captureOpening && Mathf.Abs(position.x) > diagnosticRange) continue;

                var velocity = velocities[i];
                var intendedSpeed = math.length(desired[i]);
                var contact = contacts[i];
                var constraintNormal = constraintNormals[i];
                var firstCorrection = _diagnostics.GetSolverCorrection(i, 0);
                var lastCorrection = _diagnostics.GetSolverCorrection(i, lastIteration);
                _diagnosticWriter.Write(Time.frameCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(Time.time.ToString("F4", invariant));
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(i);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write((i & 1) == 0 ? "left" : "right");
                WriteFloat(position.x, invariant);
                WriteFloat(position.y, invariant);
                WriteFloat(desired[i].x, invariant);
                WriteFloat(desired[i].y, invariant);
                WriteFloat(velocity.x, invariant);
                WriteFloat(velocity.y, invariant);
                WriteFloat(velocity.y, invariant);
                WriteFloat(intendedSpeed > 1e-5f ? math.length(velocity) / intendedSpeed : 0f, invariant);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.AgentContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.BlockingAgentContactCount);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.IsTouching);
                _diagnosticWriter.Write(',');
                _diagnosticWriter.Write(contact.HasConstraint);
                WriteFloat(constraintNormal.x, invariant);
                WriteFloat(constraintNormal.y, invariant);
                WriteFloat(allowedNormalSpeeds[i], invariant);
                WriteFloat(firstCorrection.x, invariant);
                WriteFloat(firstCorrection.y, invariant);
                WriteFloat(lastCorrection.x, invariant);
                WriteFloat(lastCorrection.y, invariant);
                WriteFloat(neighborDistance, invariant);
                WriteFloat(collisionPredictionTime, invariant);
                WriteFloat(lateralSpeedRatio, invariant);
                WriteFloat(lateralFlowFollowing, invariant);
                WriteFloat(avoidanceWeight, invariant);
                _diagnosticWriter.WriteLine();
            }

            if (Time.unscaledTime >= _nextDiagnosticFlushTime)
            {
                _diagnosticWriter.Flush();
                _nextDiagnosticFlushTime = Time.unscaledTime + 1f;
            }
        }

        private void WriteFloat(float value, IFormatProvider provider)
        {
            _diagnosticWriter.Write(',');
            _diagnosticWriter.Write(value.ToString("F5", provider));
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
            if (_leftMaterial != null) Destroy(_leftMaterial);
            if (_rightMaterial != null) Destroy(_rightMaterial);
        }
    }
}
