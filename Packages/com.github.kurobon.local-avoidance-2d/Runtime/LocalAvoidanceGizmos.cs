#if UNITY_EDITOR
using UnityEngine;

namespace LocalAvoidance2D
{
    public enum LocalAvoidanceGizmoPlane : byte
    {
        XY,
        XZ
    }

    public struct LocalAvoidanceGizmoSettings
    {
        public int MaximumAgents;
        public bool DrawVelocity;
        public float VelocityScale;
        public LocalAvoidanceGizmoPlane Plane;
        public float Elevation;
        public Color DefaultColor;
        public Color ContactColor;
        public Color VelocityColor;

        public static LocalAvoidanceGizmoSettings Default => new()
        {
            MaximumAgents = 512,
            VelocityScale = .25f,
            Plane = LocalAvoidanceGizmoPlane.XY,
            DefaultColor = Color.cyan,
            ContactColor = Color.yellow,
            VelocityColor = Color.green
        };
    }

    /// <summary>Editor-only visualization for a LocalAvoidanceSimulation.</summary>
    public static class LocalAvoidanceGizmos
    {
        public static void Draw(LocalAvoidanceSimulation simulation, LocalAvoidanceGizmoSettings settings)
        {
            if (simulation == null || !simulation.Active.IsCreated) return;
            var active = simulation.Active;
            var positions = simulation.ResolvedPositions;
            var velocities = simulation.ResolvedVelocities;
            var radii = simulation.Radii;
            var contacts = simulation.Contacts;
            var remaining = settings.MaximumAgents <= 0 ? int.MaxValue : settings.MaximumAgents;
            var velocityScale = Mathf.Max(0f, settings.VelocityScale);

            for (var i = 0; i < simulation.Capacity && remaining > 0; i++)
            {
                if (active[i] == 0) continue;
                var position = settings.Plane == LocalAvoidanceGizmoPlane.XZ
                    ? new Vector3(positions[i].x, settings.Elevation, positions[i].y)
                    : new Vector3(positions[i].x, positions[i].y, settings.Elevation);
                Gizmos.color = contacts[i].IsTouching != 0
                    ? settings.ContactColor
                    : settings.DefaultColor;
                Gizmos.DrawWireSphere(position, radii[i]);

                if (settings.DrawVelocity)
                {
                    var velocity = velocities[i];
                    Gizmos.color = settings.VelocityColor;
                    var worldVelocity = settings.Plane == LocalAvoidanceGizmoPlane.XZ
                        ? new Vector3(velocity.x, 0f, velocity.y)
                        : new Vector3(velocity.x, velocity.y, 0f);
                    Gizmos.DrawLine(position,
                        position + worldVelocity * velocityScale);
                }
                remaining--;
            }
        }
    }
}
#endif
