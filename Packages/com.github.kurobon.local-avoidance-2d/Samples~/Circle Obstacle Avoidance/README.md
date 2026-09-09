# Circle Obstacle Avoidance

Open `CircleObstacleAvoidance.unity` and enter Play Mode.

A compact agent grid moves from left to right while preserving its grid-shaped destination layout, with one circular obstacle centered on the route. Each agent teleports back to its own spawn position after crossing its goal X, matching the continuous loop used by Basic Crowd Avoidance. There are no navigation waypoints. Set Agent Count to `1` to isolate circle avoidance, then increase it to test the interaction with agent-agent avoidance.

The diagnostic CSV reports each agent's destination, surface clearance, retained passing side, side switches, continuous stalled time, obstacle contacts, and goal arrival. A successful run reaches every goal without negative clearance, prolonged stalling, or repeated side switching.
