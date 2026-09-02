# Rectangle Obstacle Avoidance

Open `RectangleObstacleAvoidance.unity` and enter Play Mode.

- Agents receive only a constant rightward desired velocity.
- Local Avoidance predicts collisions with the four segment edges and steers the crowd around them.
- Contact resolution prevents penetration if predictive steering is insufficient.

Change the rectangle size, crowd settings, or prediction settings on the Sample object.

Set Avoidance Weight to zero to disable predictive agent avoidance while retaining obstacle non-penetration.

When Enable Diagnostic Log is enabled, a CSV containing obstacle clearance, avoidance side, velocity,
and contact state is written to `Application.persistentDataPath`.
