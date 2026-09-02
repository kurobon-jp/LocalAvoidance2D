# Basic Crowd Avoidance

Open `BasicCrowdAvoidance.unity` and enter Play Mode.

- Blue and red agents move in opposite directions.
- Agents avoid one another while crossing.
- `BasicCrowdAvoidanceSample.cs` shows the complete input, `Step`, and output flow.
- A CSV diagnostic log is written to `Application.persistentDataPath` while Play Mode is running.

Change Agent Count, Radius, Speed, Neighbor Distance, Collision Prediction Time, Lateral Speed Ratio,
or Lateral Flow Following on the Sample object to test different densities and avoidance behavior.

Avoidance Weight controls per-agent predictive avoidance strength; set it to zero to disable prediction.
