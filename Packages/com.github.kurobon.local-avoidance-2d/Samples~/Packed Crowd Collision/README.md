# Packed Crowd Collision

The blue crowd starts packed at the origin. The red crowd travels from left to right and collides with it.

- Leave **Moving Group Joins Packed Destination** disabled to observe two groups with different destinations.
- Enable it before Play mode to make the moving group join the packed crowd at the origin.

The sample demonstrates the transition from predictive travel to contact-density-driven packing and shows that a packed crowd remains dynamic rather than becoming a static obstacle.

When **Enable Diagnostic Log** is enabled, the sample writes
`local-avoidance-packed-collision-<timestamp>.csv` under `Application.persistentDataPath`.
The log identifies packed and moving agents and records destination sleep state, contacts,
resolved velocity, retained avoidance side and time, and the first and last Jacobi corrections.
