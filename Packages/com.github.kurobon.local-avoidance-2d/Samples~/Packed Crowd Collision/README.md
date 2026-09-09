# Packed Crowd Collision

The blue crowd starts packed at the origin. The red crowd travels from left to right while two dark circular obstacles narrow the direct route. A configurable waypoint above or below the obstacles represents the route supplied by a navigation layer; local avoidance handles only nearby agents and obstacles along that route.

- Leave **Moving Group Joins Packed Destination** disabled to observe two groups with different destinations.
- Enable it before Play mode to make the moving group join the packed crowd at the origin.

The sample demonstrates the transition from predictive travel to contact-density-driven packing, keeps the packed crowd as dynamic agents, and shows the intended separation between navigation and local avoidance.

When **Enable Diagnostic Log** is enabled, the sample writes
`local-avoidance-packed-collision-<timestamp>.csv` under `Application.persistentDataPath`.
The log identifies packed and moving agents and records destination sleep state, contacts,
resolved velocity, retained avoidance side and time, and the first and last Jacobi corrections.
