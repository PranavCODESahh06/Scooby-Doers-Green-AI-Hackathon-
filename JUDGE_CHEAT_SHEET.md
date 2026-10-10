# AquaMind — judge answers (honest prototype framing)

**Where is the AI?** Autonomous planning and energy-aware rules. We have not trained a vision network. The species filter reads a simulation label, which is an explicit prototype limitation.

**How is navigation done?** `Planner.plan` uses A* across an obstacle-aware 2D grid, with destination depth represented in route waypoints. The heuristic is Euclidean distance. Current/obstacle avoidance is simplified.

**Why lionfish?** They are invasive predators in parts of the western Atlantic, Caribbean and Gulf of Mexico, presenting a conservation challenge. Avoid unsupported claims about Corpus Christi population counts.

**Does it avoid native species?** The simulated classifier checks `Fish.native` and the route/capture checks proximity exclusions. This is not verified camera recognition.

**How does it save resources?** The robot estimates propulsion distance and energy, sensor use, capture costs and a reserve for return, and declines targets that violate modeled energy budget. No externally validated cost savings are claimed.

**Why eight fish?** It's a design constraint to demonstrate capacity-aware mission logic, not a validated physical payload size.

**What happens after eight?** Return to support vessel, simulated unload, recharge, and redeploy while other lionfish remain.

**What would you do next?** Train and benchmark actual underwater computer vision, prototype the mechanism, test energy and hydrodynamics, add geofenced no-go habitat, consult marine biologists and regulators.

**Who would use it?** Potentially researchers and licensed marine-removal operators, with a user-facing dashboard. Partnerships and local field viability are not established.

**Hardest non-code challenge and workload split?** Supply your actual team's answers; do not invent contributions.
