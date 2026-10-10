# Devpost submission draft — AquaMind

**Title:** AquaMind — Energy-Aware Autonomous Lionfish Control Simulator

**Tagline:** A simulated underwater robot for invasive-species monitoring and controlled capture, with native-wildlife safeguards.

## Inspiration
Invasive lionfish can disrupt reef ecosystems in parts of the Atlantic, Caribbean and Gulf of Mexico. We wanted to explore how resource-aware autonomous robotics might support marine conservation. Corpus Christi's coastal community motivated the *illustrative* ocean setting; this is not a verified survey of lionfish at any particular beach or port.

## What it does
AquaMind visualizes a robot deploying from a fishing-support vessel, scanning the water, choosing simulated lionfish targets, avoiding native fish and reef obstacles, slowing for a close approach, animating spherical containment, and storing up to eight captures. It returns to the boat, unloads, recharges and resumes searching. Live telemetry shows state, depth, remaining battery, targets, distance and energy consumed. Five cameras include robot POV.

## How we built it
Python + Panda3D. A* search on a navigation grid plans around obstacles. A finite-state machine handles mission phases. A rule-based species flag prevents the simulated robot from capturing native fish; it is **not** trained machine vision. A simple energy model evaluates whether a pursuit leaves enough reserve for returning. Procedural 3D geometry generates the environment, fish, boat, robot and containment animation.

## Challenges
**Customize with true team experience before publishing.** Example technical challenge: configuring VS Code to use the same Python environment as the Panda3D installation. We addressed it with a per-project `.venv` and explicit run settings.

## Accomplishments
**Verify these in a live run before publishing:** 3D visualization, eight-capture payload accounting, exclusion of native species, energy-aware target choice, return-to-boat mission flow, and camera telemetry.

## What we learned
How to combine a state machine, heuristic path planning, environmental modeling and a visual simulation into a coherent system under hackathon time constraints.

## What's next
Evaluate real species-detection models using annotated underwater imagery; test uncertainty handling, real reef maps, physical containment feasibility, safer equipment designs, power consumption, field supervision and environmental approvals.

## Built with
Python; Panda3D; VS Code; FreeCAD for conceptual design.

## Potential users / local relevance
Conservation researchers, marine-robotics educators and licensed removal teams *could evaluate* the concept. Padre Island National Seashore, Texas State Aquarium, and the Coastal Bend Bays & Estuaries Program are examples of local conservation contexts, **not project partners or confirmed deployment sites**. Lionfish presence and removal needs must be independently assessed for each location.

## Links to add
- GitHub repository: [ADD LINK]
- Recorded demo: [ADD LINK]
- Team members: [ADD NAMES]
- Screenshot / media: [ADD MEDIA]
