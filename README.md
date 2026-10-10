# 🌊 AquaMind
### Autonomous Lionfish Detection and Control System
**IslanderHack 2026 | Green AI & Resource Efficiency**

AquaMind is a Python-based 3D simulation of an autonomous underwater robot designed to help address the ecological challenges caused by invasive lionfish.

Our project explores how autonomous robotics, intelligent navigation, and energy-efficient decision-making could support marine conservation while potentially reducing the time, labor, and resources required for invasive species management.

Developed for IslanderHack 2026 at Texas A&M University–Corpus Christi.

---

## 🌎 The Problem

Invasive lionfish pose a threat to marine ecosystems in the western Atlantic, Caribbean, and Gulf of Mexico.

Lionfish reproduce rapidly, consume native reef fish, and can disrupt marine food webs. Their presence creates challenges for marine conservation efforts and can affect ecosystems that support local fisheries.

Currently, lionfish management often relies on human divers, organized removal programs, and manual capture methods.

These approaches require considerable time, equipment, and human involvement.

We wanted to explore a different approach:

**What if an autonomous underwater robot could help identify and remove invasive lionfish while minimizing energy consumption and protecting native marine life?**

That question inspired AquaMind.

---

## 🤖 Our Solution

AquaMind simulates an autonomous underwater vehicle (AUV) that identifies invasive lionfish, navigates toward them, and performs a simulated capture and neutralization using a spherical mechanism.

The robot operates alongside a fishing support vessel that serves as its deployment, collection, and recharging station.

The complete autonomous mission follows this sequence:

**DEPLOY → SEARCH → DETECT → VERIFY → APPROACH → CAPTURE → STORE → RETURN → UNLOAD → RECHARGE → REDEPLOY**

The robot can store up to eight simulated lionfish during each deployment.

Once its payload reaches capacity, or the battery approaches its safety reserve, the robot returns to the support vessel.

After unloading and recharging, it can automatically begin another mission.

The system is designed to demonstrate how autonomous decision-making could support more resource-conscious marine conservation.

---

## ✨ Key Features

### 🐠 Autonomous Lionfish Detection

The robot searches a simulated underwater environment containing invasive lionfish and non-target marine wildlife.

Each fish has its own position, movement speed, swimming behavior, and classification.

A simulated recognition system distinguishes lionfish from native fish.

Only lionfish are eligible for capture.

**Note:** The current prototype uses simulated species labels rather than a trained computer-vision model.

### 🧭 A* Pathfinding and Obstacle Avoidance

AquaMind uses the A* search algorithm to plan navigation routes around underwater obstacles.

The robot considers:
- Artificial reef structures
- Rocky formations
- Protected habitat zones
- Non-target wildlife exclusion zones
- Navigation boundaries

Instead of always moving directly toward a fish, the robot calculates a route through a navigation grid.

This improves navigation reliability in the simulated environment.

### ⚡ Energy-Aware Target Selection

AquaMind evaluates whether pursuing a lionfish is worthwhile before beginning an approach.

The decision system considers:
- Distance to the target
- Estimated propulsion energy
- Estimated capture energy
- Remaining battery capacity
- Energy required to return to the support vessel
- A safety reserve

If pursuing a target would compromise the robot's estimated ability to return safely, the robot rejects that target.

This resource-aware decision-making is central to AquaMind's Green AI concept.

### 🫧 Spherical Capture Mechanism

The robot uses a simulated extending spherical containment mechanism.

When an eligible lionfish is within the configured three-meter capture range:

1. The robot slows down.
2. The containment mechanism extends toward the target.
3. A spherical capture field expands around the fish.
4. The simulation displays a neutralization effect.
5. The containment system retracts.
6. The fish is transferred into the robot's simulated storage compartment.
7. The onboard capture count increases.

The robot can hold a maximum of eight lionfish per deployment.

The visual capture and neutralization effects are illustrative and do not represent a physically validated mechanism.

### 🚤 Support Vessel Integration

AquaMind includes a fishing support vessel that acts as the robot's operational base.

The robot can:
- Deploy from the support vessel
- Conduct autonomous underwater search missions
- Return when its payload reaches capacity
- Return when its energy reserve becomes insufficient
- Dock at the vessel
- Transfer simulated captured lionfish
- Recharge its battery
- Automatically redeploy

This creates a repeatable autonomous mission cycle.

### 📷 Multiple Camera Views

The simulation includes five camera modes:

1. Third-person robot view
2. Wide underwater environment view
3. Close robot-follow camera
4. Fishing support vessel overview
5. First-person robot camera

The robot POV includes simulated telemetry such as depth, speed, heading, battery level, target distance, and capture capacity.

### 🌊 Underwater Environment

AquaMind uses procedural 3D graphics to create a Gulf Coast-inspired underwater environment.

The simulation includes:
- Animated lionfish
- Non-target marine fish
- Rocky formations
- Artificial reef structures
- Coral-like habitats
- Underwater vegetation
- Suspended water particles
- Ocean fog and underwater lighting
- Animated ocean currents
- A fishing support vessel
- A custom underwater robot

The environment is inspired by coastal ecosystems rather than an exact reconstruction of Corpus Christi waters.

---

## 🧠 Technical Implementation

AquaMind was developed using Python and Panda3D.

The simulation combines several algorithms and programming techniques.

### 1. A* Search Algorithm

A* is used to calculate routes through the simulated underwater environment.

The algorithm evaluates navigation nodes using:

f(n) = g(n) + h(n)

Where:

- `g(n)` represents the known cost of reaching a node.
- `h(n)` estimates the remaining cost to the destination.
- `f(n)` estimates the total route cost.

AquaMind uses a grid-based representation of the environment and a Euclidean-distance heuristic.

Obstacle clearance is incorporated into the path planner to help avoid restricted regions.

**Implementation:** `Planner` class in `AquaMind_FINAL.py`.

### 2. Finite-State Machine

The robot's autonomous behavior is controlled by a finite-state machine.

Important states include:

- `DOCKED`
- `PREPARING`
- `DEPLOYING`
- `SEARCHING`
- `TARGET_DETECTED`
- `TARGET_VERIFICATION`
- `APPROACHING`
- `CAPTURING`
- `RETURNING_TO_BASE`
- `DOCKING`
- `UNLOADING`
- `RECHARGING`
- `REDEPLOYING`
- `LOW_BATTERY`
- `SAFE_HOLD`
- `MISSION_COMPLETE`

Each state controls a particular stage of the mission.

Transitions occur based on simulated conditions such as target eligibility, distance, payload capacity, and battery level.

**Implementation:** `Simulation` class in `AquaMind_FINAL.py`.

### 3. Simulated Species Classification

The recognition system uses predefined simulated fish attributes to distinguish lionfish from non-target species.

Before attempting capture, the controller checks whether:

- The target is classified as a lionfish.
- The target is still active.
- The capture capacity has not been reached.
- The target is outside protected exclusion zones.
- The estimated battery reserve is sufficient.

Native fish are excluded from capture.

**Implementation:** `Fish`, `eligible()`, `safe_zone()`, and `can_capture()`.

This is rule-based classification using simulation data, not a trained machine-learning model.

### 4. Energy-Aware Decision-Making

The simulation tracks energy consumption using a simplified model.

**Total Energy = Propulsion Energy + Sensor Energy + Capture Energy**

The robot estimates the energy required to approach a target, capture it, and return to its support vessel.

Targets are rejected when the estimated energy requirement exceeds the available battery.

This approach demonstrates how resource consumption can influence autonomous mission planning.

**Implementation:** `eligible()`, `energy_return()`, and the simulation's battery tracking system.

The energy model is illustrative and has not been calibrated against real underwater hardware.

### 5. Simplified Motion and Environmental Physics

The simulation uses a simplified movement model that includes:

- Robot acceleration
- Velocity-based movement
- Reduced approach speed
- Gradual changes in direction
- Animated fish swimming
- Simplified ocean-current effects
- Speed-dependent propeller animation

The robot uses twin ducted thrusters with counter-rotating animated propellers.

These animations represent the intended propulsion behavior rather than validated hydrodynamics.

### 6. Procedural 3D Rendering

Panda3D is responsible for:

- Rendering the underwater environment
- Generating procedural geometry
- Displaying lighting and fog
- Animating robot movement
- Animating marine wildlife
- Rendering the capture mechanism
- Managing camera views
- Displaying the real-time dashboard

The project does not require paid graphical assets, cloud rendering, or external APIs.

---

## 🌱 Green AI and Resource Efficiency

AquaMind was created around the IslanderHack theme of Green AI and resource efficiency.

Instead of pursuing every detected target, the robot evaluates energy requirements before committing to a capture.

The system tracks:

- Battery percentage
- Estimated energy consumption
- Distance traveled
- Captures per mission
- Total captures
- Deployment count
- Recharge cycles
- Return trips
- Mission duration

These metrics could support future research into more efficient autonomous conservation operations.

AquaMind does not yet demonstrate experimentally verified energy savings compared with human-operated removal methods.

---

## 📍 Potential Applications in South Texas

AquaMind was designed with the Corpus Christi coastal environment in mind.

Potential future applications include:

### Padre Island National Seashore

AquaMind could be adapted for underwater wildlife monitoring and invasive-species research, subject to habitat suitability, regulatory approval, and ecological assessment.

### Port of Corpus Christi

A future system could potentially assist authorized marine monitoring teams with underwater inspections and invasive-species surveillance in suitable operational areas.

### Texas State Aquarium

The simulation could support educational demonstrations involving marine robotics, invasive species, conservation, and autonomous systems.

These are proposed applications rather than confirmed partnerships.

**Important:** AquaMind does not claim that lionfish are currently a major documented problem at these specific South Texas sites.

---

## 💻 Installation and Running the Simulation

### Requirements

- Python 3.14 or another Panda3D-compatible Python version
- Panda3D
- macOS, Windows, or Linux
- A desktop environment capable of running a 3D graphics window

### macOS — VS Code

Download or clone the GitHub repository.

Open the entire project folder in VS Code.

Open a terminal inside the project folder and run:

```bash
python3.14 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python AquaMind_FINAL.py
```

After installation, launch the simulation using:

```bash
.venv/bin/python AquaMind_FINAL.py
```

Alternatively, select the project's `.venv/bin/python` interpreter in VS Code and use the included Run and Debug configuration.

### Windows — PowerShell

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe AquaMind_FINAL.py
```

### Linux

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python AquaMind_FINAL.py
```

If the graphics dependency is incompatible with your installed Python version, create the virtual environment using a supported Python version.

---

## 🎮 Simulation Controls

| Key | Action |
|-----|--------|
| Space | Pause / Resume |
| R | Reset simulation |
| C | Cycle camera views |
| 1 | Third-person camera |
| 2 | Wide underwater view |
| 3 | Close robot-follow camera |
| 4 | Support vessel overview |
| 5 | Robot POV camera |
| A | Toggle automatic redeployment |
| P | Show / Hide planned navigation paths |
| D | Toggle algorithm diagnostics |
| B | Toggle stylized blood effects |
| G | Toggle particle quality |
| + / - | Adjust simulation speed |
| Esc | Exit |

---

## 📂 Project Structure

```text
AquaMind_Devpost_Ready/
│
├── AquaMind_FINAL.py
├── requirements.txt
├── README.md
│
├── .vscode/
│   ├── launch.json
│   └── settings.json
│
├── tests/
│
├── docs/
│   ├── DEVPOST_SUBMISSION.md
│   └── JUDGE_CHEAT_SHEET.md
│
├── setup_mac.command
├── run_mac.command
│
├── HalfDiscCaptureRobot.FCMacro
└── AquaMind_IslanderHack_Judging_Deck.pptx
```

---

## 🧪 Testing

The project includes automated tests for core simulation logic.

These tests can be run without opening the Panda3D graphics window.

On macOS or Linux:

```bash
./.venv/bin/python -m unittest discover -s tests -v
```

The tests focus on areas such as:

- Mission initialization
- Species exclusion rules
- Capture capacity
- Battery management
- Navigation logic
- Mission state transitions
- Reset behavior

The 3D graphics require separate visual verification.

---

## 🚀 Future Improvements

AquaMind is currently a proof-of-concept simulation.

With additional development time and resources, we would work toward:

1. **Computer Vision:** Train a species-recognition model using labeled underwater images of lionfish and native species.

2. **Improved Navigation:** Integrate sonar and depth sensors for obstacle detection and autonomous underwater navigation.

3. **Physical Prototype:** Develop and test a real underwater robot based on the proposed half-disc design.

4. **Capture Mechanism Testing:** Evaluate whether the proposed spherical capture mechanism can operate safely and reliably underwater.

5. **Energy Optimization:** Compare navigation and target-selection strategies using repeatable experiments.

6. **Marine Conservation Partnerships:** Work with researchers and conservation organizations to determine appropriate deployment locations.

7. **Environmental Validation:** Evaluate effects on native wildlife, protected habitats, and marine ecosystems.

8. **Field Testing:** Conduct controlled experiments after appropriate safety reviews and regulatory approvals.

---

## 🏆 IslanderHack 2026

**Hackathon:** IslanderHack 2026

**Theme:** Green AI & Resource Efficiency

**University:** Texas A&M University–Corpus Christi

**Project:** AquaMind — Autonomous Lionfish Control Simulator

**Technology:** Python, Panda3D, A* Pathfinding, Autonomous State Machines, Procedural 3D Graphics

### Our Goal

To explore how automation and energy-aware autonomous systems could contribute to marine conservation while reducing unnecessary resource consumption.

---

## ⚠️ Project Limitations

AquaMind is a simulation and has not been deployed in a real marine environment.

Important limitations include:

- Species identification uses predefined simulated labels rather than trained computer vision.
- Robot movement uses simplified physics.
- Ocean currents and marine habitats are illustrative.
- The capture mechanism has not been physically validated.
- Energy estimates are simulated and are not measured hardware consumption.
- No real-world energy or cost savings have been established.
- No partnerships or field deployments are implied.
- The environment is inspired by the Gulf Coast rather than a geographically accurate reconstruction.

This project demonstrates a potential approach to autonomous invasive-species management, not a field-ready marine robot.

---

## 💙 Our Vision

AquaMind represents our vision of combining computer science, artificial intelligence concepts, and marine conservation to explore new ways of protecting underwater ecosystems.

By combining autonomous navigation, selective targeting, resource-aware decision-making, and repeatable missions, we hope to demonstrate how technology could support future conservation efforts.

**Protecting marine ecosystems through smarter automation.**

---

*Developed for IslanderHack 2026 at Texas A&M University–Corpus Christi.*
