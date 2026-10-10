# AquaMind — IslanderHack 2026 | Python 3D simulation

Autonomous lionfish-control **simulation** in a Gulf Coast-inspired environment. Built in Python with free Panda3D and procedural graphics; no paid services or missing models. An illustrative robot searches, selects lionfish, approaches, animates a spherical capture, holds up to eight, returns to its support vessel, unloads, recharges, and redeploys. Native fish are explicitly excluded.

## START HERE — macOS + VS Code

1. **Extract this ZIP**, then open **the whole `AquaMind_Devpost_Ready` folder** using VS Code > File > Open Folder. Do not open just `AquaMind_FINAL.py` in an empty/untitled workspace.
2. From VS Code **Terminal > New Terminal**, run this **once**:
   ```bash
   python3.14 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
   ```
   If `python3.14` is not found, use `python3` (a compatible version).
3. In VS Code, press **F5**, choose **AquaMind — Run 3D Simulation**. This uses the project's `.venv/bin/python` explicitly, so it won't invoke missing `/bin/sh: python`.
4. For the **top-right Python ▶ button**, press Cmd+Shift+P > **Python: Select Interpreter** > enter/select `.venv/bin/python`. Select **Run Python File**, **not** Run Code, if the Code Runner extension offers both. The included `.vscode/settings.json` also provides a Code Runner command, but F5 is the most reliable route.
5. Alternative: Double-click `setup_mac.command` (one-time install), then `run_mac.command` (launch). macOS may require right click > Open for downloaded scripts. Or use the commands below.

**Everyday terminal launch (macOS/Linux):**
```bash
cd /path/to/AquaMind_Devpost_Ready
./.venv/bin/python AquaMind_FINAL.py
```

**Windows — PowerShell (from extracted folder):**
```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe AquaMind_FINAL.py
```

**Linux:**
```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python AquaMind_FINAL.py
```

Python 3.14/macOS support depends on having an available Panda3D wheel for your platform. If installation fails, check the error and use a compatible Python such as 3.13 to create `.venv`; don't silently switch interpreters. You need a desktop graphics session, not a headless Devpost runner.

## Keyboard shortcuts

Space pause/resume · R reset · 1–5 views (5 robot POV with depth telemetry) · C cycle views · A auto redeploy · P path overlay · D diagnostics · B toggle stylized blood · G particle quality · +/- speed · Esc exit.

## Code map (single Python file)

* `Config`: editable simulation parameters.
* `Planner`: A* grid-based route planner using a Euclidean heuristic and obstacle clearance.
* `Simulation`: finite-state machine and environmental fish updates, native-fish exclusion, battery and target logic.
* `AquaMindApp`: Panda3D rendering, procedural fish and reef geometry, robot with two ducted animated rotors, boat and HUD.

**Identification disclaimer:** The code uses a simulated rule-based species flag (`Fish.native`), not image-based machine learning. **Energy disclaimer:** Wh numbers are illustrative—not field-validated. The capture/neutralization, robot scale, fluid motions and habitats are also simulated. We have not field-tested a real-world mechanism or established a local lionfish population survey.


## Testing

Run graphics-free test suite:
```bash
./.venv/bin/python -m unittest discover -s tests -v
```
(Tests load the logic portion of the single-file program without importing Panda3D.)
