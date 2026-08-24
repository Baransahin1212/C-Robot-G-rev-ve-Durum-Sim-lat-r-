# RobotSimulator

**C++ Robot Task and State Simulator**
*(C++ Robot Görev ve Durum Simülatörü)*

A command-line, event-driven finite-state-machine simulator for a robot's
mission lifecycle: load a JSON scenario, feed its events through a robot
state machine, log every event and transition, and produce a final mission
report.

## Purpose

This project simulates how a robot's high-level task controller might
react to a sequence of events — it does **not** control or connect to any
physical robot or sensor hardware. In summary:

- No physical robot, motors, or sensors are involved anywhere in this
  project. All "sensor" input is data read from a scenario file.
- Events (`ScenarioLoaded`, `ObstacleDetected`, `BatteryCritical`, …) are
  read from a JSON scenario file, one at a time.
- A finite-state machine (`RobotStateMachine`) determines what the robot's
  state becomes in response to each event, based on a fixed, centrally
  defined transition table.
- Transitions the state machine does not recognize for the current state
  are rejected safely — the state is left unchanged, the simulation
  continues, and the rejection is counted, never thrown as an exception.
- Every event received and every transition (accepted or rejected) is
  logged with a simulated timestamp.
- At the end of a run, a mission report summarizes the outcome (mission
  outcome, final state, event/transition counts, simulated end time).

This is an educational/demonstration simulator, not a certified robot
safety controller — see
[`docs/technical-decisions.md`](docs/technical-decisions.md#fail-safe--emergency-stop)
for an explicit statement of that boundary.

## Requirements / prerequisites

Tested development environment:

- Windows
- A C++17-compatible compiler — tested with **MSVC / Visual Studio 2022**
- **CMake** (3.20+)
- Git is optional for users — only needed if you want to clone the
  repository yourself rather than download a copy

Three dependencies are used and are **not** something you need to install
yourself:

- [GoogleTest](https://github.com/google/googletest) (`v1.15.2`)
- [nlohmann/json](https://github.com/nlohmann/json) (`v3.11.3`)
- [raylib](https://github.com/raysan5/raylib) (`6.0`) — used only by the
  `RobotSimulator3D` visual simulator's rendering/window/input layer (see
  [3D visual simulator](#3d-visual-simulator-phase-13m13n13o13p13q) below);
  the CLI `RobotSimulator` executable does not depend on it at all (see
  [CMake / library structure](#cmake--library-structure)).

All three are fetched automatically at configure time via CMake
`FetchContent` (requires network access the first time you configure the
project).

## Build instructions

From the project root, using the Visual Studio 2022 / MSVC generator:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Debug
```

This produces, among other targets, `build\Debug\RobotSimulator.exe`.

## Testing

Run the full automated test suite via CTest:

```powershell
cd build
ctest -C Debug --output-on-failure
```

To see the exact number of discovered tests without running them:

```powershell
ctest -C Debug -N
```

As of this writing, this discovers and passes **565 automated tests**
(verified immediately before writing this document — re-run the command
above to confirm the current count, since it will grow as the project
grows).

## Running the application

```text
RobotSimulator <scenario-file>
RobotSimulator --live --cycles <N>
RobotSimulator --live --cycles <N> --command-script <file>
RobotSimulator --live --cycles <N> --sensor-script <file>
RobotSimulator --live --cycles <N> --command-script <file> --sensor-script <file>
RobotSimulator --help
RobotSimulator -h
```

**Scenario mode** reads a finite JSON scenario file and runs it end-to-end
through `JsonScenarioSource -> Simulator`, exactly as described throughout
this document.

**Live mode** (`--live --cycles <N>`) instead runs `N` deterministic
polling cycles through the live runtime pipeline
(`SimulatedRobotHardware -> HardwareEventSource -> RobotRuntime ->
LiveRuntimeRunner`) - see [Architecture](#architecture) below. `N` must be
a non-negative integer that fully matches the value passed. Without a
command or sensor script, live mode uses `SimulatedRobotHardware`'s
default, safe sensor values (battery 100, no obstacle, no emergency stop)
unchanged for the whole run, and no mission-lifecycle command ever arrives,
so it naturally produces `N` `NoEvent` cycles and stays in `Idle`.

**Scripted sensor injection** (`--live --cycles <N> --sensor-script
<file>`) additionally replays a deterministic script of sensor changes at
specific cycles, through `SensorScript -> ScriptedLiveRuntimeRunner ->
SimulatedRobotHardware`, still feeding the same unmodified
`HardwareEventSource -> RobotRuntime -> RobotStateMachine` pipeline - see
[Sensor script format](#sensor-script-format) below and
`docs/technical-decisions.md` (Phase 13I) for the full separation
rationale.

**Scripted command input** (`--live --cycles <N> --command-script <file>`)
separately replays a deterministic script of mission-lifecycle/operator
commands (`scenario_loaded`, `start_mission`, `mission_completed`,
`home_reached`, `reset`) at specific cycles - the events live mode has no
other way to raise, since `HardwareEventSource` only ever observes sensor
state and never fabricates mission lifecycle events. See [Command script
format](#command-script-format) below and `docs/technical-decisions.md`
(Phase 13J). `--command-script` and `--sensor-script` are independent and
may be combined; when both are given, `--command-script` must come first
(`--live --cycles <N> --command-script <file> --sensor-script <file>`) -
any other ordering is a usage error.

### Sensor script format

One entry per non-empty, non-comment line:

```text
<cycle> <sensor> <value>
```

```text
0 obstacle false
3 obstacle true
8 obstacle false
12 battery 15
15 emergency true
```

- **Cycles are zero-based.** `--live --cycles 5` executes cycles `0`
  through `4`.
- Each entry's mutation is applied to `SimulatedRobotHardware` immediately
  **before** the runtime step for its cycle runs - `2 obstacle true` means
  `hardware.setObstacleDetected(true)` happens right before cycle `2`'s
  `RobotRuntime::step()`.
- Entries sharing the same cycle are applied in **file order**.
- Lines are not required to already be sorted by cycle - the parser sorts
  them internally.
- Blank lines are ignored; lines whose first non-whitespace character is
  `#` are comments and are ignored.
- `sensor` is one of `obstacle`, `battery`, `emergency`. `value` is
  `true`/`false` for `obstacle`/`emergency`, or an integer `0`-`100` for
  `battery`.
- An entry scheduled at or beyond `--cycles <N>` is not an error - it
  simply never executes.
- The script only mutates `SimulatedRobotHardware`; `HardwareEventSource`
  then detects the resulting sensor edges exactly as it would with any
  other hardware change, and the FSM only accepts transitions its existing
  rules already allow (live mode starts in `Idle`, which does not accept
  obstacle/battery/emergency events - see `docs/technical-decisions.md`).
- Live mode without `--sensor-script` is unaffected and behaves exactly as
  in Phase 13H.

### Command script format

One entry per non-empty, non-comment line:

```text
<cycle> <command>
```

```text
0 scenario_loaded
1 start_mission
12 mission_completed
```

Supported command names, each mapped 1:1 to an existing `EventType` - no
new event vocabulary is introduced:

| Command name | `EventType` |
|---|---|
| `scenario_loaded` | `ScenarioLoaded` |
| `start_mission` | `StartMission` |
| `mission_completed` | `MissionCompleted` |
| `return_home` | `ReturnHomeRequested` (Phase 13T) |
| `stop_task` | `StopTaskRequested` (Phase 13U) |
| `home_reached` | `HomeReached` |
| `reset` | `Reset` |

Sensor-derived events (`ObstacleDetected`, `ObstacleCleared`,
`BatteryCritical`, `EmergencyStop`, `InvalidSensorData`) are **not**
accepted here - those remain `SensorScript`/`HardwareEventSource`'s
exclusive responsibility (see above), and appear as unknown commands if
used in a command script.

- **Cycles are zero-based**, same as sensor scripts.
- An entry becomes *eligible* once its scheduled cycle is reached, and
  raises its `Event` on the runtime step for the **first** cycle at or
  after that point where it is actually polled - if `RobotRuntime::step()`
  processes a different event that same cycle (or a same-cycle command
  ahead of it), the entry remains pending and is delivered on a later
  cycle. It is never dropped.
- Entries sharing the same cycle are delivered in **file order**.
- Lines are not required to already be sorted by cycle - the parser sorts
  them internally.
- Blank lines are ignored; lines whose first non-whitespace character is
  `#` are comments and are ignored.
- When both `--command-script` and `--sensor-script` are used together, a
  command event scheduled for the same cycle as a sensor edge is always
  delivered to the FSM **first** - see `docs/technical-decisions.md`
  (Phase 13J) for the full priority rationale.
- Live mode without `--command-script` is unaffected.

Windows Debug example, run from the project root:

```powershell
.\build\Debug\RobotSimulator.exe .\scenarios\normal_mission.json
```

Example output:

```text
Scenario: .\scenarios\normal_mission.json

Simulation Result
-----------------
Mission outcome: Completed
Final state: Completed
Events processed: 3
Successful transitions: 3
Rejected transitions: 0
Simulation end time: 5000 ms
```

Live mode example:

```powershell
.\build\Debug\RobotSimulator.exe --live --cycles 5
```

```text
Live runtime completed
Cycles executed: 5
No-event cycles: 5
Accepted transitions: 0
Rejected transitions: 0
Final state: Idle
```

Scripted live mode example:

```powershell
.\build\Debug\RobotSimulator.exe --live --cycles 20 --sensor-script sensors.txt
```

```text
Sensor script: sensors.txt
Live runtime completed
Cycles executed: 20
No-event cycles: 16
Accepted transitions: 0
Rejected transitions: 4
Final state: Idle
```

Live mission example, combining `--command-script` and `--sensor-script`
(using the `mission_commands.txt`/`obstacle_sensor.txt` fixtures under
`tests/fixtures/`, which contain exactly the two scripts below):

```powershell
.\build\Debug\RobotSimulator.exe --live --cycles 15 --command-script mission_commands.txt --sensor-script obstacle_sensor.txt
```

```text
# mission_commands.txt
0 scenario_loaded
1 start_mission
12 mission_completed
```

```text
# obstacle_sensor.txt
4 obstacle true
7 obstacle false
```

```text
Command script: mission_commands.txt
Sensor script: obstacle_sensor.txt
Live runtime completed
Cycles executed: 15
No-event cycles: 10
Accepted transitions: 5
Rejected transitions: 0
Final state: Completed
```

Concretely, this run's five accepted transitions are: `ScenarioLoaded`
(`Idle -> Ready`, cycle 0), `StartMission` (`Ready -> Moving`, cycle 1),
`ObstacleDetected` (`Moving -> WaitingForObstacleClear`, cycle 4),
`ObstacleCleared` (`WaitingForObstacleClear -> Moving`, cycle 7), and
`MissionCompleted` (`Moving -> Completed`, cycle 12) - the exact,
unmodified FSM transition table in action, driven entirely by the two
independent scripts.

### Exit codes

| Code | Meaning |
|---|---|
| `0` | The simulation executed successfully — **regardless of mission outcome**. `Aborted`, `EmergencyStopped`, `Error`, and a scenario containing rejected transitions are all legitimate simulation *results*, not application failures. |
| `1` | Invalid command line (missing or too many arguments, or malformed `--live`/`--sensor-script`/`--command-script` syntax, including `--sensor-script` given before `--command-script`). |
| `2` | The scenario file could not be loaded (not found, malformed JSON, or fails schema validation), or a `--sensor-script`/`--command-script` file could not be opened or contained a malformed line. |
| `3` | An output file (log or report) could not be created or written. |

### Generated runtime outputs

Each successful run (over)writes:

```text
logs/simulation.log
reports/simulation_report.txt
```

These directories are created automatically if they don't exist, and their
contents reflect only the most recent run. **Both `logs/` and `reports/`
are ignored by Git** — they are runtime output, not source. Representative
*tracked* examples generated by real runs are committed instead, under
[`docs/examples/`](docs/examples/).

## 3D visual simulator (Phase 13M/13N/13O/13P/13Q)

Alongside `RobotSimulator` (the CLI/FSM simulator documented above, which
is completely unchanged), there is now a second, separate executable:

```text
RobotSimulator3D
```

Run it directly - no arguments:

```powershell
.\build\Debug\RobotSimulator3D.exe
```

**Final UI/HUD polish: all on-screen interface text is Turkish by
default.** The HUD, Mission Control panel, and key hints described
throughout this section now render in Turkish (see the exact key labels
and HUD structure just below) - only the internal C++ identifiers/enum
names and this document's own English prose describing them are
unchanged. Where this section quotes an internal value name (`State`,
`Command: Stopped`, `Authority: AUTONOMOUS`, etc.) below, that documents
the underlying telemetry concept, not the literal text now drawn on
screen. Turkish diacritics (ç/Ç, ğ/Ğ, ı, i/İ, ö/Ö, ş/Ş, ü/Ü) render via a
bundled, repository-local font - **Anonymous Pro Bold** by Mark Simonson,
SIL Open Font License 1.1, at `assets/fonts/anonymous_pro_bold.ttf` (see
`assets/fonts/LICENSE-AnonymousPro.txt`) - used only for RobotSimulator3D's
own UI text, loaded via `raylib`'s `LoadFontEx()`. The build copies this
font next to `RobotSimulator3D.exe` (`assets/fonts/anonymous_pro_bold.ttf`,
relative to the executable), and `Renderer3D` resolves that same path at
runtime from the executable's own directory (`GetModuleFileNameA()`) -
never a developer machine's source-tree path - falling back to raylib's
built-in default font if the file cannot be read.

It opens a real 1280x720 window titled **"Robot Simulator 3D"** (starting
in borderless fullscreen - press `F11` to toggle windowed) showing a small
hard-coded demo world: a ground plane and grid, a two-wheel robot model
(body, two wheels, a red front heading marker), four box obstacles, and a
base/docking platform, viewed through a perspective camera you can orbit
with the mouse (`TAB` toggles mouse capture, drag to rotate, scroll to
zoom - raylib's built-in `CAMERA_FREE` mode).

**As of Phase 13U, the robot does NOT move on its own.** It starts `Idle`,
with the on-screen **Mission Control** panel (top-right) reading
`Task: NONE` - you explicitly assign a task:

```text
1  Gezinmeyi Başlat  (Start Roam: Idle/Ready -> ... -> Moving)
2  Eve Dön           (Return Home: Moving/Ready -> ReturningHome)
3  Görevi Durdur     (Stop Task: cancels whatever is currently running -> Ready)
R  Eve Dön           (Return Home, alias for 2, preserved from Phase 13T)
```

`1` reaches `Moving` through the FSM's own real transition rules -
`ScenarioLoaded` then `StartMission` from `Idle` (two Events, delivered on
two separate frames - never both forced through in one `RobotRuntime::step()`
call), or just `StartMission` if already `Ready`. Once `Moving`, the robot
"roams" the tabletop using exactly the same reactive layers Return Home
navigation already relies on - plain FSM `MoveForward` intent, body-width
obstacle perception and avoidance, and cliff/table-edge safety - no new
locomotion algorithm exists purely to make it wander; heading changes
naturally over time as it reacts to obstacles and edges. `3` cancels
whatever task is currently active (Roam, an in-progress Return Home, or
either one paused by an obstacle) and returns to the reusable `Ready`
state - `1`/`2`/`R` all work again immediately afterward. See "Mission
Control" below for the full command-to-Event mapping, the automatic Home
Zone return trigger, and why a normal Stop never cancels an active table-
edge safety recovery.

An on-screen engineering HUD (top-left, `H` toggles Sade/Ayrıntılı - see
below - shown internally as `HudMode::Compact`/`HudMode::Full`)
additionally shows
the robot's live FSM state/command, (Phase 13Q/13S/13T) drive authority
(`FSM`/`NAVIGATION`/`AUTONOMOUS`/`MANUAL`/`SAFETY`) and whether reactive
avoidance is enabled, (Phase 13R) whether the avoidance latch is currently
active and whether the forward BODY-clearance corridor is clear or
blocked, (Phase 13S) each of the four cliff sensors' SAFE/EDGE reading
plus whether table-edge safety is currently active and its recovery
state, (Phase 13T) the current Return Home navigation state
(`Inactive`/`Aligning`/`Driving`/`Arrived`), remaining distance to base,
target heading, and live heading error, position, heading, obstacle
count, (Phase 13O) the forward distance sensor's live reading, (Phase
13P) the current left/right wheel speeds and whether the last movement
was rejected by the obstacle-collision guard (`Collision: YES`/`NO`).
`SPACE` pauses/resumes world movement only (camera and FSM stepping are
unaffected). `O` toggles the one demo obstacle placed directly in the
robot's path, enabled/disabled, to demonstrate obstacle clearing. `M`
toggles manual drive mode (Phase 13P, see below); while it is on, arrow
keys drive the wheels directly (`UP`/`DOWN` forward/reverse,
`LEFT`/`RIGHT` turn) and holding `X` stops them immediately and takes
priority over every other key, for as long as it is held. `A` toggles
reactive obstacle avoidance (Phase 13Q, ON by default). `1`/`2`/`3`/`R`
are Mission Control's task-assignment keys - see above and "Mission
Control" below. `H` toggles the HUD between **Sade** ("simple" - a small
~6-8-line summary: `Durum`/`Görev`/`Kontrol`/`Engel`/`Kenar`/`Batarya`
plus at most one contextual line, never hiding an active safety recovery
or obstacle hazard - the **default** as of the final UI/HUD polish) and
**Ayrıntılı** ("detailed" - the full engineering telemetry panel
described above, translated but otherwise unchanged). This is a
presentation-only toggle - it has no effect on robot behavior in either
mode; see `docs/technical-decisions.md` (UX polish) for the original
Compact-mode field list this Sade panel superseded. Close the window
normally to exit.

**As of Phase 13N, the on-screen robot is actually driven by the real,
unmodified robot-control stack** - the same `RobotStateMachine`/
`RobotController` the CLI uses; **as of Phase 13O, obstacle stop/resume is
driven by a real geometry-based forward distance sensor feeding the same
real, unmodified `HardwareEventSource`/`CompositePollingEventSource` the
CLI's `--live` mode uses**:

```text
DemoCommandSource (visual-only, delivers ScenarioLoaded then StartMission)
      |
      v
CompositePollingEventSource   (command-before-sensor priority, unmodified)
      |                    \
      v                     \
  RobotRuntime          HardwareEventSource   (unmodified)
      |                     ^
      v                     |
RobotStateMachine     VirtualRobotHardware.obstacleDetected()
      |                     ^
      v                     |
 RobotController      VirtualDistanceSensor   (geometry-based, raylib-free)
      |                     ^
      v                     |
VirtualRobotHardware  ------+   (IRobotHardware implementation, visual-only)
      |
      v
  VirtualWorld robot pose / obstacle geometry
      |
      v
   Renderer3D   ->   visible robot movement + sensor ray + HUD telemetry
```

`main3d.cpp` calls `RobotRuntime::step()` exactly once per rendered frame
(the render loop itself is the scheduler - no `LiveRuntimeRunner` inside
the render loop) and `VirtualRobotHardware::update(GetFrameTime())` once
per frame afterward (skipped while `SPACE`-paused), so the robot starts
stationary, transitions through `Ready` into `Moving` within the first
couple of frames via the FSM's real transition rules (never forced
directly), and then visibly moves forward in a straight line along its
current heading, matching the direction the red front marker points - until
`VirtualDistanceSensor` reports the one demo obstacle placed in its path
within `kDetectionDistance` (1.0 world units), at which point
`HardwareEventSource` emits `ObstacleDetected`, `RobotStateMachine`
transitions `Moving -> WaitingForObstacleClear` through its real, unmodified
transition rules, and `RobotController` stops the hardware - the robot
visibly halts before reaching the obstacle. Pressing `O` disables that
obstacle; `VirtualDistanceSensor` then reports no hit, `HardwareEventSource`
emits `ObstacleCleared`, the FSM returns to `Moving`, and the robot resumes.
main3d.cpp never injects `ObstacleDetected`/`ObstacleCleared` itself - both
only ever come from `HardwareEventSource` reading `VirtualRobotHardware`'s
real sensor state, exactly like the CLI's `--live` mode.

**As of Phase 13P, movement is real differential-drive kinematics, not just
straight-line motion.** `VirtualRobotHardware` owns a `DifferentialDrive`
(raylib-free, no FSM/Event/RobotController knowledge of its own - see
`include/robot/visual/DifferentialDrive.hpp`) and maps `moveForward()` to
equal positive left/right wheel speeds, `stop()`/`returnToBase()` to zero -
the *same* FSM-driven behavior as Phase 13N/13O, now expressed as
kinematics instead of a hand-written "move along heading" calculation:

```text
left wheel speed, right wheel speed
              |
              v
  v = (vRight + vLeft) / 2        (robot linear velocity)
  omega = (vRight - vLeft) / L    (robot angular velocity, L = wheel track)
              |
              v
     x / z / heading (RobotPose)
```

The normal FSM path is unchanged end to end: `Moving` still means equal
positive wheel speeds and straight movement; `WaitingForObstacleClear`
still means zero wheel speeds via `RobotController::stop()`. **No new FSM
states, turn events, or `IRobotHardware` methods were added** - turning
capability exists only inside `DifferentialDrive`/`VirtualRobotHardware`.

To prove the robot can actually turn without touching `IRobotHardware`,
`RobotController`, `RobotRuntime`, or `RobotStateMachine`, `RobotSimulator3D`
has a manual-drive debug mode local to `main3d.cpp`: press `M` to toggle it
(the HUD's "Drive mode" line shows `MANUAL`), then `UP`/`DOWN` drive both
wheels forward/backward, `LEFT`/`RIGHT` turn (arc while combined with
`UP`/`DOWN`, in-place when held alone), and `X` stops the manual wheels.
Arrow keys were chosen over `WASD` to avoid `CAMERA_FREE`'s own `WASD`
panning - but raylib's `UpdateCamera()` separately reads the *same*
`UP`/`DOWN`/`LEFT`/`RIGHT` keys for camera pitch/yaw (plus continuous
mouse-look while the cursor is captured), so **camera updates are
suppressed entirely while manual drive mode is active** - the camera holds
perfectly still, and arrow keys/mouse only ever drive the robot until `M`
is pressed again. (An earlier version of this phase left camera updates
running during manual mode; the same keys silently also rotated the
camera every frame, which could make correctly-moving wheels look like the
robot wasn't moving at all - see `docs/technical-decisions.md`, Phase
13P.) While manual mode is on, `RobotRuntime` keeps polling FSM events
every frame exactly as before,
but the manual wheel speeds win for physical movement - obstacle-triggered
`WaitingForObstacleClear` can still fire, but manual driving is not stopped
by it, since proving kinematics is this mode's whole purpose. Pressing `M`
again immediately restores the wheel speeds matching whatever
`VirtualDriveCommand` `RobotController` most recently issued.

**A small collision guard prevents the robot from physically entering an
enabled obstacle - including in manual mode.** `VirtualRobotHardware::update()`
validates the proposed position against a conservative circular footprint
around the robot (`RobotCollision.hpp`, radius derived from
`RobotDimensions`) versus each enabled obstacle's X/Z footprint; on
collision, only the position is rejected (kept at its previous value) -
the heading still updates, so turning at an obstacle boundary is never
blocked, only driving into it is. This is a last-resort physical guard,
**not a full collision solver** - no bounce, sliding, or force response
exists, it only ever keeps the robot's last-committed position outside
obstacle geometry. In normal (non-manual) mode `VirtualDistanceSensor`/
`HardwareEventSource` still stop the robot well before this guard would
ever engage, exactly as in Phase 13O; the guard exists specifically for
manual mode, where the FSM's stop is deliberately bypassed. The HUD's
"Collision" line shows `YES` on any frame this guard rejects a move. See
`docs/technical-decisions.md` (Phase 13P) for the exact geometry and the
`X`-stop bug this same round of real-world testing also caught and fixed.

**As of Phase 13Q, `RobotSimulator3D` demonstrates simple REACTIVE
obstacle avoidance - the robot can now turn itself away from an obstacle
and continue, without any human pressing a key.** This is explicitly
**not** pathfinding, A*, waypoint planning, SLAM/mapping, or general
navigation - the entire policy is "turn in place until the forward path
clears, then continue," and it may permanently change the robot's heading
with no attempt to return to its original trajectory. Three things can now
own the physical wheels, in a fixed, explicit priority visible in code and
tested directly (`VirtualRobotHardware::driveAuthority()`):

```text
Manual  >  AutonomousAvoidance  >  Fsm
```

`currentCommand()` (what `RobotController`/the FSM *wants*) and
`driveAuthority()` (who currently owns the physical wheels) are
deliberately different things - while avoiding, the HUD routinely shows
`Command: Stopped` (`RobotController` still wants the robot stopped, per
Phase 13O's real `WaitingForObstacleClear` -> `stop()` mapping, completely
unmodified) alongside `Drive authority: AUTONOMOUS` with opposite-sign
wheel speeds - proof that the visual autonomous-locomotion layer, not the
FSM, is temporarily driving the wheels. `ReactiveObstacleAvoidance`
(`include/robot/visual/ReactiveObstacleAvoidance.hpp`, raylib-free,
stateless) always returns the same fixed in-place-turn wheel speeds
(`-0.6`/`+0.6`); `main3d.cpp` engages them - via
`VirtualRobotHardware::setAutonomousWheelSpeeds()`, one priority level
below the manual override - only while avoidance is enabled (`A`), the FSM
is actually `WaitingForObstacleClear`, and the forward sensor still
reports the obstacle. The moment any of those stops holding - including
the real `ObstacleCleared` edge below - the override releases:

```text
Moving -> obstacle detected -> WaitingForObstacleClear
                                       |
                                       v
                    autonomous avoidance takes temporary wheel authority
                                       |
                                       v
                     robot turns in place (heading changes, position fixed)
                                       |
                                       v
                       forward sensor eventually reports clear
                                       |
                                       v
              HardwareEventSource observes true -> false, emits ObstacleCleared
                                       |
                                       v
                          FSM returns to Moving (real transition rules)
                                       |
                                       v
                    autonomous override releases; RobotController::moveForward()
                                       |
                                       v
                     robot moves forward again, on its NEW heading
```

Exactly like Phase 13P's manual mode, **this never calls
`stateMachine.processEvent()`/`handleEvent()` or injects
`ObstacleDetected`/`ObstacleCleared` directly** - `HardwareEventSource`
observes the sensor's real edge, entirely because the turn actually
rotated the sensor ray away from the obstacle. `A` toggles the whole
policy off; with it off, `WaitingForObstacleClear` behaves exactly like
Phase 13O/13P - the robot just stays stopped until the obstacle clears
some other way (e.g. `O`). Manual mode always outranks avoidance: pressing
`M` mid-turn hands the wheels to manual immediately, and the avoidance
request is not lost - it resumes automatically the instant `M` is pressed
again, with no need to re-trigger it. **No new `RobotState` values, FSM
transitions, or `IRobotHardware` methods were added** - `RobotController`'s
`RobotState -> IRobotHardware` mapping is unchanged from Phase 13N, and
`HardwareEventSource`/`CompositePollingEventSource` are unchanged from
Phase 13J/13O. Autonomous turning still goes through the full Phase 13P
collision guard exactly like manual driving does - a circular collision
footprint is rotation-independent, so in-place avoidance turning is never
blocked by it, but the guard can still cap how far the robot travels once
it resumes forward on its new heading if that heading did not turn out to
be quite clear enough (this is expected V1 behavior, not a bug - see
`docs/technical-decisions.md`, Phase 13Q, "limitations").

**As of Phase 13R, avoidance is body-clearance-aware, closing the Phase
13Q limitation above.** The bug: "the forward sensor RAY is clear" is not
the same fact as "the robot's physical BODY has room to move forward" -
`VirtualDistanceSensor` casts a single zero-width ray, but the robot's
actual collision footprint (`RobotCollision.hpp`) is a circle with real
width. A heading could clear the narrow ray, hand control back to the FSM
via the real `ObstacleCleared` edge, and then have `RobotController`'s
`moveForward()` immediately run straight back into the very obstacle the
ray had just "cleared" - `Moving` with zero further physical progress,
repeatedly capped by the collision guard. Three distinct safety concepts
now exist side by side, each with one job:

```text
VirtualDistanceSensor    -> perception: single forward ray, drives the real
                             ObstacleDetected/ObstacleCleared Events
                             (unchanged, Phase 13O)
ForwardClearanceProbe    -> avoidance's release condition: does the robot's
                             swept BODY corridor have room to move forward
                             (new, Phase 13R)
RobotCollision            -> final physical guard: unconditionally rejects
                             any proposed pose that would penetrate an
                             obstacle, regardless of what the other two
                             report (unchanged, Phase 13P)
```

`ForwardClearanceProbe` (`include/robot/visual/ForwardClearanceProbe.hpp`,
raylib-free, headless) answers "does the robot's body have a physically
safe corridor to move forward along its CURRENT heading for
`kLookaheadDistance` (1.4F) world units?" - a Minkowski-sum-style test:
each enabled obstacle's X/Z footprint is expanded outward by
`RobotCollision`'s own `kRobotCollisionRadius` (the *same* source of
truth the physical guard uses, never a second hand-duplicated body
dimension) plus a small `kSafetyMargin` (0.08F), and the finite forward
center-line segment (robot center -> robot center +
`forwardDirection(pose) * kLookaheadDistance`) is tested against that
expanded box. `ReactiveObstacleAvoidance` (Phase 13Q) is now a small
stateful LATCH instead of a stateless per-frame recomputation - once
triggered (avoidance enabled, FSM `WaitingForObstacleClear`, sensor still
detected), it stays `active()` across frames - including frames where the
trigger condition has already gone false, e.g. the instant the real
`ObstacleCleared` edge returns the FSM to `Moving` - until
`ForwardClearanceProbe::isForwardCorridorClear()` is also true:

```text
Moving -> ObstacleDetected -> WaitingForObstacleClear -> Command: Stopped
                                       |
                                       v
                    avoidance latch activates, robot turns in place
                                       |
                                       v
                     forward sensor RAY eventually clears
                                       |
                                       v
        HardwareEventSource naturally emits ObstacleCleared; FSM -> Moving
                                       |
                                       v
       ** latch stays active if ForwardClearanceProbe still reports BLOCKED **
                                       |
                                       v
              State: Moving, Command: MoveForward, Drive authority: AUTONOMOUS
                          (a real, expected, tested intermediate state)
                                       |
                                       v
                        robot keeps turning; body corridor clears
                                       |
                                       v
                   latch deactivates; drive authority returns to FSM
                                       |
                                       v
                  equal positive wheel speeds take effect; robot advances
                  without the collision guard immediately rejecting it
```

`currentCommand()` (`MoveForward`) and `driveAuthority()` (`AUTONOMOUS`)
can legitimately disagree for several frames after the FSM has already
returned to `Moving` - this is intentional and directly tested
(`ClearanceAwareAvoidanceClosedLoopThroughRealEventChain` in
`tests/visual/VirtualRobotHardwareTests.cpp`), not a contradiction. Manual
mode still always outranks avoidance (`Manual > AutonomousAvoidance >
Fsm`, unchanged): the latch stays logically active underneath a manual
override and is restored automatically the instant manual mode ends,
exactly like Phase 13Q. **`ForwardClearanceProbe` and
`ReactiveObstacleAvoidance` remain entirely raylib-free**, and this is
still **reactive avoidance only** - no A*, Dijkstra, SLAM, map building,
waypoint navigation, target tracking, or return-to-original-path
behavior; the robot still does not attempt to recover its original
heading once avoidance releases. The HUD gained two lines: `Avoidance
active: YES/NO` (the latch's `active()`, distinct from the `Avoidance:
ON/OFF` enable toggle above it) and `Forward clearance: CLEAR/BLOCKED`
(this frame's `isForwardCorridorClear()` reading), plus `Clearance
lookahead` showing the constant in use. See `docs/technical-decisions.md`
(Phase 13R) for the full geometry derivation and latch semantics.

**As of Phase 13S, the physical robot is modeled as sitting on a finite
tabletop, not an unbounded plane - the old ~10x10 "invisible wall" that
simply clamped the robot in place at the world bound is gone.** Outside
`VirtualWorld::tableSurface()`'s rectangle is not a solid wall - it
represents a drop off the edge of the table, so it is deliberately never
represented as a `RobotCollision` obstacle AABB. Four new pieces work
together:

```text
VirtualDistanceSensor          -> solid-obstacle perception (unchanged)
ForwardClearanceProbe          -> safe obstacle-avoidance corridor (unchanged)
RobotCollision                 -> solid-obstacle penetration guard (unchanged)
VirtualCliffSensor             -> supporting-surface / table-edge detection (new)
TableEdgeSafetyController      -> emergency edge-recovery policy (new)
```

`VirtualCliffSensor` (`include/robot/visual/VirtualCliffSensor.hpp`,
raylib-free, headless) models four downward-looking corner sensors -
`FrontLeft`/`FrontRight`/`RearLeft`/`RearRight` - placed exactly at the
robot's rectangular footprint corners (`RobotDimensions::kBodyWidth`/
`kBodyLength`, the same source of truth every other geometry component
uses), rotated with `pose.headingDegrees` via a new `rightDirection()`
helper alongside the existing `forwardDirection()` (`VisualMath.hpp`).
Each reports `true` ("cliff detected," documented explicitly given how
dangerous an inverted convention would be here) when its corner's X/Z
position falls outside the table rectangle. `TableEdgeSafetyController`
(raylib-free, stateful) converts those readings into emergency wheel
speeds through a small V1 recovery state machine - these are internal
recovery states, entirely separate from `RobotStateMachine`:

```text
Inactive -> BackingAway (front cliff)              -> Turning -> AdvancingInward -> Inactive
Inactive -> MovingForwardFromRearEdge (rear cliff)  -> Turning -> AdvancingInward -> Inactive
```

Two separate conditions gate release, never collapsed into one boolean:
**heading alignment** (is the robot pointed toward the table center?) and
**support margin** (is the WHOLE footprint robustly - not just barely -
back on the table?). Human manual validation of the first version of this
phase found "all sensors safe" alone was not sufficient - on a straight
edge, a single simulation step's rotation was frequently enough to
satisfy it, releasing safety with an almost-unchanged, still-edge-facing
heading (a visible back-and-forth ping-pong at the edge). That was fixed
by adding the heading-alignment requirement
(`|shortestSignedHeadingErrorDegrees(pose.headingDegrees, target)| <=
10.0F`) - but a **second** manual-validation finding then showed heading
alignment alone is not sufficient either: a robot can reach its target
heading while one corner (typically a rear one, since rotating never
translates the center) is still off the table, and `Turning`'s only
action is an in-place turn - which cannot fix a purely positional
problem, so the robot could spin in place indefinitely with the heading
already correct. The fix: a new `AdvancingInward` state drives straight
forward (the one thing pure rotation cannot do) whenever heading is
already safe but `areAllCornersSafelyInsideTable()` (all four corners
inside the table by a `kRecoverySupportMargin` of `0.15F`, stricter than
the actual physical edge) is not yet satisfied - re-entering `Turning` if
heading drifts outside tolerance while advancing. Both fixes are
geometric completion conditions, not timers, so they are correct by
construction for every approach angle and every edge with no per-scenario
tuning. See `docs/technical-decisions.md` (table-edge recovery bugfixes)
for the full root-cause analyses and fixes. Authority is extended to a
fourth, now HIGHEST-priority level:

```text
Safety  >  Manual  >  AutonomousAvoidance  >  Fsm
```

**Safety overrides even manual driving** - `VirtualRobotHardware` gained
`setSafetyWheelSpeeds()`/`clearSafetyWheelOverride()`/
`safetyOverrideActive()`, the same override shape as the manual/
autonomous overrides but ranked above all of them. Holding `UP` in manual
mode right up to the table edge is safely overridden the instant a cliff
sensor trips; the manual request is never lost - it resumes automatically
once the recovery completes, exactly like avoidance resuming after a
manual override ends. `RobotState` can legitimately still read `Moving`
the entire time (`RobotStateMachine`/`RobotController`/`HardwareEventSource`/
`CompositePollingEventSource` are completely unmodified by this phase) -
this is the same command-vs-effective-actuator distinction Phase 13Q/13R
already established for `DriveAuthority`. A final table-support fail-safe
guard inside `VirtualRobotHardware::update()` backstops an unusually large
frame time tunneling past the edge before the recovery controller gets a
chance to react - but it only rejects a proposed position when **all
four** corners are off the table (a genuine full-footprint fall), not
merely one overhanging corner, since a partial overhang is the normal,
expected state while the recovery controller is actively backing away.
The HUD gained `Cliff FL/FR/RL/RR: SAFE/EDGE`, `Edge safety: ACTIVE/
INACTIVE`, and `Edge recovery state: ...`; `Drive authority` now also
shows `SAFETY`. `Renderer3D` draws the table as a simple raised
rectangular platform - never a vertical wall - so the ground/grid remains
visible beyond its edges as open space. See `docs/technical-decisions.md`
(Phase 13S) for the full state-machine transition table, the exact
corner-sensor geometry, and the known limitation around a simultaneous
front-and-rear-edge corner case.

**Manual validation also found a second, independent defect: obstacle
perception was body-width-blind.** `VirtualRobotHardware::obstacleDetected()`
- the one signal `HardwareEventSource` polls to produce `ObstacleDetected`/
`ObstacleCleared` - was backed by exactly one thing: `VirtualDistanceSensor`'s
single center-line ray. A solid obstacle offset enough to miss that one
ray, but still within the robot's actual body-width forward path, was
invisible to perception - the robot would keep receiving `MoveForward`
until `RobotCollision` (a last-resort pose guard, never meant to be the
primary obstacle signal) finally rejected a pose at the last moment. The
fix widens perception in two complementary ways: `VirtualObstacleSensorArray`
(new, raylib-free) casts three parallel rays - `FrontLeft`/`FrontCenter`/
`FrontRight`, offset by `(RobotDimensions::kBodyWidth / 2) - kLateralInset`
either side of center (`FrontCenter` geometrically identical to the
existing single ray) - and a new `ForwardClearanceProbe::isForwardCorridorClearWithinDistance()`
overload lets `VirtualRobotHardware` reuse the SAME swept-body corridor
`ReactiveObstacleAvoidance` already uses for its release condition, as a
secondary hazard signal closing the gaps three discrete rays still leave,
with its own independently-derived, deliberately SHORTER lookahead
(`kBodyCorridorHazardLookahead`, 0.82F) so a centered obstacle still
triggers at exactly the pre-existing distance - reusing avoidance's own
1.4F lookahead for detection too would have changed existing centered-
obstacle detection distances and risked event oscillation (see
`docs/technical-decisions.md`, manual-validation bugfix, for the full
derivation and the worked regression-test example that first caught
this). `HardwareEventSource` is completely unmodified - it still observes
only the one aggregate `obstacleDetected()` edge, exactly as before. Three
responsibilities remain distinct: range rays (perception/distance
telemetry), the body corridor (swept-body forward safety, now also a
detection-hazard input, not only an avoidance-release one), and
`RobotCollision` (the unconditional final penetration guard, which should
never be the FIRST signal that an obstacle exists - the whole point of
this fix). The HUD replaced its old single `Obstacle distance`/`Obstacle
detected` pair with `Obstacle L/C/R: CLEAR` or a detected distance, plus
`Body corridor: CLEAR/BLOCKED`; `Renderer3D` now draws all three
perception rays.

**As of Phase 13T, Return Home is real geometric navigation, driven
through the same event/FSM/authority architecture as everything else.**
Pressing `R` (edge-triggered) feeds a `ReturnHomeRequested` Event through
a new `ReturnHomeRequestSource` - never a direct `RobotStateMachine`
mutation - consumed via one small, explicitly justified FSM addition:
`Moving + ReturnHomeRequested -> ReturningHome` (the pre-existing
`BatteryCritical` trigger to the same state, and the pre-existing
`ReturningHome + ObstacleDetected -> WaitingForObstacleClear` interruption/
resume behavior with `resumeState_`, were both audited and found to
already exist unmodified - see `docs/technical-decisions.md`, Phase 13T,
for the full audit). A new `HomeNavigator` (raylib-free, headless) is V1
reactive point-to-point navigation - deliberately NOT path planning (no
A*, Dijkstra, occupancy grid, SLAM, waypoint graph, or docking vision): it
continuously recomputes the straight-line direction to
`VirtualWorld::basePlatform()`'s center from the robot's current pose
every frame (reusing `VisualMath.hpp`'s existing heading utilities, never
duplicating base coordinates) and cycles through
`Inactive -> Aligning -> Driving -> Arrived`, with two-threshold heading
hysteresis (8.0F to start driving, 15.0F to fall back to aligning)
preventing rapid oscillation, and a `0.40F` arrival radius (never exact
coordinate equality) sized against both the base platform's own footprint
and the robot's own collision radius. `HomeNavigator` is enabled purely by
whether `VirtualRobotHardware::currentCommand()` currently reads
`ReturnToBase` - it never independently decides a mission should return
home - which naturally handles every interruption case (obstacle, safety,
manual) with no extra bookkeeping: it resets to `Inactive` the instant an
obstacle stops the hardware, and resumes with a freshly recomputed target
the instant `ReturningHome` resumes, while staying logically armed
underneath a manual or safety interruption (which never touch
`currentCommand()`) so it resumes automatically, unchanged, the moment
that interruption ends. Authority is extended to a fifth tier:

```text
Safety  >  Manual  >  AutonomousAvoidance  >  Navigation  >  Fsm
```

`VirtualRobotHardware` gained `setNavigationWheelSpeeds()`/
`clearNavigationWheelOverride()`/`navigationOverrideActive()`, the same
override shape as every other tier, ranked just above the plain `Fsm`
command. Arrival is reported back into the FSM the same way every other
Event enters it: a new `HomeArrivalEventSource`
(`IPollingEventSource`) observes `HomeNavigator`'s own `Arrived` state,
edge-triggered exactly like `HardwareEventSource`, and emits `HomeReached`
- never a direct `stateMachine.handleEvent(HomeReached)` call - consumed
through the `ReturningHome + HomeReached` transition (see the manual-
validation bugfix paragraph immediately below for exactly where it leads).
Because `CompositePollingEventSource` only ever combines two
sources, the four effective sources this phase needs (command-priority:
`DemoCommandSource` + `ReturnHomeRequestSource`; hardware-priority:
`HardwareEventSource` + `HomeArrivalEventSource`) are combined by NESTING
two instances of the unmodified class, never by rewriting it - this also
guarantees a safety/obstacle event occurring the same frame `HomeReached`
becomes ready can never be silently dropped. A same-frame arrival can
naturally take one extra frame to be consumed (`HomeNavigator::update()`
runs after `runtime.step()` each frame, matching the existing
`hardware.update()` ordering) - an accepted, deterministic one-frame
latency, not a bug. The HUD gained `Home nav: .../Home distance: .../Home
target heading: .../Home heading error: ...` (Ayrıntılı/detailed mode) or,
in Sade/simple mode, an `Eve uzaklık: ...` line shown only while the
current task is actually Return Home, plus an optional cyan target-
direction guide line while actively steering; `Drive authority`/`Kontrol`
now also shows `NAVIGATION`/`Eve Dönüş`.

**Manual-validation bugfix: `R` can be pressed repeatedly, indefinitely.**
Human testing found that after one successful user-requested Return Home,
a second `R` press silently did nothing - `ReturningHome + HomeReached`
always led to `Aborted`, a terminal state with no outgoing transitions at
all, so `Aborted + ReturnHomeRequested` was rejected forever. The fix:
`RobotStateMachine` now tracks *why* it entered `ReturningHome`
(`ReturnHomeReason::MissionAbort` for the automatic `BatteryCritical`
trigger, unchanged; `ReturnHomeReason::UserRequest` for the explicit `R`
command) and only `MissionAbort` still leads to `Aborted` - a `UserRequest`
arrival instead leads to `Ready`, the same reusable "loaded, idling, can
accept a new mission or another Return Home" state `ScenarioLoaded`
already produces, which now also accepts `ReturnHomeRequested` (so a
second, third, or Nth `R` press all work identically). Driving away under
`M` (manual) in between never touches `RobotState` at all - manual
remains authority-only, exactly as every prior phase established - so the
FSM is still sitting in `Ready` the whole time, ready to accept the next
`R`. See `docs/technical-decisions.md` (Phase 13T) for the full audit,
the nested-composite/latency/arrival-radius rationale, and the
closed-loop test coverage (obstacle-during-Return-Home, table-edge-
during-Return-Home, manual-interruption-during-Return-Home, and repeated
user-requested Return Home after a manual interruption).

### Mission Control (Phase 13U)

`RobotSimulator3D` no longer auto-starts - Phase 13N through Phase 13T's
`DemoCommandSource` (which delivered `ScenarioLoaded` then `StartMission`
automatically at startup) is not used by the interactive executable
anymore (the class itself is untouched and still used by tests/examples
that want a fixed automatic sequence). Mission assignment is fully
explicit through a new **Mission Control** panel and its single event
source, `MissionControlEventSource`:

On-screen (final UI/HUD polish, Turkish by default) the panel shows this
same table's `Command` column as `Gezinmeyi Başlat`/`Eve Dön`/`Görevi
Durdur`/`Eve Dön` - presentation text only, computed in `main3d.cpp` from
these exact same `MissionControlEventSource` calls/`Event` values, never a
second implementation:

| Key | Command | Turkish label shown | Event(s) |
|---|---|---|---|
| `1` | Start Roam | `Gezinmeyi Başlat` | `ScenarioLoaded` + `StartMission` (from `Idle`) or just `StartMission` (from `Ready`) - never both forced through in one frame; a no-op if already `Moving`/`ReturningHome` |
| `2` | Return Home | `Eve Dön` | `ReturnHomeRequested` |
| `R` | Return Home (alias) | `Eve Dön` | `ReturnHomeRequested` - the exact same call as `2`, never a second implementation |
| `3` | Stop Task | `Görevi Durdur` | `StopTaskRequested` (new `EventType` - audited first; `MissionCompleted` implies success, `EmergencyStop` is a physical fault, `Reset` is reserved for clearing `EmergencyStopped`/`Error`, so none fit a normal task cancellation) |

`StopTaskRequested` is accepted from `Moving`, `ReturningHome`, and
`WaitingForObstacleClear` (an obstacle-paused Roam or Return Home), always
landing in `Ready` - reusable immediately: `1`, `2`, or `R` all work again
right afterward. Cancelling a Return Home clears
`RobotStateMachine::returnHomeReason()` back to `None` (otherwise a stale
reason would leak into the `Ready` state's telemetry). **Stopping never
defeats physical safety**: if `3` is pressed while `TableEdgeSafetyController`
is actively recovering from a table edge, `DriveAuthority` stays `SAFETY`
until recovery genuinely completes - the FSM moves to `Ready` immediately,
but the wheels do not physically stop until Safety itself releases.
Likewise, an active `ReactiveObstacleAvoidance` turn is not forcibly
cancelled - the FSM no longer requests it (its trigger condition already
requires `WaitingForObstacleClear`), but the turn keeps rotating under its
own existing release condition (the forward body corridor genuinely
clearing) until it lets go on its own.

**Mission Control's `Task` display** (`NONE`/`ROAM`/`RETURN HOME`) is
derived fresh every frame from `RobotStateMachine`'s own public state
(`MissionTask.hpp::deriveMissionTask()`) - `Moving` → `Roam`,
`ReturningHome` → `ReturnHome`, `WaitingForObstacleClear` → whichever task
was interrupted (read from `returnHomeReason()`: `None` means it was Roam,
since Roam never sets a return reason), everything else → `None`. This is
presentation/Home-Zone-activation context only, deliberately not a second
state machine with any authority of its own.

**Home Zone**: while the task is Roam, `HomeZoneMonitor` watches distance
from `VirtualWorld::basePlatform()` with hysteresis - `kHomeZoneExitRadius`
(9.0F) and `kHomeZoneRearmRadius` (6.0F), chosen against this project's
actual demo geometry rather than blindly reused from illustrative numbers
(see `docs/technical-decisions.md`, Phase 13U, for the full derivation).
The instant the robot wanders outside the exit radius, it emits exactly
one `ReturnHomeRequested` through its own `IPollingEventSource::pollEvent()`
- never a direct Navigation call or `RobotState` mutation - triggering the
exact same `Moving + ReturnHomeRequested -> ReturningHome` path a manual
`2`/`R` press would. It re-arms once back within the (smaller) rearm
radius, so a later Roam session can trigger an automatic return again. The
Mission Control panel shows `Home zone: INSIDE`/`OUTSIDE` and the live
`Base distance`.

**Event-source priority** (highest first): explicit Mission Control
commands, then hardware obstacle/sensor events, then `HomeReached`
arrival, then the automatic Home Zone request - composed via the same
nesting technique Phase 13T already established (`CompositePollingEventSource`
only ever combines two sources at a time, unmodified). Explicit user
commands are never starved by an automatic trigger; safety-relevant
hardware perception is never lost underneath a same-frame `HomeReached`/
Home-Zone readiness; `HomeReached` completing an already-active Return
Home outranks a brand new automatic request.

**Roaming adds no new drive-authority tier.** `Moving` still just means
the plain FSM `MoveForward` command - `DriveAuthority` remains exactly
`Safety > Manual > AutonomousAvoidance > Navigation > Fsm`, unchanged from
Phase 13T. "Roam" is not a new behavior needing its own priority level; it
is the existing reactive layers (obstacle avoidance, table-edge safety)
already doing their job on top of a plain forward FSM command.

**Still deliberately simple - not full robotics simulation.** The robot's
position is clamped to a generic ~10x10 simulation-coordinate bound (a
purely defensive numeric safety net, never expected to be reached in
practice now that the table-edge safety system keeps the robot on the
much smaller ~12x12 table well before that) so it cannot drift away
indefinitely (`DifferentialDrive` itself is world-bounds- and
obstacle-agnostic - both clamping and collision-checking stay in
`VirtualRobotHardware`); and wheel speeds change instantly with no
acceleration/inertia/friction model. See `docs/technical-decisions.md`
(Phase 13N/13O/13P/13Q/13T/13U) for the full rationale and the
`robot_visual`/`robot_visual_simulation`/raylib dependency boundaries.

**`RobotSimulator` never links raylib, `robot_visual`, or
`robot_visual_simulation`.** The two executables are entirely independent
- the CLI's build, dependencies, and behavior are completely unaffected by
the visual simulator's existence.

## The six final scenarios

All under [`scenarios/`](scenarios/):

| File | Purpose | Expected final `RobotState` | Expected `MissionOutcome` |
|---|---|---|---|
| `normal_mission.json` | Straightforward mission with no interruptions. | `Completed` | `Completed` |
| `obstacle_resume.json` | An obstacle interrupts the mission while `Moving`, then clears, and the mission resumes and completes. | `Completed` | `Completed` |
| `low_battery_return.json` | Battery goes critical mid-mission; the robot returns home instead of completing the task. | `Aborted` | `Aborted` |
| `emergency_stop.json` | An emergency stop is triggered while the robot is active. | `EmergencyStopped` | `EmergencyStopped` |
| `invalid_sensor_data.json` | A sensor reports data the FSM can no longer trust. | `Error` | `Error` |
| `invalid_transition.json` | Demonstrates a rejected transition (see below), followed by a normal mission that still completes. | `Completed` | `Completed` |

**`InvalidSensorData` vs. malformed JSON — these are not the same thing.**
`InvalidSensorData` is one of the ten well-defined `EventType` values
(`INVALID_SENSOR_DATA` in the JSON schema); it is a perfectly valid event
in a perfectly valid scenario file, and its defined FSM reaction is to
transition the robot to `Error`. Malformed JSON — a syntax error, a missing
required field, an unrecognized event-type string — is a completely
different failure category, caught before the simulation ever starts and
reported as a `ScenarioParseError` (exit code `2`), never fed into the
state machine at all.

**`invalid_transition.json` is intentionally valid JSON.** It contains a
well-formed event sequence — `MissionCompleted` is sent first, while the
robot is still `Idle`, which the FSM has no rule for. `RobotStateMachine`
rejects that one transition (state stays `Idle`, the rejection is counted)
and the simulator keeps processing the remaining events, which go on to
complete a normal mission. This demonstrates that a rejected FSM transition
is an ordinary, recoverable simulation outcome — not a crash, not an
exception, and not an application failure (the app's exit code is still
`0`).

## Architecture

```text
Scenario JSON
     |
     v
JsonScenarioSource   (implements IEventSource)
     |
     v
  Simulator
   /  |  \
  v   v   v
FSM Logger RobotController
(RobotStateMachine) (StreamSimulationLogger)   |
 |                                             v
 v                                       IRobotHardware
SimulationResult                       (SimulatedRobotHardware)
 |
 v
SimulationReport -> StreamReportWriter
```

`Simulator::run()` pulls one `Event` at a time from the `IEventSource`,
feeds it to `RobotStateMachine::processEvent()`, and (if a logger was
supplied) records the event and its outcome. When a `RobotController` is
attached, `Simulator` also synchronizes hardware to the FSM's starting
state and, after every *accepted* transition, hands the resulting
`RobotState` to `controller->applyState()` - `RobotController` alone
decides the `RobotState -> IRobotHardware` command mapping; `Simulator`
never calls hardware directly, and a rejected transition never reaches the
controller. The CLI (`Application::runSimulation`) wires a
`SimulatedRobotHardware` and `RobotController` in by default; see
[`docs/technical-decisions.md`](docs/technical-decisions.md) for the full
Phase 13A-13D hardware-abstraction rationale. The accumulated
`SimulationResult` is converted to a human-facing `SimulationReport` and
written out by a `StreamReportWriter`.

### Hardware transport boundary (Phase 13K/13L)

`RobotController`/`HardwareEventSource` only ever depend on `IRobotHardware`
- they cannot tell, and do not need to know, which implementation sits
behind it. Alongside the simulated path used everywhere in this document,
there is now a second, parallel `IRobotHardware` implementation that
prepares for a future physical robot without adding any platform-specific
code yet:

```text
Simulated path (used everywhere today):

RobotController / HardwareEventSource
              |
              v
        IRobotHardware
              |
              v
     SimulatedRobotHardware


Real path under construction (implementation boundary only - not wired
into the CLI):

RobotController / HardwareEventSource
              |
              v
        IRobotHardware
              |
              v
      RealRobotHardware
              |
              v
       IRobotTransport
              |
              v
     SerialRobotTransport
              |
              v
         ISerialPort
              |
              v
   (not implemented yet) future platform serial
   implementation (Windows COM port / Linux /dev/tty*)
```

`RealRobotHardware` speaks a small, deliberately tiny text protocol over
`IRobotTransport` (documented centrally in
[`include/robot/RobotProtocol.hpp`](include/robot/RobotProtocol.hpp)):
`MOVE_FORWARD`/`STOP`/`RETURN_TO_BASE` commands, and
`GET_BATTERY`/`GET_OBSTACLE`/`GET_ESTOP` queries answered with
`BATTERY <0-100>`/`OBSTACLE <0|1>`/`ESTOP <0|1>` responses. Response
parsing is strict - a wrong prefix, a missing or extra token, a
non-numeric or out-of-range value, or an empty response all throw
`RobotTransportError` rather than silently substituting a fake sensor
value. This protocol is a **logical message**, not serial bytes - it never
mentions a line terminator.

**`SerialRobotTransport` adds line framing underneath `IRobotTransport`,
nothing else.** It implements `IRobotTransport` over a line-oriented
`ISerialPort`, and its entire responsibility is appending `'\n'` to
whatever command/request string it is given (`GET_BATTERY` becomes the
serial write `GET_BATTERY\n`) and, for queries, returning
`ISerialPort::readLine()`'s result completely unchanged - not trimmed, not
validated. It has no knowledge of `BATTERY`/`OBSTACLE`/`ESTOP` response
formats; that parsing remains `RealRobotHardware`'s job one layer up.
`ISerialPort::readLine()`'s contract is to return a line **with its line
terminator (`'\n'` or `'\r\n'`) already removed** - `SerialRobotTransport`
does not strip anything itself, and a future concrete `ISerialPort` owns
that removal.

**No serial-port, USB, network, or other platform-specific transport is
implemented yet.** `ISerialPort` is tested only through a deterministic,
test-only `RecordingSerialPort` fake that lives in
`tests/SerialRobotTransportTests.cpp` (not shipped as a production type,
matching `RealRobotHardwareTests.cpp`'s own `RecordingRobotTransport`
convention), and `RobotSimulator`/`robot_app` continue to use only
`SimulatedRobotHardware`, unchanged - no `--real`/`--serial`/`--port` CLI
flag exists. See `docs/technical-decisions.md` for the full rationale.

### CMake / library structure

Matches [`CMakeLists.txt`](CMakeLists.txt) exactly:

| Target | Responsibility |
|---|---|
| `robot_domain` | Header-only `INTERFACE` target exposing the shared vocabulary types (`Event`, `EventType`, `RobotState`, `IEventSource`). No implementation of its own. |
| `robot_core` | `RobotStateMachine` (the FSM) and `Simulator` (orchestration). Has no JSON dependency. `RobotStateMachine` has no hardware dependency either — it never includes `IRobotHardware`/`RobotController`. |
| `robot_hardware` | `IRobotHardware` abstraction and `SimulatedRobotHardware`, a deterministic in-memory implementation. Depends only on `robot_domain`. |
| `robot_transport` | `IRobotTransport` abstraction and `RobotProtocol.hpp`'s wire-protocol constants (Phase 13K). Header-only `INTERFACE` target, matching `robot_domain`'s own pattern - `IRobotTransport` is pure virtual and `RobotProtocol.hpp` is only `constexpr` constants, so neither has a `.cpp`. Depends only on `robot_domain` for the include directory; knows no `RobotState`/`Event`/`RobotController`/`RobotRuntime` type. |
| `robot_real_hardware` | `RealRobotHardware` (Phase 13K), an `IRobotHardware` implementation that drives `IRobotTransport`'s tiny text protocol instead of `SimulatedRobotHardware`'s in-memory state. Depends on `robot_hardware` and `robot_transport`. Not linked into `robot_app`/`RobotSimulator` - no CLI wiring yet, and no concrete transport (serial or otherwise) exists yet. |
| `robot_serial_transport` | `ISerialPort` abstraction and `SerialRobotTransport` (Phase 13L), the line-oriented `'\n'`-framing `IRobotTransport` implementation sitting under `RealRobotHardware`. `ISerialPort` lives in this target's public headers rather than a separate target - it has exactly one consumer. Depends only on `robot_transport`. No OS-level `ISerialPort` implementation exists yet, and this target is not linked into `robot_app`/`RobotSimulator` either. |
| `robot_controller` | `RobotController`, which maps a resulting `RobotState` to one `IRobotHardware` actuator command. Depends on `robot_domain` and `robot_hardware`. |
| `robot_hardware_events` | `HardwareEventSource`, an `IEventSource`/`IPollingEventSource` implementation that turns `IRobotHardware` sensor reads into `Event`s. Depends on `robot_domain` and `robot_hardware`. |
| `robot_runtime` | `RobotRuntime`, the live one-cycle-per-`step()` counterpart to `Simulator` (see `docs/technical-decisions.md`), wired into the CLI's `--live` mode via `robot_app`. Depends on `robot_domain`, `robot_core`, and `robot_controller`. |
| `robot_runtime_runner` | `LiveRuntimeRunner`, a deterministic finite scheduler that calls `RobotRuntime::step()` an exact number of times and tallies the results. No timing policy yet. Depends only on `robot_runtime`. |
| `robot_sensor_script` | `SensorScript`, the deterministic sensor-injection script parser (Phase 13I). Parses `<cycle> <sensor> <value>` text files into sorted `SensorScriptEntry` values. Depends only on `robot_domain` (for its include directory - it uses no domain types). |
| `robot_scripted_runtime` | `ScriptedLiveRuntimeRunner` (Phase 13I/13J), a simulator-only adapter that, once per cycle, applies `SensorScript` mutations to `SimulatedRobotHardware` and/or advances a `ScriptedCommandEventSource`'s current cycle, immediately before each `RobotRuntime::step()`. Reuses `RuntimeRunSummary` from `robot_runtime_runner`. Depends on `robot_runtime`, `robot_runtime_runner`, `robot_hardware`, `robot_sensor_script`, and `robot_command_events`. |
| `robot_command_script` | `CommandScript`, the deterministic mission-command script parser (Phase 13J). Parses `<cycle> <command>` text files into sorted `CommandScriptEntry` values, each mapped to an existing `EventType`. Depends only on `robot_domain` (for `EventType`/the include directory). |
| `robot_command_events` | `ScriptedCommandEventSource` (Phase 13J), an `IPollingEventSource` that turns a `CommandScript`'s eligible-and-unconsumed entries into `Event`s on an externally-driven cycle schedule (`setCurrentCycle()`). Depends on `robot_domain` and `robot_command_script`. |
| `robot_polling_composite` | `CompositePollingEventSource` (Phase 13J), combines a command `IPollingEventSource` and a sensor `IPollingEventSource` into the one `RobotRuntime` accepts, with command-before-sensor priority. Depends only on `robot_domain`. |
| `robot_scenario` | `JsonScenarioSource` — converts a scenario JSON file into `Event` objects. Depends on `robot_domain` and, privately, on nlohmann/json. |
| `robot_logging` | `StreamSimulationLogger`, the concrete `ISimulationLogger` implementation that writes to any `std::ostream`. |
| `robot_reporting` | `MissionOutcome` mapping and `StreamReportWriter`, which turn a `SimulationResult` into a human-readable report. Depends on `robot_core` for `SimulationResult`. |
| `robot_app` | CLI argument parsing and the `runApplication`/`runSimulation`/`runLiveSimulation` composition logic, kept separate from `main.cpp` so it's directly unit-testable without a subprocess. Constructs the default `SimulatedRobotHardware`/`RobotController` for the CLI's scenario path, the live pipeline (`HardwareEventSource`/`RobotRuntime`/`LiveRuntimeRunner`) for `--live`, `SensorScript`/`ScriptedLiveRuntimeRunner` for `--sensor-script`, and additionally `CommandScript`/`ScriptedCommandEventSource`/`CompositePollingEventSource` for `--command-script`. Depends on all of the above. |
| `RobotSimulator` | The executable — `src/main.cpp` is a ~10-line composition root that calls into `robot_app`. |
| `robot_visual_world` | `VirtualWorld`/`RobotPose`/`BoxObstacle`/`BasePlatform` (Phase 13M) - plain demo-scene data using this project's own `Vec3`, deliberately raylib-free. `VirtualWorld::setRobotPosition()`/`setRobotHeading()` (Phase 13N) and `setObstaclePosition()`/`setObstacleEnabled()` (Phase 13O) are its only mutation entry points. Depends only on `robot_domain` for the include directory. |
| `robot_visual` | `VisualRobot`/`Renderer3D` (Phase 13M), the raylib-based 3D drawing layer. Depends on `robot_visual_world` and `raylib`. This is the **only** point where this project depends on raylib - the dependency points inward, never the other way, and no robot-core/domain/hardware target links it. `Renderer3D` receives a plain `VisualTelemetry` struct (state/command text plus sensor readings, Phase 13N/13O) rather than depending on any FSM/hardware/sensor type. |
| `robot_visual_simulation` | `VirtualRobotHardware` (Phase 13N) - the `IRobotHardware` implementation that is the visual simulator's FSM-driven actuator/sensor boundary, translating `RobotController` commands into wheel speeds and, once per frame, `VirtualWorld` pose changes via `update()` - plus `VirtualDistanceSensor` (Phase 13O), the raylib-free geometry-based forward distance sensor `obstacleDetected()`/`obstacleDistance()` are backed by, and `DifferentialDrive`/`RobotCollision`/`ManualDriveInput` (Phase 13P) - the raylib-free differential-drive kinematic model `VirtualRobotHardware::update()` delegates movement to, the raylib-free circle-vs-AABB obstacle collision query that same `update()` validates a proposed pose against before committing it, and the pure X/UP/DOWN/LEFT/RIGHT wheel-speed decision function behind `RobotSimulator3D`'s manual drive mode (only caller: `main3d.cpp`), respectively, and `ReactiveObstacleAvoidance` (Phase 13Q; a stateful latch as of Phase 13R) - the raylib-free "what wheel speeds does an avoidance turn use, and is the turn still active" policy behind `RobotSimulator3D`'s reactive obstacle avoidance, with `VirtualRobotHardware::driveAuthority()`/`setAutonomousWheelSpeeds()`/`clearAutonomousWheelOverride()` implementing its fixed Manual > AutonomousAvoidance > Fsm priority, plus `ForwardClearanceProbe` (Phase 13R) - the raylib-free swept-body forward-corridor clearance query (expanded-AABB-vs-segment, reusing `RobotCollision`'s own collision radius) that drives the avoidance latch's release condition, distinct from `VirtualDistanceSensor`'s single-point-ray perception and never a substitute for `RobotCollision`'s own unconditional final penetration guard, plus `VirtualCliffSensor`/`TableEdgeSafetyController` (Phase 13S) - the raylib-free four-corner table-edge detection sensor and the small stateful V1 emergency-recovery policy (`Inactive`/`BackingAway`/`MovingForwardFromRearEdge`/`Turning`) built on it, with `VirtualRobotHardware::driveAuthority()`/`setSafetyWheelSpeeds()`/`clearSafetyWheelOverride()` now implementing the full Safety > Manual > AutonomousAvoidance > Fsm priority (Safety highest), plus a table-support fail-safe guard inside `VirtualRobotHardware::update()` distinct from `RobotCollision`'s solid-obstacle guard - a table edge is never modeled as a solid obstacle, plus `VirtualObstacleSensorArray` (manual-validation bugfix) - the raylib-free three-ray (`FrontLeft`/`FrontCenter`/`FrontRight`) body-width-aware forward obstacle-perception array `VirtualRobotHardware::obstacleDetected()` now ORs together with a second, independently-derived-lookahead `ForwardClearanceProbe` query (`isForwardCorridorClearWithinDistance()`), closing the single-center-ray blind spot a laterally-offset obstacle could otherwise slip through undetected until `RobotCollision` caught it at the last moment, plus `HomeNavigator`/`HomeArrivalEventSource` (Phase 13T) - the raylib-free V1 reactive point-to-point Return Home navigation policy (`Inactive`/`Aligning`/`Driving`/`Arrived`, reusing `VisualMath.hpp`'s heading utilities against `VirtualWorld::basePlatform()`) and the edge-triggered `IPollingEventSource` that turns its own `Arrived` state into `HomeReached`, with `VirtualRobotHardware::driveAuthority()`/`setNavigationWheelSpeeds()`/`clearNavigationWheelOverride()` now implementing the full Safety > Manual > AutonomousAvoidance > Navigation > Fsm priority, plus `MissionControlEventSource`/`HomeZoneMonitor`/`MissionTask` (Phase 13U) - the raylib-free explicit-task-assignment event source behind `1`/`2`/`3`/`R` (turning each into `ScenarioLoaded`/`StartMission`/`ReturnHomeRequested`/`StopTaskRequested`, with Start Roam's own Idle-vs-Ready state-dependent event queueing), the raylib-free hysteresis automatic-Return-Home trigger around `VirtualWorld::basePlatform()` for Roam, and the pure `RobotState`-derived task-status enum (`None`/`Roam`/`ReturnHome`) driving both the Mission Control panel and Home Zone activation - never a second authority over the robot. Depends on `robot_visual_world`, `robot_hardware`, `robot_controller`, `robot_runtime`, and `robot_domain` - the same real FSM/controller/runtime stack the CLI uses. Deliberately has no raylib dependency, and is a sibling of `robot_visual` (neither depends on the other) under `RobotSimulator3D`. |
| `RobotSimulator3D` | The interactive 3D visual simulator executable. Depends on `robot_visual` (rendering), `robot_visual_simulation` (FSM-driven movement/sensing), and (Phase 13O) `robot_hardware_events`/`robot_polling_composite` directly, since `main3d.cpp` constructs a real `HardwareEventSource`/`CompositePollingEventSource` - never links `robot_app`, and `RobotSimulator` never links any of these or raylib. `main3d.cpp` constructs the real `RobotStateMachine`/`RobotController`/`RobotRuntime`, driven by `MissionControlEventSource` (Phase 13U - explicit `1`/`2`/`3`/`R` task assignment; the simulator no longer auto-starts, so `DemoCommandSource` is not used here, though the class itself remains for tests/examples that still want it) and the real `HardwareEventSource` to reach `WaitingForObstacleClear`/back to `Moving` through the same real transition rules. |

Dependencies flow one way only: `RobotStateMachine` never depends on
JSON parsing, logging, or reporting; `JsonScenarioSource` never depends on
reporting or the CLI layer.

## State vs. Event

- **State** — what condition the robot is currently in right now (e.g.
  `Moving`, `WaitingForObstacleClear`). A `RobotState` value is a snapshot,
  not an action.
- **Event** — something that *happened* and may trigger a transition to a
  new state (e.g. `ObstacleDetected`, `BatteryCritical`). An `Event` is an
  input, not a snapshot.

For example:

```text
Moving + ObstacleDetected -> WaitingForObstacleClear
```

`ObstacleDetected` and `BatteryCritical` were deliberately modeled as
**events**, not as robot states, because they describe *occurrences* the
robot must react to, not *conditions* the robot persists in. There is no
meaningful "the robot is currently an ObstacleDetected" — but there is a
meaningful "the robot is currently `WaitingForObstacleClear`" as a direct
*consequence* of that event. Keeping this distinction explicit is what
makes `RobotStateMachine::processEvent(state, event) -> state` a clean,
testable pure function: states model *where the robot is*, events model
*what pushes it there*. See
[`docs/technical-decisions.md`](docs/technical-decisions.md) for more on
this and other design decisions, and
[`docs/state-machine.md`](docs/state-machine.md) for the full transition
diagram.

## Project structure

```text
RobotSimulator/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── include/robot/          # Public headers: domain types, FSM, Simulator,
│                            # JsonScenarioSource, loggers, reporting, CLI
├── src/                     # Implementation (.cpp) for everything above
│   ├── RobotStateMachine.cpp
│   ├── Simulator.cpp
│   ├── JsonScenarioSource.cpp
│   ├── StreamSimulationLogger.cpp
│   ├── StreamReportWriter.cpp
│   ├── Application.cpp     # CLI logic (robot_app)
│   └── main.cpp             # Thin composition root
├── tests/                   # GoogleTest suites (one file per concern)
│   └── fixtures/            # Small JSON fixtures for parser-error tests
├── scenarios/                # The six final deliverable scenario files
└── docs/
    ├── state-machine.md      # Mermaid state diagram + resumeState_ explanation
    ├── technical-decisions.md
    ├── references.md
    └── examples/              # Tracked, real (not hand-written) sample outputs
```

`logs/` and `reports/` are created at runtime and are not part of the
tracked project structure (see [Generated runtime outputs](#generated-runtime-outputs)
above).

## Known Limitations

These are documented, audited **V1 scope limitations** — deliberate
boundaries of what this project set out to build, not delivery blockers or
undiscovered defects. Each is discussed in full (root cause, geometry, and
rationale) in [`docs/technical-decisions.md`](docs/technical-decisions.md).

- **Reactive avoidance/navigation resonance.** `ReactiveObstacleAvoidance`
  and `HomeNavigator` can, for certain obstacle placements relative to the
  straight line to base, repeatedly re-trigger each other — the avoidance
  turn changes heading, `HomeNavigator` immediately re-aims back toward
  base, and the cycle repeats with little or no net forward progress. A
  real fix would need obstacle-aware approach-angle memory in
  `HomeNavigator`, which is out of scope for V1.
- **Reactive navigation only — not global path planning.** Both obstacle
  avoidance and Return Home navigation are purely reactive ("turn until
  clear," "steer straight at the target") — there is no A*, Dijkstra,
  SLAM/mapping, occupancy grid, or waypoint graph anywhere in this project,
  and the robot never attempts to recover its original heading/path once a
  reactive layer releases.
- **Table-edge corner case.** If a front and a rear cliff sensor both
  detect an edge at the same instant (a robot straddling two edges near a
  table corner), `TableEdgeSafetyController`'s recovery choice
  (`BackingAway`) is not guaranteed correct for the rear edge too — a
  narrow, geometrically unusual case that remains unresolved.
- **Discrete-ray obstacle perception.** `VirtualObstacleSensorArray` casts
  three parallel forward rays (not a continuous sensor field), which by
  construction leaves small unswept gaps between rays; a secondary
  body-corridor hazard check (`ForwardClearanceProbe`) closes most of that
  gap but the underlying perception is still discrete, not continuous.
- **Fixed V1 constants.** Detection ranges, safety margins, arrival radii,
  hysteresis thresholds, and similar geometry/timing constants throughout
  the visual simulator are hardcoded, tuned against this project's own demo
  scene — they are not runtime-configurable or auto-tuned for arbitrary
  geometry.
