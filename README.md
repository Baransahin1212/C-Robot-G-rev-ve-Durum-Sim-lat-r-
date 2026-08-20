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

Two dependencies are used and are **not** something you need to install
yourself:

- [GoogleTest](https://github.com/google/googletest) (`v1.15.2`)
- [nlohmann/json](https://github.com/nlohmann/json) (`v3.11.3`)

Both are fetched automatically at configure time via CMake `FetchContent`
(requires network access the first time you configure the project).

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

As of this writing, this discovers and passes **77 automated tests**
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

## 3D visual simulator (Phase 13M/13N/13O/13P)

Alongside `RobotSimulator` (the CLI/FSM simulator documented above, which
is completely unchanged), there is now a second, separate executable:

```text
RobotSimulator3D
```

Run it directly - no arguments:

```powershell
.\build\Debug\RobotSimulator3D.exe
```

It opens a real 1280x720 window titled **"Robot Simulator 3D"** (starting
in borderless fullscreen - press `F11` to toggle windowed) showing a small
hard-coded demo world: a ground plane and grid, a two-wheel robot model
(body, two wheels, a red front heading marker), four box obstacles, and a
base/docking platform, viewed through a perspective camera you can orbit
with the mouse (`TAB` toggles mouse capture, drag to rotate, scroll to
zoom - raylib's built-in `CAMERA_FREE` mode) and an on-screen HUD showing
the robot's live FSM state/command, position, heading, obstacle count,
(Phase 13O) the forward distance sensor's live reading, and (Phase 13P)
the current left/right wheel speeds, drive mode (`FSM` or `MANUAL`), and
whether the last movement was rejected by the obstacle-collision guard
(`Collision: YES`/`NO`). `SPACE` pauses/resumes world movement only
(camera and FSM stepping are unaffected). `O` toggles the one demo
obstacle placed directly in the robot's path, enabled/disabled, to
demonstrate obstacle clearing. `M` toggles manual drive mode (Phase 13P,
see below); while it is on, arrow keys drive the wheels directly
(`UP`/`DOWN` forward/reverse, `LEFT`/`RIGHT` turn) and holding `X` stops
them immediately and takes priority over every other key, for as long as
it is held. Close the window normally to exit.

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
**Autonomous obstacle avoidance/steering is not implemented** - obstacle
stop/resume in normal (non-manual) mode is exactly Phase 13O's behavior,
unchanged.

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

**Still deliberately simple - not full robotics simulation.** The robot's
position is only clamped to the demo world's ~10x10 bounds so it cannot
drift away indefinitely (`DifferentialDrive` itself is world-bounds- and
obstacle-agnostic - both clamping and collision-checking stay in
`VirtualRobotHardware`); wheel speeds change instantly with no
acceleration/inertia/friction model; obstacle removal is manual (`O`) for
this phase, not any kind of automatic avoidance; and `ReturnToBase` is
accepted as a command but not yet implemented as navigation - it currently
behaves like `Stopped` (zero wheel speeds). See
`docs/technical-decisions.md` (Phase 13N/13O/13P) for the full rationale
and the `robot_visual`/`robot_visual_simulation`/raylib dependency
boundaries.

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
| `robot_visual_simulation` | `VirtualRobotHardware` (Phase 13N) - the `IRobotHardware` implementation that is the visual simulator's FSM-driven actuator/sensor boundary, translating `RobotController` commands into wheel speeds and, once per frame, `VirtualWorld` pose changes via `update()` - plus `VirtualDistanceSensor` (Phase 13O), the raylib-free geometry-based forward distance sensor `obstacleDetected()`/`obstacleDistance()` are backed by, and `DifferentialDrive`/`RobotCollision`/`ManualDriveInput` (Phase 13P) - the raylib-free differential-drive kinematic model `VirtualRobotHardware::update()` delegates movement to, the raylib-free circle-vs-AABB obstacle collision query that same `update()` validates a proposed pose against before committing it, and the pure X/UP/DOWN/LEFT/RIGHT wheel-speed decision function behind `RobotSimulator3D`'s manual drive mode (only caller: `main3d.cpp`), respectively. Depends on `robot_visual_world`, `robot_hardware`, `robot_controller`, `robot_runtime`, and `robot_domain` - the same real FSM/controller/runtime stack the CLI uses. Deliberately has no raylib dependency, and is a sibling of `robot_visual` (neither depends on the other) under `RobotSimulator3D`. |
| `RobotSimulator3D` | The interactive 3D visual simulator executable. Depends on `robot_visual` (rendering), `robot_visual_simulation` (FSM-driven movement/sensing), and (Phase 13O) `robot_hardware_events`/`robot_polling_composite` directly, since `main3d.cpp` constructs a real `HardwareEventSource`/`CompositePollingEventSource` - never links `robot_app`, and `RobotSimulator` never links any of these or raylib. `main3d.cpp` constructs the real `RobotStateMachine`/`RobotController`/`RobotRuntime` plus a tiny visual-only `DemoCommandSource` to reach `Moving` through real FSM transitions, and (Phase 13O) the real `HardwareEventSource` to reach `WaitingForObstacleClear`/back to `Moving` through the same real transition rules. |

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
