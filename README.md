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

### Hardware transport boundary (Phase 13K)

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


Future real path (implementation boundary only - not wired into the CLI):

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
   (not implemented yet) SerialRobotTransport
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
value.

**No serial-port, USB, network, or other platform-specific transport is
implemented in this phase** - `IRobotTransport` is tested only through a
deterministic, test-only `RecordingRobotTransport` fake that lives in
`tests/RealRobotHardwareTests.cpp` (not shipped as a production type), and
`RobotSimulator`/`robot_app` continue to use only
`SimulatedRobotHardware`, unchanged. See `docs/technical-decisions.md` for
the full rationale.

### CMake / library structure

Matches [`CMakeLists.txt`](CMakeLists.txt) exactly:

| Target | Responsibility |
|---|---|
| `robot_domain` | Header-only `INTERFACE` target exposing the shared vocabulary types (`Event`, `EventType`, `RobotState`, `IEventSource`). No implementation of its own. |
| `robot_core` | `RobotStateMachine` (the FSM) and `Simulator` (orchestration). Has no JSON dependency. `RobotStateMachine` has no hardware dependency either — it never includes `IRobotHardware`/`RobotController`. |
| `robot_hardware` | `IRobotHardware` abstraction and `SimulatedRobotHardware`, a deterministic in-memory implementation. Depends only on `robot_domain`. |
| `robot_transport` | `IRobotTransport` abstraction and `RobotProtocol.hpp`'s wire-protocol constants (Phase 13K). Header-only `INTERFACE` target, matching `robot_domain`'s own pattern - `IRobotTransport` is pure virtual and `RobotProtocol.hpp` is only `constexpr` constants, so neither has a `.cpp`. Depends only on `robot_domain` for the include directory; knows no `RobotState`/`Event`/`RobotController`/`RobotRuntime` type. |
| `robot_real_hardware` | `RealRobotHardware` (Phase 13K), an `IRobotHardware` implementation that drives `IRobotTransport`'s tiny text protocol instead of `SimulatedRobotHardware`'s in-memory state. Depends on `robot_hardware` and `robot_transport`. Not linked into `robot_app`/`RobotSimulator` - no CLI wiring yet, and no concrete transport (serial or otherwise) exists yet. |
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
