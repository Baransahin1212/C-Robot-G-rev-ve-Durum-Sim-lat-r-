# Technical Decisions

Short rationale for the notable design decisions made while building
RobotSimulator, written for a reviewer who wants to know *why*, not just
*what*. See [`references.md`](references.md) for sources consulted.

## Modern C++

The codebase targets C++17 and actually uses the following, rather than
claiming a generic "modern C++" label:

- **Classes with one focused responsibility each** — `RobotStateMachine`
  only decides transitions, `JsonScenarioSource` only parses JSON into
  `Event`s, `Simulator` only orchestrates, `StreamSimulationLogger` only
  formats log lines, `StreamReportWriter` only formats a report. See the
  [architecture section](../README.md#architecture) of the README.
- **`enum class`** for `RobotState`, `EventType`, `TransitionResult`,
  `LogLevel`, `MissionOutcome`, `ArgumentAction` — scoped, strongly typed,
  no implicit conversion to `int`.
- **References for dependency injection** — `Simulator` takes
  `IEventSource&`, `RobotStateMachine&`, and an optional
  `ISimulationLogger*`; `StreamSimulationLogger`/`StreamReportWriter` take
  `std::ostream&`. Everything is a non-owning reference or pointer supplied
  by the caller, never constructed internally.
- **`std::optional`** — `Event::value` (a event may or may not carry a
  numeric payload), `SimulationResult::lastEventTimestampMs`, the
  `std::optional<JsonScenarioSource>` used in `Application.cpp` to defer
  construction until after a `try`/`catch`.
- **`std::filesystem`** — creating the `logs/`/`reports/` output
  directories and building paths in `Application.cpp`.
- **RAII / standard-library resource management** — `std::ifstream`,
  `std::ofstream`, `std::vector`, `std::string`, `std::optional` all manage
  their own resources; there is no manual `new`/`delete` anywhere in the
  codebase.

## FSM implementation choice

Alternatives considered for `RobotStateMachine`:

- **Large procedural logic with scattered `if` statements** — rejected;
  hard to verify all transitions are covered, easy to introduce silent
  gaps.
- **State Pattern (one class per state)** — rejected for this project's
  scale. With 9 states and ~10 event types, a class hierarchy would add
  indirection (virtual dispatch, one file per state) without a matching
  benefit; it also fights against keeping transition rules in one place
  that's easy to audit against a spec.
- **Third-party FSM library** — rejected; the transition table is small
  and stable, and a dependency here would obscure exactly the logic a
  reviewer most needs to read directly.
- **Centralized explicit FSM (`switch` on state, nested `switch` on event)**
  — chosen. Every transition is one `case` in one function
  (`RobotStateMachine::processEvent`), which is easy to read top-to-bottom,
  easy to diff against [`state-machine.md`](state-machine.md), and easy to
  unit test exhaustively (all 21 `RobotStateMachineTest` cases enumerate
  specific `(state, event) -> state` outcomes). For a project whose primary
  goals are readability and testability rather than supporting dozens of
  states, this was the better fit.

## Event-driven design

```text
Event source (IEventSource) -> Event -> Simulator -> RobotStateMachine
```

`Simulator::run()` pulls one `Event` at a time from an `IEventSource` and
hands it to `RobotStateMachine::processEvent()`. The state machine never
reaches out and asks for input; it only reacts to what's handed to it. This
keeps the FSM a pure function of `(current state, event) -> new state` with
no knowledge of where events come from — a scenario file today, potentially
something else tomorrow (see below).

## Interface and abstraction: `IEventSource`

```cpp
class IEventSource {
public:
    virtual ~IEventSource() = default;
    virtual std::optional<Event> nextEvent() = 0;
};
```

`JsonScenarioSource` is the only implementation that exists today. The
interface exists so `Simulator` depends on an abstraction, not a concrete
JSON reader — `Simulator`'s constructor takes `IEventSource&`, and nothing
in `robot_core` includes JSON headers.

**Not implemented**, but explicitly enabled by this interface as future
extensibility: a `RealSensorEventSource` or `NetworkEventSource` could
implement `IEventSource::nextEvent()` by reading from real hardware or a
network socket, and `Simulator`/`RobotStateMachine` would not need to
change at all. This is a design property demonstrated by the interface
boundary, not a shipped feature.

## Sensor/decision separation

`JsonScenarioSource` (in `robot_scenario`) only converts a JSON file into a
sequence of `Event` objects. It does not know what `RobotState::Moving`
means, does not call `RobotStateMachine::processEvent`, and does not decide
what the robot should do — it would be exactly as valid a design if the
events it produced were consumed by something other than a robot
simulator.

Conversely, `RobotStateMachine` (in `robot_core`) has no `#include` of
`nlohmann/json.hpp` and no knowledge that its events might have originated
from a file at all. `robot_core`'s CMake target has no dependency, direct
or transitive, on the JSON library.

This split means the JSON schema can change without touching FSM logic,
and the FSM's transition rules can change without touching the parser —
each has exactly one reason to change.

## JSON vs CSV

JSON was chosen over CSV for the scenario format because:

- **Structured, nested data** — an event is naturally a small object
  (`type`, `timestamp_ms`, optional `value`), not a flat row. Representing
  an optional field in CSV means either a fixed empty column or a
  variable-width row format, both worse than JSON's native `"value"?`.
- **Optional numeric values** — `BatteryCritical` carries a `value`; most
  other events don't. JSON expresses "this key is absent" directly;
  in CSV every row needs the same column count, so the value column would
  have to always exist with some placeholder for "no value" (e.g. empty
  string), and the parser would need to reintroduce whatever null-marker
  convention we invented.
- **Extensibility** — adding a new optional field (a second sensor value,
  a text note, a source identifier) later is a non-breaking, additive JSON
  change. In CSV it would require re-deciding column order/count and
  updating every existing file.

CSV is not impossible for this problem — a fixed, well-specified CSV
schema with a sentinel for "no value" would work — but it was a worse fit
given the schema already needed one optional field and might reasonably
need more, and JSON's tooling (nlohmann/json) made structured validation
straightforward.

## Error handling

Three genuinely different kinds of failure are kept separate on purpose:

1. **`ScenarioParseError`** (thrown by `JsonScenarioSource`) — the input
   file could not be opened, was not valid JSON, or did not match the
   scenario schema (missing/wrong-typed fields, unknown event type). This
   represents *inability to construct a valid scenario at all*, which is
   an exceptional, all-or-nothing failure — the whole file is validated
   eagerly in the constructor before any event is returned.
2. **Application/I/O errors** (`std::runtime_error`, caught in
   `Application.cpp`) — a log or report output file could not be opened
   for writing. Also exceptional: the process cannot do its job without
   writable output.
3. **`TransitionResult::InvalidTransition`** (returned, never thrown, by
   `RobotStateMachine::processEvent`) — an ordinary, expected outcome. A
   scenario routinely sends events the current state doesn't handle (see
   `invalid_transition.json`), and that is not a programming error or a
   reason to unwind the stack — it's exactly the kind of result a caller
   is expected to branch on. Exceptions are reserved for "this run cannot
   proceed"; a rejected transition is "this run continues, with one
   recorded rejection." `Simulator::run()` counts it in
   `SimulationResult::rejectedTransitions` and keeps processing subsequent
   events, and the CLI's exit code stays `0` for a scenario that completes
   with rejected transitions (see the [README](../README.md#exit-codes)).

## Logging

`ISimulationLogger` defines three specific methods —
`logEventReceived`, `logTransitionSucceeded`, `logTransitionRejected` —
rather than one generic `log(string)` call, so message *formatting* stays
inside the concrete logger and `Simulator` never builds log strings itself.
`StreamSimulationLogger` is the one implementation, writing to a
caller-supplied `std::ostream&` — this lets the exact same class log to
`std::cout`, an `std::ofstream` (the real `logs/simulation.log`), or an
`std::ostringstream` (used throughout the test suite so tests never touch
the filesystem). The logger is optional: `Simulator` takes a nullable,
non-owning `ISimulationLogger*` defaulting to `nullptr`, so callers that
don't care about logging (most unit tests) aren't forced to supply one.

## Unit testing

The project uses [GoogleTest](https://github.com/google/googletest)
(fetched via CMake `FetchContent`, pinned to tag `v1.15.2`) and
[CTest](https://cmake.org/cmake/help/latest/manual/ctest.1.html) as the
test runner. As of this writing, `ctest -C Debug -N` discovers **77
automated tests** (verified immediately before writing this document — see
the [README testing section](../README.md#testing) for the exact command).

The suite is organized by what it exercises, not by arbitrary grouping:

- **FSM unit tests** (`RobotStateMachineTests.cpp`) — every transition in
  `state-machine.md`, plus invalid-transition and terminal-state rejection.
- **Simulator tests** (`SimulatorTests.cpp`) — use a hand-written
  `FakeEventSource` implementing `IEventSource` entirely in memory, so
  `Simulator`'s orchestration logic (counters, logging calls,
  `lastEventTimestampMs`) is tested independently of JSON parsing.
- **Logging tests** (`StreamSimulationLoggerTests.cpp`) and **reporting
  tests** (`SimulationReportTests.cpp`) — verify formatted output via
  `std::ostringstream`.
- **JSON parser tests** (`JsonScenarioSourceTests.cpp`) — valid parsing
  plus every documented validation failure, using small fixture files
  under `tests/fixtures/`.
- **Final scenario integration tests** (`ScenarioIntegrationTests.cpp`) —
  run the real `scenarios/*.json` files through the actual
  `JsonScenarioSource -> Simulator -> RobotStateMachine` chain.
- **Application/CLI tests** (`ApplicationTests.cpp`) — drive
  `robot::app::runApplication()` directly with in-memory streams and a
  build-tree-local output directory, so CLI behavior (help, argument
  errors, exit codes) is tested without spawning a subprocess or touching
  the real `logs/`/`reports/` directories.

## Hardware abstraction: `IRobotHardware`

```cpp
class IRobotHardware {
public:
    virtual ~IRobotHardware() = default;
    virtual int batteryLevelPercent() const = 0;
    virtual bool obstacleDetected() const = 0;
    virtual bool emergencyStopPressed() const = 0;
    virtual void moveForward() = 0;
    virtual void stop() = 0;
    virtual void returnToBase() = 0;
};
```

`IRobotHardware` isolates the rest of the application from any concrete
hardware, the same way `IEventSource` isolates `Simulator` from JSON.
`SimulatedRobotHardware` (in `robot_hardware`) is the one implementation
today: deterministic, in-memory, mutated only through explicit setters, so
tests can drive exact sensor states without sleeps, threads, or real I/O.

`RobotStateMachine` remains entirely hardware-independent: it does not
include `IRobotHardware.hpp`, does not hold a reference to it, and
`robot_core` has no dependency, direct or transitive, on `robot_hardware`.
This phase only introduces the abstraction and its simulated
implementation — a later phase will connect FSM states/events to hardware
commands through a separate orchestration/controller layer, keeping
`processEvent`'s job limited to `(state, event) -> state`.

## Hardware orchestration: `RobotController`

```text
Event -> RobotStateMachine -> RobotState -> RobotController -> IRobotHardware
```

`RobotController` is the only place FSM states and hardware commands meet.
It takes an `IRobotHardware&` by constructor injection (non-owning, no
dynamic allocation, matching the reference-injection pattern used
throughout - see [Modern C++](#modern-c) above) and maps a `RobotState` to
exactly one actuator command: `Moving` -> `moveForward()`, `ReturningHome`
-> `returnToBase()`, every other state -> `stop()`. The mapping `switch` has
no `default` case, so an unhandled `RobotState` value is a compiler warning
waiting to happen rather than a silent no-op.

`RobotStateMachine.hpp`/`.cpp` still have no knowledge of `IRobotHardware`
or `RobotController` - neither file includes either header. (As of Phase
13C, `robot_core` as a *target* does link `robot_controller`, because
`Simulator.hpp` optionally accepts a `RobotController*` - see below. That
dependency is `Simulator`'s, not `RobotStateMachine`'s.) `RobotController`
is equally one-way: it only reads a `RobotState` it is handed, and does not
decide transitions, parse JSON, log, or report.

`RobotController` is deliberately not wired into `Simulator` or
`Application` yet - it is built and unit-tested in isolation against
`SimulatedRobotHardware`. Connecting it to a real simulation run is future
work.

## Simulator/RobotController integration (Phase 13C)

`Simulator` optionally accepts a fourth, non-owning `RobotController*`
(default `nullptr`), following the exact pattern already established by
`ISimulationLogger*`. When a controller is supplied:

- `Simulator::run()` calls `controller->applyState(stateMachine.currentState())`
  once before processing any events, synchronizing hardware to the FSM's
  starting state (normally `Idle` -> `stop()`).
- After each event, hardware is only updated when
  `RobotStateMachine::processEvent()` returns `TransitionResult::Success` -
  `Simulator` calls `controller->applyState(stateMachine.currentState())`
  with the resulting state. A rejected transition never reaches the
  controller, so hardware state always reflects a state the FSM actually
  entered, never one it merely attempted.

`Simulator` never calls `IRobotHardware` methods directly - it only ever
calls `RobotController::applyState()`; the `RobotState` -> command mapping
stays exclusively inside `RobotController`. `RobotStateMachine` still has no
include of, or reference to, `IRobotHardware` or `RobotController` -
`Simulator.hpp`/`.cpp` are the only files that changed.

`Application`/CLI wiring (constructing a real `SimulatedRobotHardware` and
`RobotController` for `RobotSimulator`) is deferred to a later phase; this
phase only makes `Simulator` capable of driving a supplied controller.

## Application/CLI hardware wiring (Phase 13D)

`Application` is the hardware composition root. `runSimulation()` keeps its
original five-parameter signature (`scenarioPath`, `logsDir`, `reportsDir`,
`out`, `err`) unchanged for every existing caller, and now delegates to a
new overload that additionally takes `IRobotHardware&`:

```cpp
int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err)
{
    SimulatedRobotHardware hardware;
    return runSimulation(scenarioPath, logsDir, reportsDir, out, err, hardware);
}
```

The default (five-parameter) overload constructs a stack-local
`SimulatedRobotHardware` - the only hardware implementation that makes
sense for a desktop simulator CLI with no hardware selection flag. The
six-parameter overload does the real work: it constructs a
`RobotController` around the supplied `hardware` and passes `&controller`
as `Simulator`'s fourth constructor argument, alongside the existing
`RobotStateMachine` and logger. All four objects (`hardware`, `controller`,
`machine`, `logger`) are stack-local to `runSimulation()`, declared in
dependency order, so lifetimes are trivially safe without any dynamic
allocation.

Callers only ever supply `IRobotHardware&`, never a `RobotController` -
`Application` keeps sole responsibility for that composition step, exactly
as `RobotController` keeps sole responsibility for the `RobotState ->
command` mapping. This is what makes a future `RealRobotHardware`
implementation a drop-in replacement for `SimulatedRobotHardware`: nothing
in `RobotStateMachine`, `Simulator`, or `RobotController` would need to
change, only the object `Application` constructs.

`ApplicationTests.cpp` uses this overload with a test-only
`RecordingRobotHardware` (implementing `IRobotHardware`, never added to
production code) to prove, through the real `runSimulation()` path, that
Application actually wires `RobotController` into `Simulator` end-to-end -
not just that `Simulator`/`RobotController` work correctly in isolation
(already covered by `SimulatorTests.cpp` and `RobotControllerTests.cpp`).

CLI stdout/stderr output is intentionally unchanged in this phase - no
actuator command is printed. Hardware behavior is observable through tests
only; a dedicated hardware telemetry layer, if useful, is future work.

## Hardware sensor event source: `HardwareEventSource` (Phase 13E)

```text
IRobotHardware -> HardwareEventSource -> Event -> Simulator -> RobotStateMachine
```

`HardwareEventSource` (in `robot_hardware_events`) is a second
`IEventSource` implementation, alongside `JsonScenarioSource` - not a
replacement for it. It converts `IRobotHardware` sensor reads
(`batteryLevelPercent`/`obstacleDetected`/`emergencyStopPressed`) into the
same `Event`/`EventType` vocabulary `JsonScenarioSource` already produces:
`EventType::ObstacleDetected`, `EventType::ObstacleCleared`,
`EventType::BatteryCritical`, `EventType::EmergencyStop`. No new
`EventType` was added.

- **Edge-triggered, not level-triggered.** Each `nextEvent()` call compares
  the current sensor snapshot to the previously observed one and emits an
  event only on a *change*. A sensor condition that merely persists (e.g.
  the obstacle stays detected) produces nothing on subsequent calls.
- **No invented recovery events.** The existing vocabulary has
  `ObstacleCleared` (obstacle going true -> false), so that transition is
  emitted. It has no "emergency stop released" or "battery recovered"
  event, so those transitions update internal edge-tracking state (so the
  *next* rising edge is still detected correctly) but emit nothing -
  exactly as instructed: don't invent an `EventType` the FSM doesn't
  define.
- **Startup hazards are surfaced.** The previous-sample booleans default to
  `false`, so an already-active hazard at construction time (e.g. emergency
  stop already pressed) is treated as a rising edge on the first sample and
  reported immediately, rather than silently ignored. If the FSM is not yet
  in a state that accepts that event, `RobotStateMachine`/`Simulator`
  handle it exactly like any other rejected transition - no special-casing
  needed here.
- **Deterministic safety priority + no lost edges.** If more than one
  sensor condition becomes newly active between samples, all of the
  resulting events are queued (a `std::deque<Event>`) in a fixed order -
  emergency stop, then critical battery, then obstacle - and drained one
  per `nextEvent()` call before the next snapshot is taken. None are
  silently dropped.
- **Battery threshold.** No threshold was previously documented anywhere in
  the project, so `HardwareEventSource::kCriticalBatteryPercent = 20` is
  this project's first definition of "critical" battery, kept as a named
  constant rather than a magic number.
- **Timestamps.** `Event::timestampMs` has no default and `IRobotHardware`
  exposes no clock, so `HardwareEventSource` uses a private monotonically
  increasing counter (0, 1, 2, ...), incremented once per emitted event.
  This keeps it wall-clock-free and deterministic for tests.
- **Actuator-free.** `HardwareEventSource` only ever calls the three
  sensor-read methods on `IRobotHardware`. It never calls
  `moveForward()`/`stop()`/`returnToBase()`, and it has no knowledge of
  `RobotController` at all - it produces `Event`s, nothing more.

**Snapshot/exhaustion semantics (the key design decision of this phase).**
`IEventSource::nextEvent()` returning `std::nullopt` means "exhausted" to
`Simulator::run()` - the loop stops for good, it does not retry later.
`HardwareEventSource` embraces this as-is rather than reworking
`IEventSource` into a blocking/streaming abstraction: `nullopt` here means
"nothing new right now," and a finite batch of sensor edges set up before
a `Simulator::run()` call will all be drained before that `nullopt` is
returned. This makes `HardwareEventSource` a **finite, snapshot-driven
adapter** for Phase 13E, useful for deterministic tests and one-shot
sensor-to-FSM runs, not a continuous real-time polling service. Adding
continuous polling (a runtime loop that keeps re-invoking `Simulator`, or
re-sampling between calls) is explicitly deferred to a later phase with its
own design, not bolted on here.

**Not wired into the CLI.** `Application`/`main.cpp` still construct only
`JsonScenarioSource`; `JsonScenarioSource` remains the CLI's only event
source. No `--hardware`/`--live` flag was added. Connecting
`HardwareEventSource` to the CLI is future work, once its snapshot
semantics have been validated in isolation (this phase).

## Live sensor runtime: `IPollingEventSource` / `RobotRuntime` (Phase 13F)

Phase 13E's `HardwareEventSource` exposed a real semantic mismatch:
`IEventSource::nextEvent()` returning `std::nullopt` means "this finite
source is exhausted, stop calling it" - the contract `Simulator::run()`
relies on to know when a scenario is over. But for live sensor input,
"nothing changed this cycle" is not the same claim as "this robot will
never produce another event" - conflating the two would either make
`Simulator` loop forever waiting for a `nullopt` that never legitimately
means "done," or would force every live poll to permanently end the run.
Phase 13F resolves this by introducing a second interface and a second,
parallel orchestrator, instead of overloading `IEventSource`'s meaning:

- **`IEventSource`** (`robot_domain`, unchanged) - finite/exhaustible
  input contract. `nullopt` means exhausted. `JsonScenarioSource` and
  `Simulator` are unchanged from Phase 13A-13E.
- **`IPollingEventSource`** (new, `robot_domain`) - live polling contract.
  `pollEvent()` returning `nullopt` means "nothing available this cycle";
  the source may still produce events later. One method,
  `std::optional<Event> pollEvent()`, mirroring `IEventSource` in shape but
  not in meaning.
- **`HardwareEventSource`** now implements *both* interfaces
  (`class HardwareEventSource : public IEventSource, public
  IPollingEventSource`) over the same underlying edge-triggered queue from
  Phase 13E - unchanged: edge-triggering, the emergency/battery/obstacle
  priority order, the pending-event queue, the 20% critical-battery
  threshold, the monotonic timestamp counter, and actuator-free sensor-only
  reads. `nextEvent()` simply delegates to `pollEvent()` - the returned
  value is identical either way; only the caller's interpretation of
  `nullopt` differs by which interface reference it holds.
- **`Simulator`** (`robot_core`, unchanged) - finite scenario
  orchestration. Still loops `while (nextEvent())`, still has no idea
  `IPollingEventSource` or `RobotRuntime` exist.
- **`RobotRuntime`** (new, `robot_runtime`) - the live, one-cycle-at-a-time
  counterpart to `Simulator`. `step()` polls **at most one** `Event` via
  `IPollingEventSource::pollEvent()` and returns immediately - it never
  loops, sleeps, retries, or blocks. `RuntimeStepResult` reports what
  happened: `NoEvent` (nothing polled this cycle), `TransitionAccepted`
  (the FSM accepted the event and `RobotController::applyState()` was
  called with the resulting state), or `TransitionRejected` (the FSM
  rejected it; the controller is not called, so actuator state never
  changes because of a transition the FSM refused - the same rule
  `Simulator` already follows in Phase 13C).

**One-time synchronization.** On its first call only, `step()` calls
`controller.applyState(stateMachine.currentState())` before polling, so
hardware never starts out of sync with the FSM - exactly like
`Simulator::run()`'s pre-loop sync in Phase 13C, just triggered by the
first `step()` instead of by `run()` being called. Every later `step()`
call skips this - synchronization happens once per `RobotRuntime`
lifetime, not once per cycle.

**No scheduling policy.** `RobotRuntime::step()` is a primitive, not a
runtime. Nothing in this phase calls `step()` in a loop, on a timer, or on
a callback - the caller (a future CLI mode, a test, anything) decides how
often to call it. Adding an actual scheduler is explicitly out of scope
here.

**Dependency shape.** `RobotRuntime` depends on `IPollingEventSource` (the
interface, from `robot_domain`), `RobotStateMachine` (`robot_core`), and
`RobotController` (`robot_controller`) - never on `HardwareEventSource` or
any concrete `IRobotHardware` implementation. It never reads a sensor
method directly and never calls an actuator method directly; both flow
exclusively through the objects it was given, exactly as `Simulator`
already does.

**Not wired into the CLI.** `Application`/`main.cpp` are unchanged - the
CLI still only runs `JsonScenarioSource -> Simulator`. No `--live`/
`--hardware`/`--runtime`/`--poll` flag was added, and no production
scheduling loop exists yet. Connecting `RobotRuntime` to the CLI, and
deciding an actual polling cadence, is future work once this primitive's
semantics have been validated in isolation (this phase).

## Live runtime scheduling: `LiveRuntimeRunner` (Phase 13G)

```text
RobotRuntime::step()  -> one live cycle
LiveRuntimeRunner::runCycles(N) -> exactly N cycles, deterministically
```

`RobotRuntime::step()` (Phase 13F) already represents exactly one live
polling cycle - it processes at most one `Event` and returns a
`RuntimeStepResult`. Phase 13G adds `LiveRuntimeRunner`, a thin scheduler on
top of it: `runCycles(cycleCount)` calls `step()` **exactly** `cycleCount`
times, classifies every `RuntimeStepResult`, and accumulates the counts
into a `RuntimeRunSummary` (`cyclesExecuted`, `noEventCycles`,
`acceptedTransitions`, `rejectedTransitions`).

**`NoEvent` does not stop the runner.** This is the central point of the
phase: for a live polling source, "nothing happened this cycle" is a
routine, expected outcome (see `IPollingEventSource`'s Phase 13F
rationale), not a reason to give up early. `runCycles()` always executes
every requested cycle, whether each one turns out to be `NoEvent`,
`TransitionAccepted`, or `TransitionRejected`.

**Exactly N cycles, no hidden extras.** `runCycles(0)` never calls
`step()` at all and returns an all-zero summary. `runCycles(N)` calls
`step()` exactly `N` times - no retries, no skipped cycles, and no
implicit initialization cycle of its own; `RobotRuntime` already owns its
one-time controller/FSM synchronization internally (Phase 13F), and
`LiveRuntimeRunner` has no knowledge of that behavior at all.

**Dependency boundary.** `LiveRuntimeRunner` (in `robot_runtime_runner`)
depends only on `RobotRuntime`/`RuntimeStepResult` (`robot_runtime`). It
has no knowledge of `IRobotHardware`, `HardwareEventSource`,
`IPollingEventSource`, `RobotController`, `RobotStateMachine`, or
`SimulatedRobotHardware` - it only ever calls `RobotRuntime::step()` and
classifies the result, keeping cycle scheduling independent of robot
internals.

**No `IRuntimeStepper` interface was introduced.** `LiveRuntimeRunner`
takes a concrete `RobotRuntime&`, not an abstraction over it. Tests
substitute a test-only `IPollingEventSource` (a `QueuePollingEventSource`
or similar) underneath a real `RobotRuntime`, which was already
sufficiently simple and deterministic - `RobotRuntime` itself is the
seam Phase 13F built for exactly this purpose, so adding a second
interface whose only real implementor would ever be `RobotRuntime` would
be ceremony, not simplification.

**No timing policy yet.** `runCycles()` contains no `std::chrono`, no
sleeping, no threads, and no timers - it is a pure, synchronous loop over
a fixed cycle count, useful for deterministic tests and bounded live runs.
Deciding an actual polling cadence (a loop, a timer, a callback) and
wiring any of this into the CLI both remain future work.

## Fail-safe / emergency stop

The simulator implements a simplified software model of fail-safe
behavior, entirely at the FSM level:

- `EmergencyStop` transitions to `EmergencyStopped` from every supported
  active state (`Moving`, `WaitingForObstacleClear`, `ReturningHome`).
- `InvalidSensorData` transitions to `Error` from the same three active
  states — representing "sensor input can no longer be trusted, stop
  making decisions on it."
- There is **no automatic transition out of `EmergencyStopped`**. Nothing
  in `RobotStateMachine` moves the robot back to `Idle` on its own after an
  emergency stop.
- `Reset` is the only way out of `EmergencyStopped` or `Error`, and it must
  be sent explicitly — the scenario (or, in a future integration, an
  operator) has to say so.

**This is a software simulation of fail-safe *concepts* for a
demonstration/educational project. It is not a safety-certified industrial
robot controller, has not been through any functional-safety assessment
(e.g. IEC 61508 / ISO 13849), and must not be used as, or represent, a real
safety system.**
