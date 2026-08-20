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

## Phase 13H: CLI live mode

Exposes the existing live-runtime pipeline (Phases 13F/13G) through the CLI
as a second, independent orchestration path alongside scenario mode:

```text
Scenario mode:  JsonScenarioSource -> Simulator
Live mode:      SimulatedRobotHardware -> HardwareEventSource ->
                RobotRuntime -> LiveRuntimeRunner
```

**Two syntaxes, one executable.** `RobotSimulator <scenario-file>` is
unchanged. `RobotSimulator --live --cycles <N>` is new. `Application` is
the composition root for both - `main.cpp` still only splits `argv` and
calls `runApplication()`.

**`runLiveSimulation(cycleCount, out, err)` is the new testable entry
point**, parallel to `runSimulation()`. It builds every live-pipeline
object as a stack-local in a single function call - `SimulatedRobotHardware`,
`HardwareEventSource`, `RobotStateMachine`, `RobotController`,
`RobotRuntime`, `LiveRuntimeRunner` - runs `runner.runCycles(cycleCount)`
once, and prints the resulting `RuntimeRunSummary` plus the FSM's final
state (via the existing `toString(RobotState)`). No object is held past
the call; nothing is shared with scenario mode's `runSimulation()`.

**Scenario mode and live mode do not share orchestration.** Live mode
never touches `JsonScenarioSource` or `Simulator`; scenario mode never
touches `HardwareEventSource`, `RobotRuntime`, or `LiveRuntimeRunner`. The
only things they share are `RobotStateMachine`, `RobotController`, and
`SimulatedRobotHardware` - each path constructs its own instances.

**Cycle-count parsing is intentionally small.** A private
`parseCycleCount()` helper in `Application.cpp` uses `std::stoull`, then
enforces three things `std::stoull` alone does not: no leading `-` (which
`std::stoull` would otherwise silently accept and wrap into a huge
unsigned value), full-string consumption (rejects `"1.5"`, `"10abc"`), and
range/exception safety (catches `std::invalid_argument`/`std::out_of_range`
and rejects anything that would not fit in `std::size_t`). This lives in
Application.cpp, not as a new reusable parser type - it is a single,
narrow validation used in exactly one place.

**Boring output is correct output.** `SimulatedRobotHardware` defaults to
battery 100 / no obstacle / no emergency stop, and live mode has no CLI
flag (yet) to change those defaults. `--live --cycles N` therefore always
produces `N` `NoEvent` cycles and a final state of `Idle` - this is the
expected, deterministic result for this phase, not a bug. Sensor
scripting/injection is future work.

**Zero cycles has no hidden step.** `--live --cycles 0` calls
`runner.runCycles(0)`, which (per Phase 13G) never calls `step()` and
returns an all-zero summary. There is no separate initialization cycle;
`RobotRuntime`'s one-time controller/FSM sync (Phase 13F) only happens
inside `step()`, so it never runs at `cycleCount == 0`, and the reported
final state is the FSM's untouched starting state, `Idle`.

**No new dependency direction, only new edges from `robot_app`.**
`robot_app` now additionally links `robot_hardware_events`,
`robot_runtime`, and `robot_runtime_runner`, all `PUBLIC` - matching the
existing (`robot_core`/`robot_scenario`/`robot_logging`/`robot_reporting`)
pattern on that target, none of which appear in `Application.hpp` either.
This is required, not merely a style choice: `robot_app` is a `STATIC`
library, and CMake does not propagate a static library's `PRIVATE` link
dependencies to whatever finally links it - a `PRIVATE` dependency here
would build `robot_app.lib` successfully but leave `RobotSimulator.exe`/
`ApplicationTests.exe` unable to resolve `HardwareEventSource`/
`RobotRuntime`/`LiveRuntimeRunner` symbols at final link time.

**Nothing in the live pipeline itself changed.** `RobotStateMachine`,
`RobotRuntime`, `LiveRuntimeRunner`, `HardwareEventSource`, and
`RobotController` are untouched by this phase - only `Application.hpp`/
`.cpp`, `CMakeLists.txt`, and `ApplicationTests.cpp` changed. This phase is
CLI composition, not a redesign of any already-tested component.

## Phase 13I: scripted sensor injection

Adds an optional, deterministic way to change `SimulatedRobotHardware`'s
sensor values at specific live-mode cycles, without touching any of the
already-tested live-runtime components from Phases 13F-13H:

```text
SensorScript
    v
ScriptedLiveRuntimeRunner
    v
SimulatedRobotHardware
    v
HardwareEventSource
    v
RobotRuntime
    v
RobotStateMachine (FSM)
```

**Two new libraries, no changes to existing ones.** `robot_sensor_script`
(`SensorScript`) parses the script text format; `robot_scripted_runtime`
(`ScriptedLiveRuntimeRunner`) applies its entries to
`SimulatedRobotHardware` around calls to `RobotRuntime::step()`. Neither
`RobotStateMachine`, `RobotRuntime`, `LiveRuntimeRunner`, nor
`HardwareEventSource` has any diff in this phase - confirmed via `git diff
--stat` before merging. `HardwareEventSource` in particular must stay
exactly as it is: it only ever observes `IRobotHardware`, so it behaves
identically whether the sensor changes it sees came from a script or real
hardware - that equivalence is the entire point of layering the script
*underneath* it rather than teaching it about scripts directly.

**Why a separate `ScriptedLiveRuntimeRunner` instead of extending
`LiveRuntimeRunner`.** `LiveRuntimeRunner` is deliberately generic - it
only knows `RobotRuntime`/`RuntimeStepResult` (see Phase 13G) and must stay
that way so it keeps working unmodified for any future non-simulated
`IPollingEventSource`. Simulator-only concerns (`SimulatedRobotHardware`,
`SensorScript`, cycle-indexed injection) belong in a distinct class that
sits *alongside* `LiveRuntimeRunner`, not inside it. `ScriptedLiveRuntimeRunner`
reuses `LiveRuntimeRunner.hpp`'s `RuntimeRunSummary` type purely for a
consistent return shape - it does not construct or delegate to a
`LiveRuntimeRunner` instance, since its own `runCycles()` loop needs to
interleave a mutation step that `LiveRuntimeRunner::runCycles()` has no
hook for.

**Script format.** One entry per non-empty, non-comment line:
`<cycle> <sensor> <value>` (`sensor` is `obstacle`/`battery`/`emergency`;
`value` is `true`/`false` for the two booleans or `0`-`100` for battery).
Blank lines and `#`-prefixed comment lines are ignored. `SensorScript`'s
constructor parses and validates the entire file eagerly - exactly
`JsonScenarioSource`'s philosophy - so a successfully constructed
`SensorScript` is guaranteed to contain only well-formed entries;
`ScriptedLiveRuntimeRunner` can never observe a malformed one. Parse errors
report a 1-based line number (`"sensor script line N: ..."`), matched by a
private `SensorScript.cpp` helper deliberately duplicated from
`Application.cpp`'s `parseCycleCount()` rather than shared - `robot_sensor_script`
must not depend on `robot_app`, and the helper is small enough that the
duplication costs less than the coupling would.

**Cycles are zero-based; mutation happens strictly before its cycle's
step().** `--live --cycles N` executes cycles `0..N-1`.
`ScriptedLiveRuntimeRunner::runCycles()` applies every entry scheduled for
cycle `i` to `SimulatedRobotHardware` and only then calls
`RobotRuntime::step()` for cycle `i` - so a `0 obstacle true` entry is
visible to the very first `step()` call, including the one-time FSM/
controller sync `RobotRuntime` performs internally on that first call
(Phase 13F). An entry scheduled at or beyond `cycleCount` is not an error;
it is simply never applied, since the run never reaches that cycle.

**Same-cycle ordering is file order, via a stable sort.**
`SensorScript`'s entries need not already be sorted by cycle in the file -
the constructor stable-sorts them by cycle after parsing. Because a stable
sort preserves the relative order of equal elements, and entries are
pushed in file order during parsing, entries sharing a cycle retain their
original file order after sorting - `ScriptedLiveRuntimeRunner` then
applies them via the existing `setObstacleDetected()`/
`setBatteryLevelPercent()`/`setEmergencyStopPressed()` setters in exactly
that order, with no separate "apply group" concept needed.

**Live mode starting in `Idle` limits what scripted events can
demonstrate, and that is intentional.** `RobotStateMachine` (unmodified by
this phase) only accepts `ScenarioLoaded` from `Idle` - there is still no
sensor-driven way to reach `Moving` from the CLI, since `HardwareEventSource`
has no lifecycle events of its own (Phase 13E) and this phase does not
invent one. A script run directly via `--sensor-script` from the CLI will
therefore see its obstacle/battery/emergency edges rejected by the FSM,
which is the *correct* documented outcome, not a shortcoming to work
around with an FSM change or a `--start-moving` flag. Richer scripted
scenarios (obstacle-hit-while-Moving, battery-critical-while-Moving,
emergency-stop-while-Moving) are exercised in
`ScriptedLiveRuntimeRunnerTests.cpp` by preparing `RobotStateMachine` into
`Moving` through its real event API first, the same pattern
`RobotRuntimeTests.cpp`/`LiveRuntimeRunnerTests.cpp` already use.

**Script errors use the scenario-file exit code, not a usage error.** A
`--sensor-script` file that cannot be opened or contains a malformed line
throws `SensorScriptParseError`, caught in `Application.cpp` and reported
via `kExitScenarioError` (the same code `runSimulation()` uses for a bad
scenario JSON file) - not `kExitUsageError`, since the CLI syntax itself
was valid. A malformed `--live`/`--sensor-script` invocation (missing
`--cycles`, missing script value, trailing arguments) is still a
`kExitUsageError`, caught entirely during argument parsing before any file
is opened.

**CMake dependency shape.** `robot_sensor_script` depends only on
`robot_domain` (for the include directory; it uses no domain types).
`robot_scripted_runtime` depends on `robot_runtime`, `robot_runtime_runner`,
`robot_hardware`, and `robot_sensor_script` - all `PUBLIC`, since every one
of those types appears in `ScriptedLiveRuntimeRunner.hpp`'s public API.
`robot_app` gained both new libraries `PUBLIC` too, for the same
static-library link-propagation reason documented in the Phase 13H section
above.

## Phase 13J: live mission control / command input

Live mode (Phase 13H) starts in `Idle`, and `HardwareEventSource` only ever
observes `IRobotHardware` sensor state - it has no way to raise
mission-lifecycle events like `ScenarioLoaded` or `StartMission`, because
those are not sensor readings. Scripted sensor injection (Phase 13I) does
not solve this either, and must not: faking `ScenarioLoaded` through a
sensor mutation would make `SimulatedRobotHardware`/`HardwareEventSource`
lie about what they represent. This phase adds a second, independent
scripted input path - for commands, not sensors - that converges on the
same `RobotRuntime`:

```text
CommandScript
    v
ScriptedCommandEventSource  --\
                                >-- CompositePollingEventSource -- RobotRuntime -- RobotStateMachine
SensorScript -> SimulatedRobotHardware -- HardwareEventSource --/
```

**No new `EventType` was introduced.** `RobotStateMachine`/`Event.hpp` are
untouched. `CommandScript` maps its five supported command names 1:1 onto
*existing* `EventType` values that are mission-lifecycle/operator events,
not sensor readings: `scenario_loaded` -> `ScenarioLoaded`,
`start_mission` -> `StartMission`, `mission_completed` ->
`MissionCompleted`, `home_reached` -> `HomeReached`, `reset` -> `Reset`.
The task brief's illustrative names (`abort_mission`, `return_to_base`,
`resume`) do not correspond to any `EventType` this codebase defines, so
they were deliberately not implemented - inventing new `EventType` values
to support them was explicitly out of scope for this phase.
`ObstacleDetected`/`ObstacleCleared`/`BatteryCritical`/`EmergencyStop`/
`InvalidSensorData` remain sensor-only and are rejected as unknown
commands if given to `CommandScript` - the two vocabularies are
disjoint by construction, not by convention.

**`CommandScript` mirrors `SensorScript`/`JsonScenarioSource`'s eager-parse
philosophy** exactly: same `"<cycle> <command>"` line format, same
blank-line/`#`-comment handling, same 1-based `"command script line N: ..."`
error messages, the same independent, duplicated `ParseCycle()` helper (for
the same decoupling reason as `SensorScript.cpp`'s own copy), and the same
stable-sort-by-cycle for deterministic same-cycle file ordering.

**`ScriptedCommandEventSource` implements only `IPollingEventSource`, not
`IEventSource`** - a command script is inherently scheduled against live
runtime cycles via `setCurrentCycle()`, which only makes sense under
`IPollingEventSource`'s "nullopt means nothing new this cycle, ask again
later" contract (Phase 13F), not `IEventSource`'s "nullopt means
exhausted for good". It knows nothing about `IRobotHardware`,
`SimulatedRobotHardware`, or sensor state - only `CommandScriptEntry`
values and the cycle number it was last told about.

**Eligibility semantics: earliest-cycle-reached, pending-until-consumed.**
`ScriptedCommandEventSource::pollEvent()` looks only at its next
unconsumed entry (entries are pre-sorted by cycle): if that entry's cycle
is `<= currentCycle_`, it is eligible and is returned (and consumed)
immediately; otherwise `pollEvent()` returns `nullopt` without touching
anything. Because `currentCycle_` only advances (the caller - see below -
drives it monotonically) and the unconsumed-entry pointer only advances on
actual consumption, an eligible entry that isn't polled this cycle is
still the *next* thing returned on a later cycle - it is never skipped or
replayed. This is the same "scheduled cycle = earliest eligible cycle,
remains pending until consumed" semantics the task brief requires, and it
falls out naturally from reusing the already-sorted `entries()` order
rather than needing a separate "still pending" data structure.

**`CompositePollingEventSource`: command-before-sensor, at most one event
per call.** `RobotRuntime` accepts exactly one `IPollingEventSource`, so
combining a command source and `HardwareEventSource` requires an adapter
implementing that same interface. `pollEvent()` tries the command source
first and returns immediately if it has an event - `hardwareSource_` is
never even polled that call, so `HardwareEventSource`'s own edge-detection
sampling is skipped entirely on a cycle where a command wins (its
"previous sample" comparison state is simply unchanged, ready to detect
the real edge whenever it is finally polled - see the priority example
below). Command priority exists because early-mission commands
(`ScenarioLoaded` at `Idle`, `StartMission` at `Ready`) are the only way
to reach `Moving` at all, and should not be starved by a same-cycle sensor
edge from a state that would just reject it anyway.

**Priority example (worked through the actual FSM table).** Suppose at
cycle 5 a command script schedules `mission_completed` and a sensor script
schedules `battery 15` (both eligible/applied at cycle 5), with the FSM
in `Moving`. `CompositePollingEventSource::pollEvent()` returns the
command event; `RobotStateMachine::processEvent(MissionCompleted)` from
`Moving` transitions to `Completed` (an existing, unmodified rule) -
*not* `ReturningHome`, which is what the sensor's `BatteryCritical` event
would have caused had it been processed instead. The battery edge is not
lost - `HardwareEventSource` simply has not sampled it yet, since
`hardwareSource_.pollEvent()` was never called this cycle - but by the
time it is (cycle 6), the FSM is already in the terminal `Completed`
state, so that edge is correctly rejected rather than accepted, exactly
as the existing FSM table dictates for a terminal state. Verified in
`ScriptedLiveRuntimeRunnerTests.cpp`'s
`CommandAndSensorSameCycleUseDocumentedPriority` test.

**One common cycle-orchestration point, not two.** `RobotRuntime` still
knows nothing about cycle numbers (per Phase 13F's design, unchanged).
`ScriptedLiveRuntimeRunner` (already the sensor-script cycle owner from
Phase 13I) is extended - not duplicated - to also call
`commandSource.setCurrentCycle(cycle)` immediately before `runtime.step()`
in the same loop iteration that applies `SensorScript` mutations, so
"apply sensor mutation" and "make command entries for this cycle
eligible" happen in the same place, in the same order, for the same
cycle, before the single `runtime.step()` call for that cycle. Three
constructors cover the three independent-axis combinations (sensor-only,
unchanged from Phase 13I; sensor+command; command-only) rather than a
single constructor taking optional/nullable parameters, so the original
Phase 13I 3-argument constructor - and every test that already calls it -
needed no changes at all.

**`runLiveSimulation()`'s sensor-only 3-argument overload was replaced by
a general command+sensor overload**, `runLiveSimulation(cycleCount,
commandScriptPath, sensorScriptPath, out, err)`, where an empty path means
"not provided" for that axis - not four overloads for every combination
(sensor-only/command-only/both could not coexist as separate overloads
taking `(std::size_t, const std::string&, ...)` anyway, since C++ cannot
overload on parameter name alone). This is safe because no test calls
`runLiveSimulation()` directly - `ApplicationTests.cpp` only drives
`runApplication()`/`runSimulation()` - so no existing test needed to
change; only `runApplication()`'s dispatch was updated to call the general
overload whenever at least one script path is non-empty, otherwise the
original no-script overload.

**CLI ordering is fixed and validated, not a general parser.**
`--live --cycles <N> [--command-script <file>] [--sensor-script <file>]`
is the only accepted shape beyond plain `--live --cycles <N>` - exactly
one of five token-count/flag-position combinations is valid (3, 5 with
`--sensor-script`, 5 with `--command-script`, or 7 with `--command-script`
then `--sensor-script`); everything else, including the two flags in the
opposite order, falls through to `InvalidLiveArguments`/
`kExitUsageError`. A general option parser was judged unnecessary
ceremony for two optional, order-fixed flags.

**Script errors use the scenario-file exit code, not a usage error** -
same reasoning as Phase 13I's `--sensor-script`. A `--command-script` file
that cannot be opened or contains a malformed line throws
`CommandScriptParseError`, caught in `Application.cpp` and reported via
`kExitScenarioError` with an `"Error loading command script: ..."`
prefix, distinct from `"Error loading sensor script: ..."` so the two
failure sources are never ambiguous in the output.

**CMake dependency shape.** `robot_command_script` depends only on
`robot_domain`. `robot_command_events` depends on `robot_domain` and
`robot_command_script`. `robot_polling_composite` depends only on
`robot_domain` - it is deliberately independent of both
`robot_command_events` and `robot_hardware_events`, since
`CompositePollingEventSource` only ever calls `IPollingEventSource::pollEvent()`
on whatever two sources it is given. `robot_scripted_runtime` gained
`robot_command_events` `PUBLIC` (the type appears in
`ScriptedLiveRuntimeRunner.hpp`'s new constructors). `robot_app` gained
all three new libraries `PUBLIC`, for the same static-library
link-propagation reason documented in the Phase 13H section.

**Nothing in the already-tested live pipeline changed.**
`RobotStateMachine`, `RobotRuntime`, `HardwareEventSource`,
`RobotController`, and `Simulator` have no diff in this phase - confirmed
via `git diff --stat` before merging. Only `ScriptedLiveRuntimeRunner`
changed (extended with new constructors and an `if (commandSource_ !=
nullptr)` branch in its existing loop, not a rewrite), because it was
already this codebase's designated simulator-specific cycle-orchestration
seam from Phase 13I - introducing a second, competing orchestrator instead
would have duplicated the cycle loop the task brief explicitly said to
avoid duplicating.

## Phase 13K: real-hardware boundary / transport abstraction

Prepares for a future physical robot by adding a second `IRobotHardware`
implementation, without adding any platform-specific code:

```text
RobotController / HardwareEventSource
              |
              v
        IRobotHardware  (unchanged since Phase 13A)
             / \
            /   \
           v     v
SimulatedRobotHardware   RealRobotHardware
     (existing)                |
                                v
                          IRobotTransport
                                |
                                v
                (not implemented) SerialRobotTransport
```

**`IRobotHardware` itself needed no change.** `RobotController` and
`HardwareEventSource` already depended on nothing but that interface
(Phase 13A/13E), which is exactly the seam this phase needed - a second
implementation slots in without either of those two classes, or anything
downstream of them, being touched. This is verified directly, not just
assumed: `RealRobotHardwareTests.cpp` drives the real, unmodified
`RobotController` and `HardwareEventSource` against `RealRobotHardware`
and asserts the same outcomes their own test files already assert against
`SimulatedRobotHardware`.

**`IRobotTransport` is a text request/response abstraction, not a
strongly-typed robot API.** The task brief's own suggested shape
(`sendCommand(string)`/`query(string)`) was kept close to verbatim after
considering the alternative: a strongly-typed transport (e.g.
`moveForwardCommand()`/`batteryQuery()`) would require `IRobotTransport`
itself to know the wire protocol's command/query vocabulary, which is
exactly the RealRobotHardware-specific knowledge this interface must stay
ignorant of to remain a generic "move bytes" boundary reusable by any
future concrete transport. `IRobotTransport` knows only that a message is
one request or command string in, and (for queries) one response string
out - never `RobotState`, `Event`, `EventType`, `RobotController`, or
`RobotRuntime`.

**The wire protocol is centralized in `RobotProtocol.hpp`, not scattered
as literals.** Three one-way commands (`MOVE_FORWARD`/`STOP`/
`RETURN_TO_BASE`), three queries (`GET_BATTERY`/`GET_OBSTACLE`/
`GET_ESTOP`), and their three expected response prefixes (`BATTERY`/
`OBSTACLE`/`ESTOP`) are `constexpr std::string_view` constants in one
header, referenced by both `RealRobotHardware.cpp` and
`RealRobotHardwareTests.cpp`. This is a *logical* protocol only - no
framing, checksum, retry, or serialization library (no JSON) - deliberately
minimal, since defining that logical contract, not implementing it over a
real wire, is this phase's scope.

**Response parsing is strict, with no partial acceptance.** A response
must be exactly two whitespace-separated tokens: the expected prefix, then
a value. `BATTERY` alone (missing value), `BATTERY 50 extra` (trailing
tokens), a wrong prefix, or an empty response are all rejected the same
way as a genuinely invalid value (`BATTERY abc`, `BATTERY -1`,
`BATTERY 101`, `OBSTACLE 2`) - every case throws `RobotTransportError`
rather than falling back to a default/fabricated sensor value, matching
this project's existing "bad input fails loudly, eagerly, before it can
propagate" philosophy (`ScenarioParseError`, `SensorScriptParseError`,
`CommandScriptParseError`). The three sensor parsers
(`batteryLevelPercent()`/`obstacleDetected()`/`emergencyStopPressed()`)
share one `ExtractValueToken()` helper for the "exactly two tokens, first
matches prefix" structural check, then apply their own
range/boolean-specific validation - avoiding three near-duplicate
tokenizing implementations without inventing a shared class hierarchy for
what is, underneath, three independent small parsers.

**`const` sensor methods calling a non-`const` `query()` is legal, not
a workaround.** `IRobotHardware`'s sensor methods are `const` (Phase 13A);
`IRobotTransport::query()` is not, since issuing a request is naturally a
side-effecting operation. `RealRobotHardware` stores `transport_` as a
reference member (`IRobotTransport&`), not a value or pointer - C++ does
not propagate `const` through a reference member the way it does through a
value or raw-pointer member, so calling `transport_.query(...)` from
inside a `const` `RealRobotHardware` method is ordinary, standard-conforming
C++, not a `const_cast` or `mutable` escape hatch.

**Test-only transport, not a production fake.** Per the task brief,
`RecordingRobotTransport` lives entirely inside
`tests/RealRobotHardwareTests.cpp` - no `FakeRobotTransport` production
type was added anywhere under `include/`/`src/`. It records every sent
command and every query string, and returns a per-request configured
response (or `""` for an unconfigured request, which doubles as free
coverage of the "empty response" rejection path).

**No CLI wiring, no serial implementation - both deliberately out of
scope.** `Application.cpp`/`Application.hpp` have no diff in this phase;
`RobotSimulator`/`robot_app` continue to construct only
`SimulatedRobotHardware`, exactly as before. No `--real`/`--serial`/
`--port`/`--device` flag exists. No `CreateFile`, COM port, `termios`,
`/dev/tty*`, Boost.Asio, libserialport, or any other platform-specific or
USB/serial API appears anywhere in this phase's changes - `IRobotTransport`
existing as an abstraction is the entire deliverable; an actual
`SerialRobotTransport` (or similar) is explicitly future work.

**CMake dependency shape.** `robot_transport` is header-only
(`IRobotTransport.hpp` is pure virtual; `RobotProtocol.hpp` is only
constants - neither has a `.cpp`), so it is an `INTERFACE` target
depending only on `robot_domain`, matching `robot_domain`'s own pattern
(and, like `robot_domain`, deliberately excluded from the `/W4` `foreach`
loop, since `target_compile_options(... PRIVATE ...)` is not valid on an
`INTERFACE` library that compiles nothing itself). `robot_real_hardware`
depends on `robot_hardware` (`IRobotHardware`) and `robot_transport`
(`IRobotTransport`/`RobotProtocol`), both `PUBLIC` since both types appear
in `RealRobotHardware.hpp`'s public API. Neither target is linked into
`robot_app`.

## Phase 13L: serial transport abstraction

Adds a platform-independent line transport layer underneath
`IRobotTransport`, extending the boundary from Phase 13K one step further
without implementing an actual OS serial port yet:

```text
RobotController / HardwareEventSource
              |
              v
        IRobotHardware        (Phase 13A - unchanged)
              |
              v
      RealRobotHardware        (Phase 13K - unchanged this phase)
              |
              v
       IRobotTransport         (Phase 13K - unchanged this phase)
              |
              v
     SerialRobotTransport      (Phase 13L - new)
              |
              v
         ISerialPort           (Phase 13L - new)
              |
              v
   (not implemented) future Windows COM-port / Linux
   /dev/tty* ISerialPort
```

**Neither `RealRobotHardware` nor `IRobotTransport` needed any change.**
Both already existed as exactly the seam this phase needed:
`RealRobotHardware` only ever calls `IRobotTransport::sendCommand()`/
`query()`, and has no idea whether the concrete `IRobotTransport` behind
that reference is the Phase 13K test fake or `SerialRobotTransport` - so
plugging in a new `IRobotTransport` implementation required touching
neither file. Verified directly: `git diff --stat` shows zero diff for
both in this phase, and `SerialRobotTransportTests.cpp` re-runs the same
kind of battery/obstacle/e-stop/actuator assertions
`RealRobotHardwareTests.cpp` already makes, just with `RealRobotHardware`
now wired to a `SerialRobotTransport` instead of the Phase 13K
`RecordingRobotTransport`.

**Two responsibilities, two classes, one clean split.**
`SerialRobotTransport`'s *entire* job is line framing: append `'\n'` to
whatever command/request string it is handed, and (for queries) return
`ISerialPort::readLine()`'s result unchanged. It never looks at message
content beyond that - it does not know `MOVE_FORWARD` from `GET_BATTERY`
from any other string, and it has no notion of a `BATTERY`/`OBSTACLE`/
`ESTOP` response shape. `RealRobotHardware` keeps 100% of the semantic
parsing responsibility it already had (Phase 13K) - `SerialRobotTransport`
being interposed underneath it changes nothing about what
`ExtractValueToken()`/`ParseBatteryValue()`/`ParseBooleanValue()` do
because those functions never depend on framing at all. This mirrors the
project's existing sensor/command separation philosophy (SensorScript vs.
CommandScript, Phase 13I/13J): each layer owns exactly one concern.

**`ISerialPort::readLine()`'s line-terminator contract is a deliberate,
documented choice, not an accident.** `readLine()` returns a line with any
terminator (`'\n'` or `'\r\n'`) already stripped - `SerialRobotTransport`
performs no trimming of its own, confirmed by
`ResponseWhitespaceIsNotNormalized` (a response of `" BATTERY 78 "` comes
back exactly as `" BATTERY 78 "`, leading/trailing spaces included - only
the terminator is a `readLine()` implementation's concern, not arbitrary
whitespace). This keeps the terminator-stripping logic in exactly one
place - whatever future concrete `ISerialPort` actually parses raw serial
bytes - rather than duplicating "did the far end use `\n` or `\r\n`?"
guesswork into `SerialRobotTransport` too. `RealRobotHardware`'s existing
strict, no-partial-parsing rules (Phase 13K) are what ultimately reject
any payload issue that slips through, exactly as before.

**Errors are not caught, wrapped, or converted at this layer.** If
`ISerialPort::write()` or `readLine()` throws, `SerialRobotTransport` lets
the exception propagate completely unchanged - no new exception type was
introduced for this phase, since there is nothing serial-specific to add
to the message an `ISerialPort` implementation would already provide. An
empty string returned by `readLine()` (rather than a thrown exception) is
likewise passed through unchanged, exactly like Phase 13K's
`RecordingRobotTransport` returning `""` for an unconfigured request -
`RealRobotHardware`'s `RobotTransportError` is still what ultimately
rejects it, one layer up, unmodified.

**`ISerialPort` stays deliberately low-level - no configuration
surface yet.** It exposes exactly `write(const std::string&)` and
`readLine() -> std::string`; no device path, baud rate, parity, data/stop
bits, flow control, or timeout appears anywhere in this interface. Those
are construction-time concerns for a *concrete* `ISerialPort`
implementation (a future `WindowsSerialPort`/`PosixSerialPort` or
similar), not the abstraction itself - exactly as `IRobotTransport` in
Phase 13K deliberately carried no robot-domain vocabulary, `ISerialPort`
here deliberately carries no serial-configuration vocabulary. Introducing
either now, before a concrete implementation exists to justify a specific
shape, would be speculative design.

**Test-only `RecordingSerialPort`, not a production fake.** Per the task
brief and matching Phase 13K's `RecordingRobotTransport` precedent,
`RecordingSerialPort` lives entirely inside
`tests/SerialRobotTransportTests.cpp` - no `FakeSerialPort` production
type was added anywhere under `include/`/`src/`. It records every
`write()` call verbatim (including the appended `'\n'`) and returns a
per-call configured line from a FIFO queue (empty string once exhausted),
plus an interleaved `callOrder` log for the one test
(`QueryPerformsWriteBeforeRead`) that needs to assert relative ordering
rather than just final state.

**The `HardwareEventSource` integration test required the exact real
sensor-read order, not a guess.** `HardwareEventSource::sampleAndEnqueueEdges()`
(unchanged, Phase 13E) calls `emergencyStopPressed()`, then
`batteryLevelPercent()`, then `obstacleDetected()`, in that fixed order,
every time it takes a fresh sample - so each sample corresponds to three
serial reads in `ESTOP`/`BATTERY`/`OBSTACLE` order. `RecordingSerialPort`
is a dumb FIFO with no correlation between what was written and what it
returns, so `SerialRobotTransportTests.cpp`'s end-to-end test queues its
six response lines (two three-line samples) in that exact order - queuing
them in query-issuance order (e.g. battery/obstacle/e-stop, matching the
task brief's illustrative example ordering rather than the real one) would
have silently fed the wrong response to the wrong sensor and produced a
misleading test.

**CMake dependency shape.** `robot_serial_transport` depends only on
`robot_transport` (`IRobotTransport`). `ISerialPort.hpp` lives in this
target's public headers rather than a fourth micro-target, since it has
exactly one consumer (`SerialRobotTransport`) inside this same library -
splitting it out further would be fragmentation without a decoupling
benefit, the same judgment call already made for `IRobotTransport`
co-existing with `RobotProtocol.hpp` inside `robot_transport` in Phase
13K. `robot_serial_transport` is not linked into `robot_app`.

## Phase 13M: 3D visual simulator foundation

Adds a second, completely independent executable - `RobotSimulator3D` - a
real 3D window showing a stationary demo robot/world, built on raylib.
This is purely a visualization foundation: no FSM, runtime, sensor, or
physics integration exists yet.

```text
RobotSimulator3D
      |
      v
  robot_visual            (VisualRobot, Renderer3D - raylib-based drawing)
      |
      v
robot_visual_world         (VirtualWorld - plain scene data, raylib-free)
      |
      v
  robot_domain             (include-directory only)

  robot_visual
      |
      v
    raylib                 (Phase 13M - new)
```

**raylib 6.0, fetched via `FetchContent`, pinned to a tagged release -
never `master`.** Matches this project's existing dependency convention
exactly (nlohmann/json `v3.11.3`, GoogleTest `v1.15.2`): `GIT_REPOSITORY`
+ `GIT_TAG 6.0`, not a floating branch. `BUILD_EXAMPLES`/`BUILD_GAMES` are
forced `OFF` before `FetchContent_MakeAvailable(raylib)` so raylib's own
example/game subdirectories never get configured or built - they are not
something this project needs, and skipping them keeps the dependency
footprint and build time down.

**raylib is isolated to exactly one target boundary, `robot_visual ->
raylib`, and nothing upstream of it.** No `robot_core`/`robot_domain`/
`robot_hardware`/`robot_app`/`RobotSimulator` target links raylib, and no
header under those targets includes `raylib.h` or `rlgl.h` - confirmed by
grepping every non-`visual/` header and source file for `raylib`/`rlgl`
and finding zero matches. This is the same "dependency points inward
only" discipline already applied to `IRobotTransport`/`ISerialPort`
(Phase 13K/13L: transport knows nothing about robot-domain vocabulary) and
nlohmann/json (linked `PRIVATE` into `robot_scenario` only) - a third-party
dependency's blast radius is deliberately confined to the one target that
actually needs it.

**`VirtualWorld` (data) and rendering are two different targets, not just
two different files.** `robot_visual_world` (`VirtualWorld.cpp` only)
depends only on `robot_domain` and defines its own tiny `Vec3` - never
raylib's `Vector3` - so it has *no* dependency on raylib at all, provably:
`VirtualWorldTests` links only `robot_visual_world` and `GTest::gtest_main`,
never `robot_visual` or `raylib`, and still passes. `robot_visual`
(`VisualRobot.cpp`, `Renderer3D.cpp`) depends on both `robot_visual_world`
and `raylib`, and is where the `Vec3 -> Vector3` conversion happens (a
small `toRaylibVector3()` helper in `Renderer3D.cpp`). This mirrors the
task brief's explicit instruction - "VirtualWorld should represent the
world. Renderer3D should draw the world. Do NOT put rendering code inside
world mutation logic." - enforced at the build-system level, not just by
convention: `VirtualWorld.cpp` physically cannot call a raylib drawing
function, because it is never linked against raylib in the first place.

**Primitive geometry, not external model files.** The robot (body,
two `DrawCylinderEx`-based wheels, a small front marker), the box
obstacles, and the base platform are all drawn with raylib's built-in
`DrawCube`/`DrawCubeWires`/`DrawCylinderEx`/`DrawPlane`/`DrawGrid` calls -
no `.obj`/`.gltf`/texture asset loading exists yet. `DrawCylinderEx`
(start/end points) was chosen over the simpler `DrawCylinder` (which
always extrudes along +Y) specifically because a wheel needs to extrude
along the robot's local X axis (its rolling axis) - using `DrawCylinder`
here would draw a wheel standing on its flat face like a can, not lying
against the body like a wheel.

**World coordinate convention: X = horizontal, Y = up, Z = depth; ground
is Y = 0.** `VisualRobot`'s heading rotation is applied around Y via
`rlPushMatrix()`/`rlTranslatef()`/`rlRotatef()`/`rlPopMatrix()` in local
space (heading 0 = facing +Z), so a future phase that starts changing
`headingDegrees` needs no rendering-code changes - the rotation plumbing
already exists, only the value driving it is currently constant.

**Camera: `CAMERA_FREE`, raylib's built-in mode, not a custom camera
framework.** A single `UpdateCamera(&camera_, CAMERA_FREE)` call per frame
gives mouse-drag-to-orbit and scroll-to-zoom for free - enough to inspect
the whole scene from any angle without writing or maintaining any camera
math. Initial position `(8, 8, 8)` looking at the origin with a 45°
`fovy` was chosen specifically so the entire ~10x10-unit demo scene (both
the robot's start position and the base platform in the opposite corner)
is inside the frustum on the very first rendered frame, with no camera
adjustment required to see the whole layout.

**No `InitWindow()`/`CloseWindow()` inside `Renderer3D`.** Window
lifecycle (`InitWindow`, `SetTargetFPS`, the `while (!WindowShouldClose())`
loop, `CloseWindow`) lives entirely in `main3d.cpp`, matching the task
brief's suggested split ("Renderer3D: rendering" / "main3d: application
lifecycle / render loop"). `Renderer3D::renderFrame()` assumes a window is
already open; it only ever calls `BeginDrawing()`/`EndDrawing()` and the
3D/2D drawing calls in between. This keeps `Renderer3D` a pure "draw one
frame given a world" component, independently reasoned-about from
"when/how often does a frame get drawn."

**The `while (!WindowShouldClose())` loop in `main3d.cpp` is not a
violation of the "no infinite loops in robot core/runtime" rule.** It is
the interactive window's own render loop, exactly the shape every raylib
application takes, and it lives entirely inside `RobotSimulator3D` - a
target that `RobotStateMachine`/`RobotRuntime`/`RobotController`/
`Simulator`/`LiveRuntimeRunner` know nothing about and never execute
inside. No sleep/thread/timer of this project's own was added;
`SetTargetFPS(60)` is raylib's own internal frame pacing.

**No FSM/runtime construction in `RobotSimulator3D` yet, and no CLI
wiring for the visual simulator either.** `main3d.cpp` constructs exactly
`VirtualWorld` and `Renderer3D` - no `RobotStateMachine`, `RobotRuntime`,
`RobotController`, `CommandScript`, or `SensorScript`. Conversely,
`Application.cpp`/`Application.hpp`/`src/main.cpp` have zero diff in this
phase; `RobotSimulator` was not given a `--3d`/`--visual` flag or any
other new option. Connecting the visual robot's pose to the real FSM/
runtime is explicitly future work, not attempted here. (Historical note:
this paragraph describes Phase 13M as originally shipped. Phase 13N,
below, is that future work - `main3d.cpp` now does construct the real
`RobotStateMachine`/`RobotController`/`RobotRuntime`. The CLI wiring
statement remains true unchanged: `RobotSimulator`/`Application` still
have no `--3d`/`--visual` flag or any diff in Phase 13N either.)

**Tests stay headless, deliberately.** `VirtualWorldTests.cpp` exercises
only `VirtualWorld`'s deterministic constructed scene (initial robot
position/height/heading, obstacle count and sizes, base platform position/
footprint, and construction determinism across instances) - it never
calls `InitWindow()` or any raylib drawing function, and because
`robot_visual_world` has no raylib dependency at all, it structurally
cannot. No test asserts on rendered pixels or raylib draw-call behavior,
per the task brief's explicit instruction.

## Phase 13N: FSM-driven 3D robot movement

Connects the existing robot-control architecture to the 3D visual
simulator, so the on-screen robot actually moves when the real FSM/
controller commands movement - closing the gap the Phase 13M section
above explicitly deferred:

```text
DemoCommandSource (visual-only)
      |
      v
  RobotRuntime                     (unchanged)
      |
      v
RobotStateMachine                  (unchanged)
      |
      v
 RobotController                   (unchanged)
      |
      v
VirtualRobotHardware               (new, Phase 13N)
      |
      v
VirtualWorld::setRobotPosition()   (new mutation entry point)
      |
      v
   Renderer3D                      (draws whatever pose VirtualWorld holds)
```

**`VirtualRobotHardware` is a fourth `IRobotHardware` implementation,
alongside `SimulatedRobotHardware`, `RealRobotHardware`, and (implicitly)
any future one.** `RobotController`/`RobotRuntime` needed zero changes to
work with it, for exactly the same reason `RealRobotHardware` (Phase 13K)
needed none: both only ever depend on the `IRobotHardware` interface, not
a concrete type. `VirtualRobotHardware::moveForward()/stop()/
returnToBase()` only ever record a `VirtualDriveCommand` - they never
touch `VirtualWorld` directly. Actual movement happens later, once per
frame, inside `update(deltaSeconds)`, the only place this class mutates
`VirtualWorld` - mirroring the same "record the command now, act on it
later" split the task brief specified, and keeping actuator-command
handling side-effect-free at the exact moment `RobotController` calls it
(consistent with `RobotController::applyState()`'s own synchronous,
non-blocking contract).

**Movement direction was derived from `VisualRobot.cpp`'s actual
`rlRotatef` call, not assumed.** `drawVisualRobot()` applies
`rlRotatef(pose.headingDegrees, 0, 1, 0)` in local space before drawing
the front marker at local `(0, h/2, L/2)` (i.e. local +Z). Working through
the standard right-hand-rule Y-rotation matrix that `rlRotatef` implements
gives `x' = x*cosθ + z*sinθ`, `z' = -x*sinθ + z*cosθ`; substituting the
marker's local `(0, z=L/2)` yields a front direction of
`(sin θ, 0, cos θ)`. `VirtualRobotHardware::update()` uses exactly that
same `(sin, cos)` mapping for forward movement, so heading 0 moves along
+Z and heading 90 moves along +X - matching the front marker exactly, by
construction rather than coincidence. `HeadingZeroMovesInFrontMarkerDirection`/
`HeadingNinetyMovesInCorrectDirection` in `VirtualRobotHardwareTests.cpp`
assert both cases numerically against this derivation.

**`VirtualWorld` gained two narrow mutation methods, not a general
setter.** `setRobotPosition(const Vec3&)` and `setRobotHeading(float)`
are `VirtualWorld`'s only mutation entry points - rendering continues to
consume `VirtualWorld` through a `const&` exclusively (see `Renderer3D`).
`setRobotHeading()` has no production caller yet in this phase (the demo
scene's heading never changes - there is no turning/differential-drive
logic yet), but exists so `VirtualRobotHardwareTests.cpp` can exercise the
heading-direction convention directly rather than only ever testing the
one heading (0) the demo scene happens to start at; it is also the ready
mutation point a future turning phase will need.

**`DemoCommandSource` is a real, if minimal, `IPollingEventSource` -
not a way to fake FSM state.** It delivers exactly `ScenarioLoaded` then
`StartMission`, once each, then `std::nullopt` forever. `RobotRuntime`
polls it exactly like it would poll `HardwareEventSource` or
`ScriptedCommandEventSource` - the FSM reaches `Moving` through its real
`processEvent()` transition rules, never by `main3d.cpp` calling
`RobotStateMachine`'s state directly (which it can't - there is no such
setter). This was deliberately not built by reusing `CommandScript`/
`ScriptedCommandEventSource`: `CommandScript`'s constructor requires an
on-disk file, and writing a temporary file at startup for a two-line,
never-changing demo script would be more machinery than the two-`switch`-case
class it replaces. `DemoCommandSource` is visual-only
(`include/robot/visual/DemoCommandSource.hpp`) and does not touch
production `CommandScript`/`ScriptedCommandEventSource` semantics at all.

**Exactly one `RobotRuntime::step()` per rendered frame - the render
loop is the scheduler, not `LiveRuntimeRunner`.** `LiveRuntimeRunner`
(Phase 13G) is a *finite*, bounded-cycle-count scheduler - the wrong shape
for an interactive window that runs for an unknown, unbounded number of
frames until the user closes it. `main3d.cpp` instead calls
`runtime.step()` directly once per iteration of its own
`while (!WindowShouldClose())` loop, exactly as the task brief specified;
once `DemoCommandSource` is exhausted, every subsequent `step()` call
legitimately returns `NoEvent` (per `RobotRuntime`'s own Phase 13F
contract) and does nothing further - this is expected steady-state
behavior, not an error.

**World bounds are a simple clamp, explicitly not collision detection.**
`VirtualRobotHardware::update()` clamps the robot's `x`/`z` to a
`+-10`-unit square (matching `Renderer3D`'s own ~10x10 ground/grid) after
computing the new position, so continuous `MoveForward` cannot drift the
robot indefinitely far from the visible scene. Obstacle boxes and the base
platform are not checked against the robot's position at all - the robot
can currently pass straight through them. Obstacle sensing, collision,
differential-drive turning, and `ReturnToBase` navigation are all
explicitly out of scope for this phase; `ReturnToBase` is accepted as a
valid command (so `RobotController`/`VirtualRobotHardware` never reject
it) but `update()` treats it identically to `Stopped` - the preferred
behavior the task brief specified over silently doing nothing different.

**HUD state/command text crosses the `robot_visual`/
`robot_visual_simulation` boundary as plain strings, not typed values.**
`Renderer3D::renderFrame()` gained two `std::string_view` parameters
(`stateText`, `commandText`) rather than accepting a `RobotState` or
`VirtualDriveCommand` directly - `robot_visual` has no dependency on
`robot_visual_simulation` (they are sibling targets under
`RobotSimulator3D`, per the task brief's preferred graph), so `Renderer3D`
cannot know either type by name. `main3d.cpp` converts each value to text
itself, reusing the existing `robot::toString(RobotState)` (`RobotState.hpp`,
unmodified) and a small new visual-only `robot::visual::toString(VirtualDriveCommand)`
(mirroring that same shape) rather than inventing a different pattern.
`Renderer3D::drawHud()` was also refactored from five hand-tracked pixel
Y-offsets to a small `{text, fontSize, color}` array processed in a loop -
not required by this phase, but the two new HUD lines made the old
hand-offset approach error-prone enough that fixing it in the same change
was clearly worthwhile, and the panel-sizing logic (already
content-width-aware since the HUD-readability fix) needed no further
change to accommodate the extra lines.

**CMake dependency shape.** `robot_visual_simulation` depends on
`robot_visual_world`, `robot_hardware`, `robot_controller`, `robot_runtime`,
and `robot_domain` - the same real FSM/controller/runtime stack the CLI's
`robot_app` depends on, all `PUBLIC` since `VirtualRobotHardware.hpp`'s
public API surface touches `IRobotHardware` (from `robot_hardware`) and
`VirtualWorld` (from `robot_visual_world`). It deliberately has no
dependency on `robot_visual` or raylib - `VirtualRobotHardware` has no
raylib dependency of its own, matching `SimulatedRobotHardware`/
`RealRobotHardware`'s own boundaries exactly. `RobotSimulator3D` links
both `robot_visual` and `robot_visual_simulation` as siblings; neither of
those two targets depends on the other, avoiding the reverse
`robot_visual -> robot_visual_simulation` edge a shared-HUD-type approach
would have required.

**Nothing in the already-tested robot-control stack changed.**
`RobotStateMachine`, `RobotController`, `RobotRuntime`, `IRobotHardware`,
`HardwareEventSource`, `Application`, and `Simulator` all have zero diff
in this phase - confirmed via `git diff --stat` before merging, the same
verification discipline every prior phase in this document has used.
`VirtualRobotHardwareTests.cpp`'s final integration test
(`FsmControllerHardwareWorldIntegrationThroughRobotRuntime`) drives the
exact same `RobotRuntime`/`DemoCommandSource`/`RobotStateMachine`/
`RobotController`/`VirtualRobotHardware` composition `main3d.cpp` uses,
headless, proving the full
`FSM -> RobotController -> VirtualRobotHardware -> VirtualWorld` chain
end to end without opening a window.

## Phase 13O: virtual distance sensor / obstacle detection

Closes the remaining gap Phase 13N's section explicitly left open ("no
obstacle sensor exists in `VirtualRobotHardware`"): the 3D simulator now
detects obstacle geometry and feeds that detection through the existing
production event/FSM/controller path, unmodified:

```text
VirtualWorld obstacle geometry (BoxObstacle.enabled, position, size)
      |
      v
VirtualDistanceSensor              (new, Phase 13O - raylib-free, headless)
      |
      v
VirtualRobotHardware::obstacleDetected()   (now geometry-backed)
      |
      v
HardwareEventSource                (unchanged)
      |
      v
ObstacleDetected / ObstacleCleared
      |
      v
CompositePollingEventSource        (unchanged, command-before-sensor priority)
      |
      v
RobotRuntime -> RobotStateMachine -> RobotController   (all unchanged)
      |
      v
VirtualRobotHardware command (Stopped / MoveForward)
      |
      v
VirtualRobotHardware::update(dt) -> VirtualWorld robot pose -> Renderer3D
```

**`VirtualDistanceSensor` is a pure `VirtualWorld` geometry query, with no
FSM/Event/`IRobotHardware` knowledge of its own.** It takes a
`const VirtualWorld&` and answers exactly two questions:
`distanceToNearestObstacle()` (an `std::optional<float>`, never infinity or
a negative sentinel) and `obstacleDetected()` (that distance, if any,
`<= kDetectionDistance`). It never mutates `VirtualWorld`, never decides an
FSM transition, and never constructs an `Event` - the sensor/decision
separation this project has followed since the very first `IEventSource`
design (see "Sensor/decision separation" above) applies here exactly the
same way: `VirtualDistanceSensor` senses, `VirtualRobotHardware` exposes
that sense through `IRobotHardware::obstacleDetected()`, and
`HardwareEventSource` (completely unmodified) is the only place a sensor
reading becomes an `Event`.

**Ray/AABB intersection is a 2D (X/Z) slab test, not raylib's
`GetRayCollisionBox`.** Obstacles only need their X/Z footprint considered
for Phase 13O (Y/height is irrelevant to a ground-level forward sensor), so
`VirtualDistanceSensor.cpp` implements the classic slab algorithm directly:
narrow `[tMin, tMax]` against each axis' pair of planes, treating a
near-zero direction component (`|component| < 1e-6`) as exactly parallel to
that axis rather than dividing by it (avoids NaN/Inf from a tiny non-zero
float - e.g. heading 90° produces `cos(90°)` as a small float residual, not
exactly `0.0`, purely from the degrees-to-radians conversion). A final
`tMax < 0` check discards intersections entirely behind the sensor; a
negative `tMin` with non-negative `tMax` (sensor origin already inside an
obstacle - not expected in the demo scene, but handled deterministically
rather than left undefined) clamps to a touching distance of `0`, not a
negative one. No raylib collision helper is used anywhere in this file, so
it stays genuinely headless and unit-testable (`VirtualDistanceSensorTests.cpp`,
18 tests, no window).

**One shared `forwardDirection()` helper, not three independent heading
implementations.** Phase 13N's `VirtualRobotHardware::update()` originally
inlined its own `sin`/`cos` heading-to-direction math. Phase 13O needed the
exact same convention twice more - the sensor's ray direction, and (in
`main3d.cpp`) the sensor-ray visualization's direction - and the task brief
was explicit that these must never be independently guessed. Rather than
copy the math a third time, `VisualMath.hpp` (new, header-only, raylib-free)
extracts `forwardDirection(const RobotPose&)`, and
`VirtualRobotHardware::update()` was refactored to call it too, so there is
now exactly one heading convention in the codebase, proven against
`VisualRobot.cpp`'s actual `rlRotatef()` call, used identically by movement,
sensing, and rendering.

**The sensor origin is the robot's front, not its center - computed from
the same `RobotDimensions` the renderer already uses.**
`VirtualDistanceSensor::sensorOrigin()` returns
`pose.position + forwardDirection(pose) * (RobotDimensions::kBodyLength / 2)`.
`RobotDimensions` lives in `VisualRobot.hpp`, which has zero raylib
`#include`s of its own (only `VisualRobot.cpp` includes `raylib.h`/`rlgl.h`),
so `VirtualDistanceSensor.cpp` can reuse it directly as a shared source of
truth for the robot's physical size without linking `robot_visual` or
introducing a second, duplicated `kBodyLength` constant anywhere.

**`VirtualWorld` gained two small obstacle mutators,
`setObstaclePosition()`/`setObstacleEnabled()`, following the exact
precedent Phase 13N already set with `setRobotPosition()`/
`setRobotHeading()`.** Both exist for the same reason: without them, there
would be no way to exercise `VirtualDistanceSensor`'s ray/AABB geometry
against controlled, deterministic obstacle placements in
`VirtualDistanceSensorTests.cpp` (multiple-obstacles, side, behind,
beyond-range, tangent/boundary cases) without inventing a second,
disconnected obstacle representation just for tests. `BoxObstacle` gained
one new field, `enabled` (default `true`, so every existing call site is
unaffected), which both `Renderer3D` (skips drawing a disabled obstacle) and
`VirtualDistanceSensor` (skips sensing one) respect identically -
`enabled` is a single source of truth for "this obstacle currently exists
in the world," not two independent flags that could drift apart.
`VirtualWorld::kBlockingObstacleIndex` (a `static constexpr std::size_t`,
value `3`) names the one demo obstacle `RobotSimulator3D`'s `O` key
toggles, so main3d.cpp never hard-codes a bare index.

**The demo obstacle layout needed exactly one small, deliberate
adjustment.** Of the four Phase 13M demo obstacles, none sat on the robot's
actual heading-0 forward ray (constant X = -3.0) - the closest was 1 unit
off-axis. Rather than redesign the scene, the fourth obstacle's position
moved from `(-2.0, 0.4, 3.0)` to `(-3.0, 0.4, 4.3)` - X now matches the
robot's start X exactly (so the ray, which never changes X at heading 0,
runs straight through its footprint), and Z was chosen so the sensor's
first in-range reading (2.3 units, since the sensor origin starts at
Z = 1.4 and the obstacle's near face is at Z = 3.7) is comfortably above
`kDetectionDistance` (1.0) - the robot visibly travels for over a second at
its 1.0 unit/s forward speed before detection fires, and stops with roughly
a full unit of clearance in front of the obstacle (well over its own 0.8
body length), so ordinary frame timing can never visually overlap it. Size
was left unchanged. This is the only geometry change in this phase; no
other obstacle moved.

**`HardwareEventSource`, `CompositePollingEventSource`, `RobotStateMachine`,
`RobotController`, and `RobotRuntime` all have zero diff in this phase** -
confirmed via `git diff --stat` before merging, the same verification
discipline every prior phase in this document has used.
`main3d.cpp` now constructs `HardwareEventSource` (over the same
`VirtualRobotHardware` instance `RobotController` drives) and
`CompositePollingEventSource` (combining it with `DemoCommandSource`,
command-before-sensor priority, unmodified from Phase 13J) - exactly the
same live-event composition `Application::runLiveSimulation()` already uses
for the CLI's `--live` mode, just assembled by hand in `main3d.cpp` instead
of by `robot_app`. main3d.cpp never constructs an `Event` or calls
`RobotStateMachine::processEvent()` itself; `ObstacleDetected`/
`ObstacleCleared` only ever originate from `HardwareEventSource` sampling
`VirtualRobotHardware::obstacleDetected()`.

**Frame update order is deliberate: input, then `runtime.step()`, then
`hardware.update(dt)`, then telemetry, then render.** If
`VirtualDistanceSensor` newly reports a hit within threshold,
`runtime.step()` is where `HardwareEventSource` samples it, the FSM accepts
`Moving -> WaitingForObstacleClear`, and `RobotController` calls
`hardware.stop()` - all *before* `hardware.update(dt)` runs later that same
frame, so the robot never takes one extra frame's worth of movement past
the moment detection fires. (It can still overshoot the exact 1.0-unit
threshold by at most one frame's travel distance - a few hundredths of a
world unit at 60 FPS - because the sensor is polled once per step rather
than continuously; the obstacle's placement leaves roughly a full unit of
margin specifically so this is never visible.) Telemetry (a
`VisualTelemetry` struct) is collected only after both steps, so the HUD/
sensor-ray always reflect the same position `hardware.update()` just
produced this frame, not a stale pre-update reading.

**`O` only ever changes world geometry - it never emits an `Event`
directly.** `main3d.cpp`'s `KEY_O` handler calls
`world.setObstacleEnabled(VirtualWorld::kBlockingObstacleIndex, !enabled)`
and nothing else. The next `runtime.step()` is what actually notices the
change, through the same real `HardwareEventSource` edge-triggered sampling
every other sensor condition in this codebase already goes through -
disabling the obstacle produces a true→false edge (`ObstacleCleared`)
exactly like a real sensor clearing, not a simulator-specific shortcut.

**`SPACE`'s pause semantics are unchanged from Phase 13N and still only
gate `hardware.update(dt)`.** `runtime.step()` (and therefore
`HardwareEventSource` sampling) keeps running every frame regardless of
pause state - pausing freezes the robot's position, not FSM/event
processing. This was already true in Phase 13N; Phase 13O didn't need to
touch it, only to confirm obstacle detection still behaves correctly against
a paused robot (a stationary sensor origin just keeps producing the same
reading every sample, which the edge-trigger logic already handles as a
non-event).

**`Renderer3D` still doesn't know about `IRobotHardware`, `RobotRuntime`,
`RobotStateMachine`, `HardwareEventSource`, or `VirtualDistanceSensor` -
`main3d.cpp` now hands it one `VisualTelemetry` struct instead of two loose
string parameters.** `VisualTelemetry` bundles the already-formatted state/
command text (Phase 13N's approach, unchanged in spirit) with the sensor's
origin, direction, hit distance, detected flag, and maximum range - all
plain data (`Vec3`, `std::optional<float>`, `bool`, `float`), computed once
in `main3d.cpp` from a `VirtualDistanceSensor` instance it owns for display
purposes (separate from the one inside `VirtualRobotHardware`, but reading
the same `VirtualWorld` and running the identical, shared geometry code, so
both always agree). This keeps `robot_visual` a sibling of
`robot_visual_simulation`, never layered on it, while still letting the
rendered sensor ray's origin/heading/length come from the *actual* sensor
calculation rather than a renderer-side approximation - satisfying "the
sensor visual must match the sensor math" without `Renderer3D` duplicating
any geometry constant.

**Still no collision solver.** The sensor/FSM stop the demo robot before it
reaches the blocking obstacle, but `VirtualRobotHardware::update()`'s
movement math remains exactly what Phase 13N left it: it moves the robot
whenever the command is `MoveForward`, with no awareness of obstacle
geometry at all. Nothing prevents geometric overlap if a future phase drives
the robot into an obstacle some other way (a different heading, a moved
obstacle, `ReturnToBase` navigation once implemented) - there is still no
collision response, sliding, or penetration correction, and none is added in
this phase. Differential-drive physics and autonomous steering/pathfinding
remain out of scope as well; obstacle removal is deliberately manual (`O`)
for this phase, not any kind of automatic avoidance.

## Phase 13P: differential-drive kinematics

Closes the gap Phase 13N/13O's sections both explicitly left open
("straight-line motion only... no differential drive"). `VirtualRobotHardware`
no longer computes `x += forward.x * distance` itself - it delegates to a
new, raylib-free `DifferentialDrive` model:

```text
robot_visual_simulation
    VirtualRobotHardware               (unchanged role, Phase 13N/13O)
    VirtualDistanceSensor              (unchanged, Phase 13O)
    DifferentialDrive                  (new, Phase 13P - raylib-free, headless)
```

**Equations** (standard differential-drive kinematics):

```text
v     = (vRight + vLeft) / 2        (robot linear velocity, world units/sec)
omega = (vRight - vLeft) / L        (robot angular velocity, rad/sec; L = wheel track)
```

using this project's one heading convention throughout (`VisualMath.hpp`'s
`forwardDirection()`): `headingDegrees` 0 faces +Z, increasing
`headingDegrees` rotates the front marker from +Z toward +X.

**Exact arc integration, not a discrete Euler step.** For `|omega|` above a
small epsilon, `DifferentialDrive::update()` uses the closed-form solution
of `dx/dt = v*sin(theta(t))`, `dz/dt = v*cos(theta(t))`,
`theta(t) = theta0 + omega*t` over one frame's `deltaSeconds`, rather than
approximating the curve with straight-line steps. This keeps heading and
position exactly consistent regardless of how large `deltaSeconds` is (no
frame-rate-dependent turning-radius error), and it means in-place rotation
(`v == 0`) produces *exactly* zero position change, not merely
"approximately unchanged." Below the epsilon, motion falls back to the
Phase 13N straight-line formula (`dx = sin(theta)*v*dt`,
`dz = cos(theta)*v*dt`) directly - both formulas agree in the limit, so
there is no discontinuity at the epsilon boundary. `headingDegrees` is
normalized into `[0, 360)` after every `update()` call so it never grows or
shrinks without bound across a long-running session.

**Wheel track is an instance parameter, not a hidden global constant.**
`DifferentialDrive::kDefaultWheelTrack` is derived from
`RobotDimensions::kBodyWidth` (`VisualRobot.hpp`) - the same source of
truth `VisualRobot.cpp` uses to place the wheels - so this phase never
introduces an independently-guessed geometry constant, following the exact
precedent `VirtualDistanceSensor.cpp` already set by reusing
`RobotDimensions::kBodyLength` for its sensor-origin offset. The track is a
constructor parameter (defaulting to `kDefaultWheelTrack`) rather than a
`static constexpr`, specifically so `DifferentialDriveTests.cpp` can prove
"a narrower track turns faster for the same wheel-speed differential"
without a second class or a global mutable constant.

**`VirtualRobotHardware::update()` no longer branches on `currentCommand()`
at all.** Previously it early-returned unless the command was
`MoveForward`; now it unconditionally copies `VirtualWorld`'s pose, runs
`DifferentialDrive::update()`, clamps the resulting X/Z to the existing
~10-unit world bounds (unchanged from Phase 13N - `DifferentialDrive`
itself is deliberately world-bounds-agnostic, it only knows kinematics),
and writes the result back through `VirtualWorld::setRobotPosition()`/
`setRobotHeading()`. This is simpler, not just equivalent: `Stopped`/
`ReturnToBase` naturally produce zero wheel speeds -> zero `v`/`omega` ->
zero pose change, with no special-casing needed, and it is what lets the
manual override (below) move the robot even while `currentCommand()` is
`Stopped`.

**Manual wheel override lives on `VirtualRobotHardware`, not a new
type.** The phase brief allowed either "fit it into `VirtualRobotHardware`"
or "a separate visual-only override API if ownership gets awkward" - it
fit cleanly, following the exact precedent `currentCommand()`/
`obstacleDistance()` already set (Phase 13N/13O): visual-simulator-only
getters/setters that sit *beside* the `IRobotHardware` override set, never
inside it. `setManualWheelSpeeds()`/`clearManualWheelOverride()` are not
part of `IRobotHardware`, and `moveForward()`/`stop()`/`returnToBase()`
still always update `currentCommand()` even while an override is active -
only the *physical* wheel speeds `DifferentialDrive` uses are affected,
via a private `applyCurrentCommandToDrive()` helper that no-ops while
`manualOverrideActive_` is true. This is what makes
`clearManualWheelOverride()`'s "restore the latest FSM command" behavior
trivial: it just calls the same helper the FSM-facing methods already call.

**`main3d.cpp` owns manual drive mode entirely; `IRobotHardware`,
`RobotController`, `RobotRuntime`, and `RobotStateMachine` gained zero new
code.** `M` toggles `manualDriveMode`, a `bool` local to `main()`; while
true, `UP`/`DOWN`/`LEFT`/`RIGHT`/`X` are read every frame (recomputing
wheel speeds from scratch, so releasing every key already yields zero -
`X` is honored anyway for an explicit "stop" per the brief) and fed to
`VirtualRobotHardware::setManualWheelSpeeds()` - never through
`RobotController` or any FSM-facing call, and the HUD's "Drive mode" line
makes it unmistakable when this is active. **Arrow keys, not `WASD`,
deliberately** - `CAMERA_FREE`'s own keyboard movement is bound to `WASD`
(`rcamera.h`'s `UpdateCamera()`), so arrow keys sidestep *that* conflict.

**Bug found in real testing, fixed same phase: `UpdateCamera()` also reads
arrow keys, independently of `WASD`.** The first cut of this phase assumed
arrow keys were conflict-free and left `renderer.renderFrame()` receiving
`cameraCaptured` as its `updateCamera` argument unconditionally - so the
camera kept updating every frame regardless of `manualDriveMode`. Reading
raylib 6.0's actual vendored `rcamera.h` (not just recalling its API)
showed `UpdateCamera()` unconditionally calls `CameraPitch()`/`CameraYaw()`
from `IsKeyDown(KEY_UP/KEY_DOWN/KEY_LEFT/KEY_RIGHT)` in every mode except
`CAMERA_CUSTOM`/`CAMERA_ORBITAL` - including `CAMERA_FREE` - on top of its
continuous, unbounded mouse-look while `DisableCursor()` is active (which
`RobotSimulator3D` enables by default). So every manual-drive arrow-key
press was simultaneously read by two independent code paths: main3d's own
`IsKeyDown()` polling (setting the correct wheel speeds - proven correct
by `DifferentialDriveTests`/`VirtualRobotHardwareTests`, which never
touch raylib) *and* `UpdateCamera()`'s pitch/yaw, plus any incidental
mouse movement. The wheel speeds and `VirtualWorld` pose were always
right; the camera was quietly rotating/drifting at the same time, which
could make correctly-moving wheels look like nothing was happening. Fix:
`main3d.cpp` now computes `updateCamera = cameraCaptured &&
!manualDriveMode` and passes that instead - camera updates (mouse-look
and arrow-key pitch/yaw alike) are suppressed entirely for as long as
manual drive mode is active, exactly the "Preferred: do not call
`UpdateCamera()` with keyboard movement while manual drive is active"
option this phase's brief already named. This is a rendering/input-layer
fix only - `DifferentialDrive`, `VirtualRobotHardware`, and every existing
test were untouched and still pass.

**`RobotRuntime::step()` keeps running unmodified while manual mode is
on** - obstacle detection, `ObstacleDetected`/`ObstacleCleared`, and
`WaitingForObstacleClear` all still fire exactly as in Phase 13O. What
changes is only that `RobotController::stop()`'s call to
`VirtualRobotHardware::stop()` no longer reaches `DifferentialDrive` while
an override is active (see above) - so a manually-driven robot is not
physically halted by the FSM reaching `WaitingForObstacleClear`. This is
intentional per the brief: manual mode's entire purpose is proving
kinematics/turning, and it is unmistakably labeled `MANUAL` in the HUD
whenever it could diverge from normal safety behavior.

**No new FSM states, no turn events, no `IRobotHardware` methods.** Turning
capability exists only inside `DifferentialDrive`/`VirtualRobotHardware`;
`RobotController`'s `RobotState -> IRobotHardware` mapping
(`applyState()`) is byte-for-byte unchanged from Phase 13N. `Moving` still
means "equal positive wheel speeds" and `WaitingForObstacleClear` still
means "zero wheel speeds," exactly as before - Phase 13P only changed
*how* those wheel speeds turn into position/heading.

**Still no autonomous obstacle avoidance.** Nothing steers the robot around
an obstacle automatically - obstacle removal is still manual (`O`), and
that scope boundary is unchanged from Phase 13N/13O. Wheel speeds also
still change instantly on command - no acceleration/inertia/friction model
exists, matching the brief's explicit "no physics-engine complexity"
instruction for this phase.

### Real-testing fixes: manual-mode `X` priority and an obstacle-penetration guard

Human manual validation of the above surfaced two further defects, both
fixed within this same phase (not a new one):

**Bug 1 - `X` did not reliably stop the robot.** `main3d.cpp` originally
read `X` with `IsKeyPressed()` (the single-frame press edge) and applied it
*after* computing the directional wheel speeds, zeroing them out only for
that one frame. On the very next frame, if a directional key was still
physically held, `IsKeyPressed(KEY_X)` was already false, so that key's
branch fired again and the robot resumed moving - `X` was a one-frame
blip, not a real override. Fixed by extracting the whole decision into a
pure, raylib-free function, `computeManualWheelSpeeds(upHeld, downHeld,
leftHeld, rightHeld, xHeld, wheelSpeed)`
(`include/robot/visual/ManualDriveInput.hpp` /
`src/visual/ManualDriveInput.cpp`): `xHeld` (read via `IsKeyDown()`, i.e.
held, not just pressed) is checked first and, while true, returns `{0,
0}` immediately without evaluating any directional argument at all - so a
previous command can never remain latched, and `X` has unconditional
highest priority for as long as it is held. `main3d.cpp` now only ever
calls this function with fresh `IsKeyDown()` results and forwards its
result straight to `setManualWheelSpeeds()`. Splitting this out of
`main3d.cpp` also means the decision table (X overrides everything;
UP/DOWN/LEFT/RIGHT are additive; opposite pairs cancel) is unit-tested
directly (`ManualDriveInputTests.cpp`) instead of only being provable
interactively.

**Bug 2 - manual mode could drive the robot through an enabled obstacle.**
Manual mode intentionally bypasses `RobotController::stop()`'s effect on
wheel speeds (see above) - by design, so kinematics could be proven even
while `WaitingForObstacleClear`. But nothing stopped the *position* itself
from entering an obstacle's geometry, since `VirtualRobotHardware::update()`
only ever clamped to the world's outer bounds. Fixed with a small,
deliberately simple, raylib-free collision guard - not a physics engine,
no bounce/sliding/force response:

```text
DifferentialDrive
    computes a proposed RobotPose (world-agnostic, as always)
        |
        v
VirtualRobotHardware::update()
    clamps proposed X/Z to the existing +-10 world bounds
        |
        v
    validates the clamped proposed position against VirtualWorld's
    enabled obstacle geometry (RobotCollision.hpp)
        |
        v
    collision -> commit only the proposed heading, keep the previous
                 position (translation rejected for this frame)
    no collision -> commit both position and heading
```

`robotPositionCollidesWithObstacles(position, obstacles)`
(`include/robot/visual/RobotCollision.hpp` / `src/visual/RobotCollision.cpp`)
is a circle-vs-AABB test: the robot's collision footprint is a single
conservative circle, `kRobotCollisionRadius = sqrt((kBodyWidth/2)^2 +
(kBodyLength/2)^2)` (0.5 world units for this robot, derived from
`RobotDimensions` - never hand-duplicated), which fully encloses the
rectangular body regardless of heading - so the test is entirely
rotation-independent and needs no orientation information. For each
enabled obstacle, `closestX/Z = clamp(robotX/Z, min, max)` finds the
nearest point on its AABB, and `dx^2 + dz^2 < radius^2` (with a `1e-4`
epsilon added to the radius, so the boundary is deterministic rather than
occasionally flickering at exact floating-point tangency) decides
collision. Disabled obstacles never collide - re-enabling one makes it
collide again immediately, with no special-casing needed. This lives in
`robot_visual_simulation` as a small dedicated helper (matching
`VirtualDistanceSensor`'s precedent) rather than as a `VirtualWorld`
member, so `VirtualWorld` itself needed zero changes; `DifferentialDrive`
still has no obstacle/collision knowledge whatsoever.

**Why only the position is rejected, not the whole pose.** The collision
footprint is a circle, so it does not depend on heading - a position that
was not colliding before a pure in-place rotation is still not colliding
after it (the proposed position is identical to the previous one, since
`v == 0`). Committing the heading unconditionally means turning at an
obstacle boundary is never blocked by this guard, only forward/backward
translation into the obstacle is - confirmed by
`InPlaceRotationDoesNotTranslateIntoObstacle`.

**This guard is a last-resort physical safety net, not a replacement for
Phase 13O's sensor.** In normal (non-manual) FSM mode,
`VirtualDistanceSensor`/`HardwareEventSource` are completely unmodified and
still what actually halts the robot - at `kDetectionDistance` (1.0 world
units from the front sensor origin), which puts the obstacle face roughly
1.4 units from the robot's center, well outside the 0.5-unit collision
radius. The collision guard exists specifically for manual mode, where the
sensor's stop is deliberately bypassed; it never engages during normal FSM
operation, which is exactly why
`FullClosedLoopObstacleDetectionAndClearThroughRealEventChain` (unchanged)
still passes.

## Phase 13Q: reactive obstacle avoidance

Closes the gap Phase 13N's original section (and Phase 13P's README text)
both explicitly left open: "autonomous obstacle avoidance/steering is not
implemented." `RobotSimulator3D` now turns itself away from an obstacle
and continues, entirely through the real event chain, with zero new FSM
states or transitions.

**This is REACTIVE avoidance, not navigation.** Explicitly not
pathfinding, not A*, not waypoint planning, not SLAM/mapping. The entire
V1 policy is "turn in place until the forward sensor clears, then
continue" - it may permanently change the robot's heading with no attempt
to return to its original trajectory. Goal-directed recovery/path planning
is out of scope for this phase.

### Control authority: three sources, one fixed priority

Before this phase, only two things could ever want the wheels (FSM/
`RobotController` and Phase 13P's manual override). This phase adds a
third: autonomous avoidance. Rather than scattered ad-hoc `if` statements
picking a winner in different places, `VirtualRobotHardware` now exposes
one explicit, testable concept:

```cpp
enum class DriveAuthority { Fsm, AutonomousAvoidance, Manual };
```

with a single fixed priority, `Manual > AutonomousAvoidance > Fsm`,
implemented in exactly one place - a private `applyEffectiveWheelSpeeds()`
that every wheel-affecting entry point (`moveForward()`/`stop()`/
`returnToBase()`, `setManualWheelSpeeds()`/`clearManualWheelOverride()`,
`setAutonomousWheelSpeeds()`/`clearAutonomousWheelOverride()`) funnels
through:

```cpp
void VirtualRobotHardware::applyEffectiveWheelSpeeds() noexcept
{
    WheelSpeeds speeds;
    if (manualOverrideActive_)          speeds = manualSpeeds_;
    else if (autonomousOverrideActive_) speeds = autonomousSpeeds_;
    else                                speeds = wheelSpeedsForCommand(command_);
    drive_.setWheelSpeeds(speeds.left, speeds.right);
}
```

**Three deliberately distinct concepts, tested and documented as such**
(the brief's section 16 called out exactly this naming risk):

| Getter | Answers |
|---|---|
| `currentCommand()` | What `RobotController`/the FSM *wants* (unchanged Phase 13N concept) |
| `driveAuthority()` | Who currently owns the physical wheels right now |
| `wheelSpeeds()` | What `DifferentialDrive` is actually executing |

This is why, mid-avoidance-turn, the HUD routinely shows `Command:
Stopped` (`RobotController` still wants the robot stopped - Phase 13O's
`WaitingForObstacleClear -> stop()` mapping is completely unmodified)
next to `Drive authority: AUTONOMOUS` with opposite-sign wheel speeds -
proof that the visual autonomous-locomotion layer, not the FSM, is
temporarily driving the wheels, exactly as the brief's section 15
required. Both overrides are tracked independently
(`manualOverrideActive_`/`manualSpeeds_`,
`autonomousOverrideActive_`/`autonomousSpeeds_`) and never destroyed by an
unrelated actor: an FSM command arriving while an override is active still
updates `command_`/`currentCommand()`, but `applyEffectiveWheelSpeeds()`
only lets it reach the physical wheels once neither override is active.
Clearing manual falls through to autonomous if one is still pending,
otherwise to the FSM command; clearing autonomous falls through to the FSM
command only if manual is not active (manual still wins either way) -
proven directly by `ClearingManualRestoresAutonomous`/
`ClearingAutonomousRestoresFsm`/`ManualStillWinsAfterFsmCommandChanges`/
`ClearingAllOverridesRestoresLatestFsmCommand`.

**A pending avoidance request is never lost while manual is active.**
`main3d.cpp` simply does not evaluate the avoidance policy at all while
`manualDriveMode` is true - it neither sets nor clears the autonomous
override during that time. So if avoidance was active when `M` was
pressed, `autonomousOverrideActive_`/`autonomousSpeeds_` sit untouched
underneath the (higher-priority) manual override; the moment
`clearManualWheelOverride()` runs (on `M` again),
`applyEffectiveWheelSpeeds()` immediately falls through to the still-valid
autonomous speeds - no extra frame of delay, no re-request needed. See
`ManualPriorityIntegrationAcrossAvoidanceAndFsm`.

### `ReactiveObstacleAvoidance`: what, not when

```text
robot_visual_simulation
    VirtualRobotHardware               (extended: DriveAuthority model)
    VirtualDistanceSensor              (unchanged, Phase 13O)
    DifferentialDrive                  (unchanged, Phase 13P)
    RobotCollision                     (unchanged, Phase 13P)
    ManualDriveInput                   (unchanged, Phase 13P)
    ReactiveObstacleAvoidance          (new, Phase 13Q - raylib-free, headless, stateless)
```

`ReactiveObstacleAvoidance::avoidanceWheelSpeeds()` always returns the same
fixed pair, `{-kTurnWheelSpeed, +kTurnWheelSpeed}` with
`kTurnWheelSpeed = 0.6F` (deliberately smaller than
`VirtualRobotHardware::kForwardWheelSpeed` = `1.0F`, so a turn reads as a
distinct maneuver) - it has no FSM, `Event`, `IRobotHardware`,
`VirtualDistanceSensor`, or `VirtualWorld` knowledge whatsoever, exactly
like `ManualDriveInput`'s decision function stays separate from
`main3d.cpp`'s raylib polling. The *when* - avoidance enabled, FSM state is
`WaitingForObstacleClear`, sensor still reports detected - lives entirely
in `main3d.cpp`, never inside this class:

```cpp
const bool shouldAvoidObstacle = avoidanceEnabled &&
    stateMachine.currentState() == robot::RobotState::WaitingForObstacleClear &&
    hardware.obstacleDetected();

if (shouldAvoidObstacle)
{
    const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
    hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
}
else if (hardware.autonomousOverrideActive())
{
    hardware.clearAutonomousWheelOverride();
}
```

**Turn direction is deterministic, verified against the real
`DifferentialDrive` convention, not assumed.**
`left = -kTurnWheelSpeed, right = +kTurnWheelSpeed` gives
`omega = (vRight - vLeft) / wheelTrack > 0`, which - per
`DifferentialDriveTests.cpp`'s own `TurningDirectionMatchesConvention` -
increases `headingDegrees` (rotates the front marker from +Z toward +X,
this project's one heading convention). V1 always turns this one
direction; no obstacle-side clearance probing, no randomness, per the
brief's explicit preference for simplicity over cleverness in this phase.

### Frame order: authority decided *after* `runtime.step()`

```text
1. input: TAB/F11/SPACE/O/M/A (single-press toggles only)
2. runtime.step()                                    <- exactly once
3. determine + apply drive authority (manual, else avoidance, else Fsm)
4. hardware.update(dt), unless SPACE-paused
5. telemetry
6. render
```

Step 3 must run *after* step 2, not before (unlike Phase 13P's manual-only
version, where the ordering didn't matter): the avoidance decision reads
`stateMachine.currentState()`, and that state can change *this same
frame* inside `runtime.step()` - a robot approaching the obstacle can
enter `WaitingForObstacleClear` and start turning within the very same
frame, with no extra frame of lag (matching the brief's section 10: "Frame
N: FSM enters WaitingForObstacleClear, avoidance override starts, robot
rotates"). Symmetrically, the instant `HardwareEventSource` observes the
sensor's real `true -> false` edge and the FSM returns to `Moving` inside
some later frame's `runtime.step()`, that same frame's step 3 immediately
clears the autonomous override (since `shouldAvoidObstacle` is now false)
*before* step 4 moves the robot - so there is no frame where a stale
avoidance turn and the new `Moving` state coexist in the committed pose.

**`SPACE` pause semantics are unchanged and orthogonal to authority.**
Pausing only skips step 4 (`hardware.update()`); steps 2-3 (event
polling, drive-authority determination) keep running exactly as before.
So wheel telemetry can legitimately show avoidance's opposite-sign speeds
while paused - it faithfully reflects who currently has authority - but
the pose genuinely does not change until unpaused.

### Autonomous turning still goes through the Phase 13P collision guard

`setAutonomousWheelSpeeds()` never bypasses `VirtualRobotHardware::update()`'s
existing collision check - avoidance wheel speeds flow through the exact
same `DifferentialDrive -> proposed pose -> RobotCollision -> commit/reject`
path manual driving does. Because the collision footprint is a circle
(rotation-independent), pure in-place avoidance turning is never blocked
by it - proven directly by extending the existing
`InPlaceRotationDoesNotTranslateIntoObstacle`-style reasoning to the
avoidance path in the closed-loop integration test below.

**Real limitation discovered while writing the closed-loop test (not a
bug - documented, expected V1 behavior):** this policy only turns enough
to clear the forward *sensor's* detection threshold
(`VirtualDistanceSensor::kDetectionDistance`, 1.0 world units from the
sensor origin) - it has no notion of the *collision guard's* tighter
circular radius (0.5 world units from the robot's center) and does not
attempt to guarantee unlimited onward travel is safe once it stops
turning. For this project's specific demo obstacle geometry, resuming
`MoveForward` immediately after one avoidance turn clears the sensor can
still run the robot's collision circle back into the very same obstacle
from its new heading within a few frames, at which point the collision
guard (not the FSM/sensor) is what actually caps further progress in that
direction - the robot does not re-enter `WaitingForObstacleClear` in this
case, since the *sensor* genuinely does not redetect (only the tighter
collision circle does). This is why
`FullAutonomousObstacleAvoidanceClosedLoopThroughRealEventChain` asserts
only that *some* genuine forward progress happens after the override
releases, not that the robot travels arbitrarily far - composing two
independently-reasonable, independently-tested safety margins (a
1.0-unit sensor threshold and a 0.5-unit collision radius) does not
automatically guarantee they agree on "clear enough to keep going" for
every possible obstacle geometry and turn angle. A future phase could
close this gap (e.g. by turning until *both* the sensor and a forward
collision projection are clear), but doing so is deliberately out of
scope here - V1's brief is explicit that this is reactive avoidance, not
optimal/complete avoidance.

### What was NOT touched

No new `RobotState` values, no new FSM transitions, no `TurnLeft`/
`TurnRight` (or any other) addition to `IRobotHardware`, no change to
`RobotController`'s `RobotState -> IRobotHardware` mapping, no change to
`HardwareEventSource`'s edge-trigger semantics or safety-priority
ordering, no change to `CompositePollingEventSource`'s command-before-
sensor priority. `stateMachine.processEvent()`/`handleEvent()` is never
called from any Phase 13Q code, and `ObstacleDetected`/`ObstacleCleared`
are never constructed/injected directly - both still only ever come from
`HardwareEventSource` reading `VirtualRobotHardware`'s real sensor state,
exactly as in Phase 13O.

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
