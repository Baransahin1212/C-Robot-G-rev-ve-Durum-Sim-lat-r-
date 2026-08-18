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
