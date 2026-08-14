# References

Sources consulted for concepts, libraries, and tools actually used in this
project. Links point to official/authoritative documentation. No
third-party source code was copied into this repository — libraries are
consumed only as compiled dependencies via CMake `FetchContent`.

- **[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)**
  — general guidance on RAII, avoiding manual `new`/`delete`, preferring
  references over pointers for non-owning parameters, and `enum class`
  over plain `enum`; informed the coding style used throughout
  `include/robot/` and `src/`.

- **[cppreference — `std::optional`](https://en.cppreference.com/w/cpp/utility/optional)**
  — reference for `std::optional<double>` (`Event::value`) and
  `std::optional<std::uint64_t>` (`SimulationResult::lastEventTimestampMs`).

- **[cppreference — `std::filesystem`](https://en.cppreference.com/w/cpp/filesystem)**
  — reference for `std::filesystem::path`, `create_directories`, and
  path composition (`operator/`) used in `Application.cpp` to create the
  `logs/`/`reports/` output directories portably.

- **[CMake documentation — `FetchContent`](https://cmake.org/cmake/help/latest/module/FetchContent.html)**
  — used to pull in GoogleTest and nlohmann/json at configure time, pinned
  to specific tagged releases, without requiring the user to install them
  manually.

- **[GoogleTest — official documentation](https://google.github.io/googletest/)**
  — reference for `TEST()` macros, assertion macros (`EXPECT_EQ`,
  `ASSERT_TRUE`, etc.), and CTest integration via `gtest_discover_tests`.

- **[nlohmann/json — official documentation](https://json.nlohmann.me/)**
  — reference for parsing (`nlohmann::json::parse`, `operator>>`), type
  inspection (`is_object`, `is_array`, `is_number`, `contains`), and
  exception types (`nlohmann::json::parse_error`) used in
  `JsonScenarioSource.cpp`.

- **[CTest documentation](https://cmake.org/cmake/help/latest/manual/ctest.1.html)**
  — reference for running and discovering tests (`ctest -C Debug`,
  `ctest -N`) as used throughout the README and this project's build
  workflow.

- **[Mermaid — state diagram syntax](https://mermaid.js.org/syntax/stateDiagram.html)**
  — reference for the `stateDiagram-v2` syntax used in
  [`state-machine.md`](state-machine.md).

- **General emergency-stop / fail-safe concept reference:
  [ISO 13850 — Safety of machinery: Emergency stop function](https://www.iso.org/standard/74152.html)**
  and **[IEC 61508 — Functional safety overview (IEC)](https://www.iec.ch/functional-safety)**
  — consulted only for the general *concept* of an emergency-stop state
  that requires explicit reset (no automatic recovery). This project
  implements a simplified software analogy of that concept for a
  simulation/demonstration context; it does **not** claim conformance with
  either standard and is not a certified safety system (see
  [`technical-decisions.md`](technical-decisions.md#fail-safe--emergency-stop)).
