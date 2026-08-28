# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PX4-Autopilot: safety-critical C/C++ flight control firmware for autopilots
(NuttX and Linux/POSIX targets), plus SITL simulation and Python tooling.
BSD-3-Clause licensed, hosted by the Dronecode Foundation.

## Build & test

- `make px4_sitl_default` — build default SITL target (used for most dev work).
- `make <vendor>_<board>_default` (e.g. `make px4_fmu-v6x_default`) — build for a specific board. `make list_config_targets` lists all valid targets; configs live under `boards/<vendor>/<model>/*.px4board`.
- `make tests` — build and run the gtest unit test suite (`px4_sitl_test` target). Filter to specific tests with `make tests TESTFILTER=<pattern>` (matches gtest test/suite names, e.g. `TESTFILTER=PositionControl`).
- `test/mavsdk_tests/` — SITL integration tests driven via MAVSDK (`make tests_integration`).
- `make check_format` — CI-enforced astyle format check. `make format` reformats all changed C/C++; `./Tools/astyle/fix_code_style.sh <file>` for a single file.
- `make clang-tidy` / `make cppcheck` / `make scan-build` — static analysis targets.
- Python code is checked with mypy and flake8 (not wrapped by a `make` target).

## Architecture

- **uORB pub/sub middleware** ties everything together: modules communicate exclusively by publishing/subscribing to typed messages rather than calling each other directly. Message schemas are `.msg` files under `msg/`, compiled into generated C++ structs at build time.
- **Modules** (`src/modules/<name>/`) are the runtime components (estimators, controllers, mavlink, commander, etc.), each a self-contained `px4_add_module(...)` CMake target with its own `CMakeLists.txt`, source files, `params.yaml` (parameter definitions), and `Kconfig` (build-time enable/config). Most modules run as their own task or on the work queue and are independently enabled/disabled per board config.
- **Drivers** (`src/drivers/`) and shared **libraries** (`src/lib/`) follow the same per-directory CMake/Kconfig/params.yaml pattern as modules.
- **Platform abstraction** (`platforms/{nuttx,posix,qurt,ros2}/`) isolates OS-specific code so the same module code runs on real flight controllers (NuttX), SITL (POSIX), QuRT, and as ROS 2 nodes.
- **Board configs** (`boards/<vendor>/<model>/`) select which modules/drivers/libs are compiled in via Kconfig, per physical board or SITL variant (`boards/px4/sitl/*.px4board`).

## Commits & PRs

- **Commits:** use the `/commit` skill. Conventional commit format with
  topic-based scope: `type(scope): description`. Scope is derived from the
  directory of the changed files (e.g. `src/modules/ekf2/` → `ekf2`,
  `src/drivers/imu/icm42688p/` → `drivers/icm42688p`).
- **Pull requests:** use the `/pr` skill. PR titles follow the same
  `type(scope): description` format — CI enforces it, and it becomes the
  squash-merge commit message.
- **No Claude attribution** — no `Co-Authored-By: Claude`, no "Generated
  with Claude Code" footer. An AI tool is never an author/co-author and
  never appears in `Signed-off-by`.
- **AI disclosure is required**: every commit with AI-generated/AI-assisted
  content must carry `Assisted-by: NAME:MODEL` in the commit body (e.g.
  `Assisted-by: Claude:claude-fable-5`). This is separate from and
  additional to the human author's `Signed-off-by` (DCO sign-off via
  `git commit -s`) — sign-off must never be applied to changes the user
  hasn't reviewed.
- **Style:** run `make format` on changed C/C++ before committing; CI
  enforces `make check_format`.
- **Testing is mandatory, not optional**: new features need unit and/or
  integration tests where practical; bug fixes need a regression test or,
  when that's infeasible (hardware-specific issues, race conditions), a
  flight log demonstrating the fix. Never claim testing that didn't happen.
