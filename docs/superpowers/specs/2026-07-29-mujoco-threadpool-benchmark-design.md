# MuJoCo thread pool: opt-in integration and benchmark test

**Date:** 2026-07-29
**Repo:** gz-physics, branch `jrivero/mujoco_3_11_0_update`
**Depends on:** MuJoCo pinned at 3.11.0

## Purpose

MuJoCo 3.10.0 added `mju_threadpool(mjData*, int nthread)`. When a pool is attached to an
`mjData`, MuJoCo parallelises parts of the pipeline — collision detection, and constraint solving
across islands. The mujoco plugin does not use it today.

This adds opt-in thread-pool support to the plugin and a benchmark test that measures what it buys.

## Non-goals

- Threading for any other physics engine.
- Exposing threading through the gz-physics public API. No new feature, no `Migration.md` entry.
- Making threading the default. Unset means off, and off is byte-identical to today.

## The hazard this design is built around

`mj_makeRawData` (`engine_io.c:1113`) does:

```c
// clear threadpool
d->threadpool = 0;
d->threadlock = 0;
```

It nulls the pointer **without destroying the pool**. `mj_deleteData` cleans up correctly
(`mju_threadpool(d, 0)`), but `mj_recompile` reaches `mj_makeRawData` via `mjCModel::MakeData`,
reuses the same `mjData` struct, and simply zeroes the field.

`Base::RecompileSpec` (`mujoco/src/Base.cc:125`) is the only `mj_recompile` call site, and it runs
on every model addition. So a pool installed once at world construction would be leaked — the
`ThreadPoolContext` and its N `std::thread`s orphaned and never joined — on the first model added,
with stepping silently falling back to single-threaded.

**The pool must therefore be torn down before every recompile and reinstalled after it.**

A second, pre-existing detail constrains the error path: when `mj_recompile` fails it calls
`mj_deleteData(d)` internally, leaving `WorldInfo::mjDataObj` dangling (and `~WorldInfo` will
double-free it). The reinstall must not run on the failure path. This design does not fix that
pre-existing bug; it only avoids adding to it.

## Configuration

Environment variable `GZ_PHYSICS_MUJOCO_THREADS`.

| Value | Behaviour |
|---|---|
| unset, empty, or `0` | Threading off. `mju_threadpool` is never called. |
| `N >= 1` | Pool of N worker threads. |
| unparseable or negative | `gzwarn`, treated as off. |

Read **at each world construction**, not once per process, and cached in
`WorldInfo::threadPoolSize`. Per-world reading is what lets the benchmark sweep N with `setenv` in a
single process instead of spawning subprocesses. World construction is rare, so the cost is
irrelevant.

## Components

### `WorldInfo` (`mujoco/src/Base.hh`)

Gains one member:

```cpp
/// \brief Number of MuJoCo worker threads for this world; 0 disables threading.
int threadPoolSize{0};
```

### `SetThreadPool` (`mujoco/src/Base.hh` / `Base.cc`)

```cpp
/// \brief (Re)install or tear down the MuJoCo thread pool on a world's mjData.
/// \param[in,out] _worldInfo World whose mjData is targeted. No-op if mjData is null.
/// \param[in] _nthread Worker thread count; 0 tears the pool down.
///
/// mj_makeRawData, reached via mj_recompile, zeroes mjData::threadpool without
/// destroying the pool, so the pool must be torn down before a recompile and
/// reinstalled afterwards or the worker threads leak.
void SetThreadPool(WorldInfo &_worldInfo, int _nthread);
```

A thin wrapper over `mju_threadpool` with a null-`mjData` guard. Upstream already makes
`mju_threadpool(d, 0)` delete-and-null, and a same-size call a no-op, so the semantics map straight
through and no extra state tracking is needed.

### `ReadThreadPoolSizeFromEnv` (`mujoco/src/Base.cc`, internal linkage)

Parses the environment variable, emits the warning on bad input, returns the clamped count.

## Data flow

```
ConstructEmptyWorld (EntityManagementFeatures.cc:105)
  mj_makeData
  worldInfo->threadPoolSize = ReadThreadPoolSizeFromEnv()
  SetThreadPool(*worldInfo, worldInfo->threadPoolSize)

Base::RecompileSpec (Base.cc:125)          <- runs on every model addition
  SetThreadPool(_worldInfo, 0)             <- else the pool leaks
  rc = mj_recompile(...)
  if (rc != 0) return false                <- mjDataObj already deleted; do not reinstall
  SetThreadPool(_worldInfo, _worldInfo.threadPoolSize)

~WorldInfo
  mj_deleteData                            <- upstream tears the pool down
```

## Error handling

| Condition | Behaviour |
|---|---|
| Env var unparseable or negative | `gzwarn`, threading off. World construction proceeds. |
| `N` greater than `hardware_concurrency()` | Allowed. MuJoCo handles oversubscription; no warning, since it is legitimate for benchmarking. |
| `mjData` null when `SetThreadPool` is called | No-op. |
| `mj_recompile` fails | No reinstall. `mjDataObj` has already been deleted upstream. |

## Testing

Split by level, because the two things worth asserting live at different layers.

### `UNIT_mujoco_ThreadPool_TEST` — `mujoco/src/ThreadPool_TEST.cc`

Has access to plugin internals, so it asserts structure directly on `mjData::threadpool`:

1. Env var unset → pool is null after world construction.
2. `GZ_PHYSICS_MUJOCO_THREADS=4` → pool is non-null after world construction.
3. **Pool survives a model addition.** Construct with threading on, add a model (forcing
   `RecompileSpec`), assert the pool is still non-null. This is the direct regression test for the
   leak described above, and it fails without the bracket.
4. Unparseable value → pool is null, construction still succeeds.

Picked up automatically by `gz_get_libsources_and_unittests`, which classifies `src/*_TEST.cc` as
unit tests.

### `INTEGRATION_mujoco_ThreadPool` — `mujoco/test/ThreadPool.cc`

Drives the plugin through the public `gz::plugin::Loader` API, using the existing
`mujoco_plugin_LIB` compile definition.

**Scene:** a 20x20 grid of free boxes dropped onto a ground plane, spaced so they do not touch each
other. Roughly 400 independent constraint islands, which is what makes island-parallel solving
measurable. Small scenes are actively misleading here: with one or two islands the pool overhead
dominates and the result reads as "threading hurts", which is an artifact of the scene, not a
finding.

**Correctness gate:** step the scene with `N=0` and with `N=4`, compare link poses within tolerance
over a short horizon (~50 steps). Assert.

**Timing report:** step the full horizon (500 steps) at each N, print per-N wall time and speedup to
stdout. Never asserted — timing assertions flake on loaded or low-core CI machines.

**Skip condition:** `std::thread::hardware_concurrency() < 2` skips the threaded case via
`GTEST_SKIP()`.

**Placement:** a new `mujoco/test/` directory with its own
`gz_build_tests(TYPE INTEGRATION_mujoco ...)` block in `mujoco/CMakeLists.txt`. It cannot live in
`mujoco/src/`, where the `_TEST.cc` suffix would silently make it a unit test, and it does not
belong in `test/integration/`, which links neither MuJoCo nor the mujoco plugin and is deliberately
engine-agnostic. Everything under `mujoco/` is already gated by `SKIP_mujoco`.

### Known risk: trajectory equivalence

Threaded and single-threaded runs are not guaranteed to match. Island solving can reorder
floating-point accumulation, and 400 boxes in contact is chaotic enough to amplify that over a long
horizon.

The equivalence gate is therefore specified as short-horizon within tolerance, with the tolerance
set **empirically during implementation** by measuring actual divergence.

If divergence proves chaotic even at 50 steps, the fallback is an invariant-based gate — bodies at
rest, none fallen through the floor, total energy bounded — and that substitution will be stated
explicitly. The tolerance will not be loosened until it passes. The timing report is unaffected
either way.

## Files touched

| File | Change |
|---|---|
| `mujoco/src/Base.hh` | `WorldInfo::threadPoolSize`; `SetThreadPool` declaration |
| `mujoco/src/Base.cc` | `SetThreadPool`, `ReadThreadPoolSizeFromEnv`; bracket the recompile |
| `mujoco/src/EntityManagementFeatures.cc` | Read env var and install pool after `mj_makeData` |
| `mujoco/src/ThreadPool_TEST.cc` | New unit test |
| `mujoco/test/ThreadPool.cc` | New integration/benchmark test |
| `mujoco/CMakeLists.txt` | `INTEGRATION_mujoco` test block |
