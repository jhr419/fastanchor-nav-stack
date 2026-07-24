# PCT planner runtime

This directory stores the project-relative Python 3.10 pybind entry modules
and the required planner/GTSAM shared libraries used by the optional original
PCT backend. They were extracted from the PCT Planner runtime associated with
the source navigation system; FastPlanner does not load libraries from 3dnav.

The modules are platform-specific and use the NumPy 1.x ABI. Project-local
copies of `liba_star_search`, `libele_planner_lib`, `libgpmp_optimizer`,
`libmap_manager`, `libcommon_smoothing`, and GTSAM are included, but compiler,
Python, NumPy, TBB, and Boost ABI compatibility is still required. Never
silently fall back to a navigation workspace installation.

For a native PCT test, either replace this directory with a complete compatible
runtime or pass a planner build directory at launch time:

```text
planner_backend:=pct planner_lib_dir:=/path/to/PCT_planner/planner/lib
```

`PCT_PLANNER_LIB_DIR` is also supported. Keep machine-specific absolute paths
out of source and YAML. Rebuild/replace the runtime when changing Python,
NumPy, compiler ABI, operating system, or architecture. Tomography generation
and the default portable `astar` feasibility test do not load these binaries.
