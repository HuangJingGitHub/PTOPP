# PTOPP
Core code implementation for passage-traversing optimal path planning (PTOPP) with sampling-based algorithms. This repository is under construction.

> [!IMPORTANT]
> **Research preview.** The expected environment is Linux with ROS 1 and catkin. The build and run workflow below has not been verified on macOS. ROS distribution and dependency versions are not pinned, so the commands should be treated as the intended workflow rather than a guaranteed reproduction.

## Main features

- 2D and 3D polygonal/prismatic obstacle representations, collision checking, clearance/passage-traversing queries, and random environment generation.
- Passage detection using Gabriel condition from Delaunay graphs of obstacles and Gabriel cell decomposition.
- RRT* and PRM* variants with clearance, passage-width-based objectives/constraints under path-length regulation.
- OpenCV rendering for 2D experiments and ROS/RViz visualization for 3D experiments.
- Experiment drivers for passage/cell detection, planned path evaluation, planning time, sample-count scaling, and cost evolution.

## Repository structure

```text
PTOPP/
├── LICENSE
├── README.md
└── src/
    └── ptopp/
        ├── CMakeLists.txt              # Catkin build targets
        ├── package.xml                 # ROS package metadata
        └── src/
            ├── PRMStar.hpp             # 2D PRM* planner
            ├── RRTStar.hpp             # 2D RRT* planner
            ├── decomposition.hpp       # 2D passage and cell decomposition
            ├── obstacles.hpp           # 2D geometry and obstacle utilities
            ├── kd_tree.hpp             # 2D KD-tree and path nodes
            ├── primitives.hpp          # Shared planning primitives
            ├── 3d/                     # 3D planners, geometry, decomposition, and KD-tree
            ├── tests/                  # Examples, benchmarks, and utility programs
            ├── vis/                    # RViz configurations and marker publishers
            ├── data/                   # Generated experiment output (Git-ignored)
            └── img/                    # Generated figures and captures (Git-ignored)
```

The core implementation is primarily header-based. Files under `tests/` provide executable examples and research benchmarks.

## Expected dependencies

The current source and build configuration expect:

- A C++11-compatible compiler and CMake.
- ROS 1 with catkin, ROS packages referenced by the build or visualization code.
- OpenCV, Eigen3.

Exact installation commands depend on the Linux and ROS releases and are intentionally not prescribed here.

## Planning workflow

A typical use of the core planners follows these steps:

1. Define the 2D/3D configuration space, start and goal positions, and obstacles. The examples can also generate random obstacles with `GenerateRandomObstacles` or `GenerateRandomObstacles3d`.
2. Detect obstacle passages and construct Gabriel cells. The planners perform this internally using routines such as `PassageCheckDelaunayGraphWithWalls` and `ReportGabrielCells` (and their 3D counterparts).
3. Instantiate `RRTStarPlanner`/`RRTStarPlanner3d` or `PRMStarPlanner`/`PRMStarPlanner3d`, selecting the desired objective family and constraints through the constructor parameters.
4. For RRT*, call `Plan`. For PRM*, construct the roadmap and query it with `ConstructRoadmap` followed by `QueryPath`.
5. Retrieve the result with `GetPath` or `GetPathInPts`, then render it with OpenCV or publish it through the RViz visualization helpers.

The numerical cost-mode mappings are still differ between some 2D and 3D experiments. Refer to the planner constructors and the corresponding example source before selecting `cost_function_type`.

## Reference build steps

The repository already has the layout of a catkin workspace: the ROS package is located at `src/ptopp`. From a Linux system with ROS 1 and the required dependencies configured:

```bash
git clone https://github.com/HuangJingGitHub/PTOPP.git
cd PTOPP

catkin_make
source devel/setup.bash
```

Run commands from the repository root. Several experiment programs use repository-relative paths when writing optional results.

## Running the main examples

### 2D planning

The main 2D example generates a random obstacle environment and compares RRT*-based passage objectives:

```bash
cd /path/to/PTOPP
source devel/setup.bash
rosrun ptopp planning_test_2d
```

The OpenCV window accepts:

- `s` to save the current visualization under `src/ptopp/src/img/planning_img/`.
- `q` to close the example.

Saved images are local experiment artifacts and are ignored by Git.

### 3D planning with RViz

The 3D example publishes obstacles and multiple planned paths as ROS markers. Use separate terminals after building the workspace.

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
cd /path/to/PTOPP
source devel/setup.bash
rviz -d src/ptopp/src/vis/3d_planning_config_side_3.rviz
```

Terminal 3:

```bash
cd /path/to/PTOPP
source devel/setup.bash
rosrun ptopp planning_test_3d
```

The supplied RViz configuration uses `planning_frame` and subscribes to the obstacle marker array and path marker topics published by the example.

## Available CMake targets

The targets currently registered in `src/ptopp/CMakeLists.txt` are grouped below.

| Category | Targets | Purpose |
| --- | --- | --- |
| Decomposition | `decomposition_test`, `decomposition_test_3d` | Exercise 2D/3D passage detection and cell decomposition. |
| Planning examples | `planning_test_2d`, `planning_test_3d` | Compare PTOPP objectives and visualize resulting paths. |
| Passage benchmarks | `passage_detection_performance`, `passage_detection_performance_3d`, `passage_detection_geodesic_distance` | Measure passage-detection behavior and runtime. |
| Planning-time benchmarks | `planning_time_2d`, `planning_time_3d`, `planning_time_sample_2d`, `planning_time_sample_3d` | Evaluate scaling with obstacle or sample counts. |
| Cost evolution | `planning_cost_evolution_2d`, `planning_cost_evolution_3d` | Record planner cost as the sample count increases. |
| Utilities | `basic_shapes`, `function_test` | Check ROS marker publication and selected geometry functions. |

Some additional standalone programs are present in `src/ptopp/src/tests/` but are not currently registered as CMake targets.

## Generated output

Benchmark programs write text logs below `src/ptopp/src/data/`, while visualization programs may write figures below `src/ptopp/src/img/`. These directories are retained for runtime use, but their generated contents, the original analysis scripts, and historical experiment results are intentionally excluded from this open-source release.

## Current limitations and TODO

- The ROS 1 build and runtime workflow has not been validated in macOS or by continuous integration.
- The exact ROS distribution, compiler, OpenCV version, and dependency installation procedure are not pinned. Compilation options are NOT optimized for best efficiency.
- Example scenarios and planner parameters are currently configured directly in the C++ sources rather than through CLI or ROS parameters.
- Several examples use random obstacle generation, so results are not deterministic by default.

## License

This project is released under the [MIT License](LICENSE).
