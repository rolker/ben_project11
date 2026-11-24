# Plan: Check and port ben_project11 on ROS 2 Rolling

## Issue

https://github.com/rolker/ben_project11/issues/30 (part of rolker/unh_marine_autonomy#415)

## Context

`ben_project11` is BEN's configuration package: 3 Python launch files, 22 ROS 1
`.launch` files (the review and checkpoint say 23; `ls launch/*.launch` counts 22),
`config/nav2_params.yaml`, `ben.yaml`, `ben_sim.yaml` and ROS 1 leftovers. No code.
It builds on Rolling today. The owner's decisions (checkpoint, 2026-11-24): **two
parts, static part now**; the ROS 1 leftovers and the `nav2_bringup` dependency
(issue 12) are **left and recorded**. Each part is its own pull request saying
`Part of #30`. Base is `rolling` (= `jazzy` = `b8a53a3`); every commit below is
cherry-picked to `jazzy` with `-x`.

**Nav2 reference used.** Underlay source `9113449e` (= `origin/main`), Jazzy from the
same clone's `origin/jazzy` (`c7046550`). Correction to the issue review: source and
install are **not** the same commit. The install was built on 2026-11-19 from
`dcffa939` (clone at 08:51, builds 12:06 and 14:45); the checkout was fast-forwarded
to `9113449e` on 2026-11-23 09:01 and not rebuilt (the 2026-11-24 colcon run selected
no package). `git diff dcffa939 9113449e` touches only ConcatenatePaths (BT node),
the GeoJSON route loader, `opennav_following` and opennav_docking's collision pose
(no parameter); none of the files read for this plan, so the readings hold for both,
and the runs below used the `dcffa939` binaries.

## Part 1 (implement now)

### 1. `error_code_names` (`nav2_params.yaml:26-29`)

**What each distro does** (read in `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`):

- **Jazzy (Nav2 1.3.14)**: reads `error_code_names` (full blackboard key names);
  default when absent `[follow_path_error_code, compute_path_error_code]`.
  `bt_navigator` is built with `automatically_declare_parameters_from_overrides(true)`
  on both branches, so an unknown `error_code_name_prefixes` in the file is declared
  and never read: **ignored, no error**.
- **Rolling (and Kilted 1.4.2, Lyrical 1.5.3)**: the `BtActionServer` constructor
  throws `parameter 'error_code_names' has been replaced by 'error_code_name_prefixes'
  and MUST be removed` whenever `has_parameter("error_code_names")`, whatever else is
  set; with auto-declare, any presence in the file triggers it. It reads
  `error_code_name_prefixes` (default: 12 stock prefixes, **no `hover`**) and looks
  up `<prefix>_error_code` and `<prefix>_error_msg`.
- So **one parameter file cannot serve both**: Rolling refuses the old key; Jazzy
  without the old key falls back to its default and drops `hover_error_code`.

**Who reads it.** The constructor runs inside `nav2_core::BehaviorTreeNavigator::on_configure`,
the base class of BEN's `marine_nav_bt_task_navigator::TaskNavigator` (unh_marine_navigation
`rolling` branch, `46b666f`); the BEN plugin does not touch these lines, so a stock
navigator plugin exercises the same code. `hover_error_code` is written by
`marine_nav_bt_task_navigator/behavior_trees/run_tasks.xml:124`
(`<Hover ... error_code_id="{hover_error_code}">`); `compute_path`/`follow_path` at
`:179`, `:191`, `:445`. That XML is the check for the `hover` prefix while the plugin
is not ported (the prefix is only a blackboard key name; nothing loads the Hover
plugin to validate it). `hover_error_msg` is never set (the Hover BT node has no
`error_msg` port); Rolling's `populateErrorCode` catches the missing key, so the
code is reported with an empty message.

**What a wrong list does.** On Rolling, the old key is a **crash**: proved below, the
process aborts in configure (with `use_respawn` it respawns and the lifecycle
manager's bring-up fails; the whole stack is down). A list missing `hover` is a
**silent loss**: a failed hover goes back in the `RunTasks` result with no error
code. Today nothing reads `RunTasks.Result.error_code` (searched mission_manager:
only `multibeam_coverage_adapter.py` writes one; mission status comes from the action
status), so the loss is of diagnostics, not of mission behaviour. Existing gap, left
alone: `run_tasks.xml:35` writes `navigate_through_poses_error_code`, in neither list.

**Proved on Rolling today** (domain 173, checked empty, `DISPLAY` unset, stock
`bt_navigator` renamed `bt_task_navigator`, the file's own `bt_task_navigator`
settings with `navigators: [navigate_to_pose]` and no BEN plugin library, then
`ros2 lifecycle set configure`; all processes stopped):

| Parameter file | Result |
|---|---|
| old key only | `[FATAL] Failed to create navigator id navigate_to_pose. Exception: parameter 'error_code_names' has been replaced by ...`, stack trace, process gone |
| both keys | same FATAL: both cannot coexist |
| new key `[compute_path, follow_path, hover]` | `Error_code parameters were set to: compute_path follow_path hover`, `Transitioning successful`, state `inactive`; `action_server_result_timeout` read back as `900.0` (declared, never used) |

Not shown before part 2: that `TaskNavigator` itself (ported to `nav2::LifecycleNode`)
and `marine_nav_behavior_tree_bt_plugins` load, and that a real hover failure reaches
the result. Jazzy is by reading, plus the CI test in step 7.

**Options** (the minimal route is a real choice here):

- **A, recommended: the file carries the Rolling key; the launch file adds the Jazzy
  key for Nav2 older than 1.4.** `nav2_params.yaml` gets
  `error_code_name_prefixes: [compute_path, follow_path, hover]`. In
  `navigation_launch.py`, a pure function `legacy_error_code_params(nav2_version)`
  returns `[{'error_code_names': ['compute_path_error_code', 'follow_path_error_code',
  'hover_error_code']}]` when the version is below 1.4, else `[]`; the version is read
  from `nav2_bt_navigator`'s installed `package.xml` (the package whose behaviour
  changed; `ROS_DISTRO` is the cheaper alternative, but says nothing about which Nav2
  is installed). Both `bt_task_navigator` nodes (plain and composed) get
  `parameters=[configured_params] + legacy`. About 15 lines, identical on both branches;
  the two lists are tied together by a test (step 6). Cost: a list in two places,
  and a version test in a launch file.
- B, two parameter files chosen by the launch file: same version test, but 340 lines
  duplicated for 4 lines of difference; they would drift. Rejected.
- C, a launch-time rewrite of the key (`RewrittenYaml key_rewrites`, present on both):
  renames the key but cannot rewrite list values (`compute_path` vs
  `compute_path_error_code`). Rejected.
- D, `rolling` and `jazzy` differ in these lines (the fix is not backported): no code,
  but breaks "same code on both branches"; every later cherry-pick touching that block
  conflicts, and Jazzy CI cannot check the Rolling file. Second choice if the owner
  dislikes a version test in a launch file.
- E, new key only, accept Jazzy's default: one file, no code, but drops hover error
  reporting on the vehicles, which run Jazzy. Rejected.

### 2. Settings Rolling ignores

| Setting | Jazzy | Rolling | Recommendation |
|---|---|---|---|
| `action_server_result_timeout: 900.0` (`:10`, `:234`) | declared by `BtActionServer` and `waypoint_follower`, **default 900.0**; sets rcl's `result_timeout`, how long a finished goal's result is kept | removed by Nav2 #5183 (`c9438b4c`, "after fixes to ROS 2", reverting #3787); nothing replaces it, rcl's default applies (`rcl_action/action_server.h`: 10 s). Proved above: the value is accepted and unused | **Remove both lines.** No change on Jazzy (equals the default) or Rolling (ignored); Nav2 dropped it from its own file. It protected long goals from having their result expired (the ROS 2 bug #5183 refers to); it is unrelated to issue 28 (planner time vs the 4 s bond timeout). Whether Rolling's 10 s is safe for multi-minute RunTasks goals is a part 2 check. Alternative: keep with a "Jazzy only" comment. |
| `behavior_server.simulate_ahead_time 2.0`, `max_rotational_vel 1.0`, `min_rotational_vel 0.4`, `rotational_acc_lim 3.2` (`:220-223`) | read on the server node (`spin.cpp`, `drive_on_heading.hpp`) | read as `<plugin>.<key>` (`spin.simulate_ahead_time` ...); top-level keys ignored | **Keep, add a comment** saying which keys each distro reads. Values equal the defaults on both, so behaviour is the same; these are vehicle tuning, and the comment stops a later edit from silently doing nothing on Rolling. Alternatives: remove (also neutral, loses the Jazzy tuning place); translate to per-plugin keys (two places to tune). |
| `docking_server.dock_backwards: false` (`:299`) | declared, default false | deprecated, still read, logs a warning every start (`parameter_handler.cpp:56-62`); replacement `<dock>.dock_direction` (default `forward`) does not exist on Jazzy | **Remove the line.** Neutral on both (false is Jazzy's default and Rolling's `forward`), removes the warning. Alternative: leave and record. |

**Key-by-key comparison, extended.** Stock plugin class names in the file
(`SimpleProgressChecker`, `SimpleGoalChecker`, `InflationLayer`, `SmacPlannerHybrid`,
`SimpleSmoother`, `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `AssistedTeleop`,
`WaitAtWaypoint`, `SimpleChargingDock`) are each in the plugin XML of both branches;
the composable node names in `navigation_launch.py` are each registered by
`RCLCPP_COMPONENTS_REGISTER_NODE` on both; every executable named exists in the
underlay install. **Parameter types** are checked only for `bt_task_navigator` (run
above). For the other nodes the plan adds a by-hand step (step 9): configure each
stock node alone with its block. How complete: names of keys, plugins and components
fully; types for the stock nodes after step 9; value semantics, topics/QoS and
anything inside BEN plugins not at all until part 2.

### 3. Cause 11: what the files name, against `package.xml`

Today declared: `nav2_bringup` (depend), `marine_autonomy`, `manda_coverage` (exec).
Found by reading (AST of the three Python files; `plugin:` and `plugin_lib_names`
in the live YAML), all `exec_depend`:

- Project source packages: `ben_description`, `mru_transform`, `s57_grids`
  (launch); `s57_layer`, `marine_nav_bt_task_navigator`,
  `marine_nav_crabbing_path_follower`, `marine_nav_behaviors`,
  `marine_nav_behavior_tree` (parameters).
- Nav2: `nav2_common` (imported at load), `nav2_controller`, `nav2_smoother`,
  `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_waypoint_follower`,
  `nav2_velocity_smoother`, `nav2_collision_monitor`, `opennav_docking`,
  `nav2_lifecycle_manager` (launch); `nav2_costmap_2d`, `nav2_smac_planner` (parameters).
- Core: `rclcpp_components`, `launch`, `launch_ros`, `ament_index_python`.
- Build: `ament_cmake` (`find_package` in CMake; only `ament_cmake_ros` is declared).

**Declare all of them now.** The Nav2 ones add nothing to what is installed while
`nav2_bringup` stays (it pulls `navigation2`), and make issue 12 a one-line removal
later. Alternative: only the project packages plus an allow-list in the test for what
`nav2_bringup` covers. **The unreleased ones** resolve to nothing on either distro
(`rosdep resolve`: no rule for any of the 10 project packages, Jazzy or Rolling; all
Nav2 keys resolve on Jazzy, none on Rolling). Locally `rosdep install --ignore-src`
treats them as source when the workspaces holding them are given or sourced; in Jazzy
CI they need `--skip-keys` (step 8). `marine_autonomy` and `manda_coverage` already
need that today. Declaring them now rather than in part 2: the package cannot run
without them on either distro now, and the step 6 check needs no exception list.

**Carried-over items.** 1 (`ben_description` undeclared): fixed here. 2 (three ROS 1
files include the missing `load.launch`): left, recorded. 3 (`publish_state_launch.py`
ignores `namespace`): rolker/ben_description#19; nothing to do here until it lands.

### 4. Other #415 causes

1, 2, 3, 6, 7, 8, 10, 12, 13, 14, 16: **none** (no C++, no CMake target, no GDAL).
4, 5: none (no `setup.py`, no rclpy logger). 9: none today (order
`buildtool_depend, depend, exec_depend, export`); the new `test_depend`s go before
`<export>`, checked with `ament_xmllint`. 15: none (new test module name shadows
nothing). 11: above.

### Approach (commits in order, each one fix, each cherry-picked to `jazzy`)

Nothing under `.agent/` goes into any of these commits.

1. **`package.xml`: declare what the launch and parameter files use** (section 3).
2. **`nav2_params.yaml` + `navigation_launch.py`: `error_code_name_prefixes`, with
   `error_code_names` added at launch for Nav2 older than 1.4** (option A). On
   Rolling, re-run the bt_navigator check above with the committed block.
3. **`nav2_params.yaml`: remove `action_server_result_timeout`** (both lines).
4. **`nav2_params.yaml`: remove `docking_server.dock_backwards`.**
5. **`nav2_params.yaml`: comment the `behavior_server` top-level settings** (no
   value change).
6. **Static test** `test/test_launch_and_config.py`, run by
   `ament_add_pytest_test` against the install (as rolker/ben_description does:
   `ENV BEN_PROJECT11_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}`), plus `test_depend`
   `ament_cmake_pytest`, `python3-yaml`. Checks:
   - *Launch files load*: each `launch/*.py` (the live set; the ROS 1 files are `.launch`
     XML and so outside it by extension) loads with
     `get_launch_description_from_python_launch_file`, and its top-level
     `DeclareLaunchArgument` names equal a list in the test (ben_core: namespace,
     enable_bridge, tf_prefix, base_frame, map_frame, odom_frame, is_simulator,
     launch_robot_state_publisher, params_file; nav2_bringup: namespace,
     use_namespace, use_sim_time, params_file, autostart, use_composition,
     use_respawn, log_level; navigation: namespace, use_sim_time, params_file,
     autostart, use_composition, container_name, use_respawn, log_level). Needs
     `nav2_common` and `nav2_bringup` installed (`navigation_launch.py` calls
     `get_package_share_directory('nav2_bringup')` at load).
   - *Live YAML parses*: the live YAML set is every `.yaml` string constant in the
     three Python files, asserted equal to `{nav2_params, ben, ben_sim}`; each parses
     with `yaml.safe_load` and every node block reaches a `ros__parameters` mapping.
   - *Every named package declared*: names from the AST (`package=` keywords,
     `FindPackageShare(...)`, `get_package_share_directory(...)`, top-level imports
     other than the standard library) and from the YAML (`plugin:` namespace before
     `::`, `plugin_lib_names` through a small table:
     `marine_nav_behavior_tree_bt_plugins` -> `marine_nav_behavior_tree`; an unknown
     library fails and asks for a table entry) must be in `package.xml`. AST, not a
     regex, so the commented-out `project11_navigation` at `navigation_launch.py:209`
     is not counted.
   - *Error-code guard*: no live YAML contains `error_code_names`;
     `bt_task_navigator.error_code_name_prefixes` contains `hover`;
     `legacy_error_code_params('1.3.14')` equals
     `[p + '_error_code' for p in prefixes]` and `legacy_error_code_params('1.4.0')`
     and `('1.5.0')` are empty.
   - *Proof of teeth*, on a scratch copy built outside the source tree: remove
     `ben_description` from `package.xml` (manifest check fails naming it); break an
     indent in `nav2_params.yaml` (parse fails); delete one `DeclareLaunchArgument`
     (argument list fails); add a bad import (load fails); put `error_code_names`
     back (guard fails); make the helper return the old key for 1.5 (guard fails).
     Record each in the pull request.
7. **Navigator configure test** `test/test_bt_navigator_params.py`, run with
   `ament_add_ros_isolated_pytest_test` (from `ament_cmake_ros`, which stays the
   `buildtool_depend` and so stops being a mismatch; `test_depend` `rclpy`,
   `lifecycle_msgs`): builds the `bt_task_navigator` block from the installed
   `nav2_params.yaml` with the stock `navigate_to_pose` navigator and no BEN library,
   adds `legacy_error_code_params(<installed version>)`, starts stock `bt_navigator`,
   configures it through `lifecycle_msgs/srv/ChangeState`, and asserts the transition
   succeeds and the logged list contains `hover`. This is the only evidence for the
   Jazzy side of step 2 before a vehicle runs it. Prove it by putting the old key
   back (fails on Rolling). Alternative: skip it and rely on reading for Jazzy.
8. **CI** `.github/workflows/ci.yml`, one file carried by both branches, modelled on
   ben_description's: job `jazzy` on `ros:jazzy-ros-core`, `ros-dev-tools`, strict
   `rosdep install --ignore-src -y --from-paths <colcon list --packages-up-to
   ben_project11>` with `--skip-keys` for exactly the 10 project source packages
   (`ben_description marine_autonomy manda_coverage mru_transform s57_grids
   s57_layer marine_nav_bt_task_navigator marine_nav_crabbing_path_follower
   marine_nav_behaviors marine_nav_behavior_tree`: rolker repositories, not in
   rosdistro), build, `colcon test --return-code-on-test-failure`, `test-result`.
   Nav2 comes from released Jazzy binaries. **No Rolling job**, and the header says
   why: two of the three launch files need `nav2_common`/`nav2_bringup` to load, and
   step 7 needs `nav2_bt_navigator`; Nav2 has no Rolling release and no CI job builds
   Nav2 source. A Rolling job could run only the YAML and manifest checks (pure
   Python, the same on both distros) with Nav2 keys skipped; it would add little,
   so not recommended. Estimated run time 6 to 10 min, most of it installing
   `nav2_bringup`'s closure (Gazebo, slam_toolbox, TurtleBot sims); issue 12 would
   shrink it.
9. **By-hand Rolling checks, recorded in the pull request**: build; `colcon test`;
   `--show-args` for the three files; `ament_xmllint`; and each stock node
   (`controller_server`, `planner_server`, `smoother_server`, `behavior_server`,
   `waypoint_follower`, `velocity_smoother`, `collision_monitor`, `opennav_docking`)
   started alone on a free domain above 150 with its block, BEN plugins removed
   (and `chart_layer` removed from the costmaps), and configured: catches type
   mismatches and parameter-handling throws. Cannot show: BEN plugins, TF-dependent
   activation, behaviour. Jazzy is verified by CI only (no Jazzy here, no Docker);
   the first Jazzy run is expected to show: rosdep resolves every Nav2 key and the
   skip-keys cover the rest; the build; all of step 6; step 7 configuring with
   `Error_code parameters were set to: compute_path_error_code follow_path_error_code hover_error_code`.

## Part 2 (outline, after its preconditions)

**Preconditions**: unh_marine_navigation's four packages named here ported to Rolling
(`marine_nav_behavior_tree`, `marine_nav_bt_task_navigator`, `marine_nav_behaviors`,
`marine_nav_crabbing_path_follower`; the six-package port also brings
`marine_nav_ca_safety`, `marine_nav_avoidance_controller`); rolker/s57_tools part 2
(`s57_grids`, `s57_layer`, rolker/s57_tools#52); the simulator pieces the simulated
vehicle launches with (unh_marine_simulation's ports; which launch file wraps
`ben_core_launch.py` with `is_simulator:=true` to be confirmed then); the underlay
install rebuilt to its source commit.

**Checks**: the simulated vehicle launches on Rolling; every plugin loads (`TaskNavigator`,
the BT library, `CrabbingPathFollower`, `Hover`, `S57Layer`); `lifecycle_manager_navigation`
brings all 10 nodes to active; `bt_task_navigator` logs the three prefixes; a RunTasks
goal is accepted and its result reaches mission_manager after it runs longer than
10 s; no deprecation warning from the file.

**Readings it confirms or overturns**: the `error_code` fix with the real navigator;
that removing `action_server_result_timeout` is harmless on Rolling (rcl's 10 s); the
per-plugin behaviour defaults; `ben_sim.yaml`'s costmap blocks reaching the costmaps
(see left alone); whether issue 28's bond timeout shows on Rolling.

## Left alone (for a follow-up issue)

- **ROS 1 launch files (22)**, by group: perception/sensors (`asv_perception`,
  `get_remote_radar`, `get_sensors`, `get_yolo_detects`, `test_radar_target_tracker`,
  `view_pano_thumbnails`); logging/replay (`log_lidar`, `log_sensors`, `log_temps`,
  `log_throttled_panos`, `replay_bag`); ROS 1 navigation (`costmap`, `costmap_chart`,
  `costmap_radar`, `move_base`, `navigator`); operator (`operator_mystique`,
  `operator_rqt`, `operator_ui`, `remote_operator`); simulation (`sim_ben_sandisland`,
  `sim_local_cw4`). Missing files: `replay_bag`, `operator_mystique`, `remote_operator`
  include `ben_description/launch/load.launch`; `costmap_chart` loads
  `config/costmap_chart_voyage.yaml`; `sim_local_cw4` loads `parameters/ben.yaml`.
  All installed by `install(DIRECTORY launch ...)`.
- **ROS 1 config**: `costmap.yaml`, `costmap_chart.yaml`, `costmap_common.yaml`,
  `costmap_radar.yaml`, `navigator.yaml`, `navigator_ccom_planner.yaml`,
  `dubins_planner.yaml`, `path_follower.yaml`, `platform.yaml`, `navigator.rviz`,
  `operator.rviz`, `operator.perspective`, three `.desktop`; `scripts/` (3 tmux
  scripts, not installed).
- **Blocks that configure nothing**: `nav2_params.yaml` `map_saver`; `ben.yaml`
  `navigator`, `asv_helm`.
- **Local copies of Nav2's bringup files**: Nav2 Jazzy's `navigation_launch.py` now
  adds `route_server` to its node list; Nav2 main launches no lifecycle manager there
  (moved to `bringup_launch.py`) and adds `following_server`. The copy's composed
  branch also starts a stray stock `bt_navigator` and omits `manda_coverage` although
  it is in `lifecycle_nodes`, so composed bring-up would wait for a node that never
  appears (reading only; BEN launches non-composed).
- **`ben_sim.yaml` scope** (reading only): `ben_core_launch.py` applies it with
  `SetParametersFromFile` inside a scoped `GroupAction`, while Nav2 is included
  outside it, so its `local_costmap`/`global_costmap` blocks may never reach the
  costmaps. Check in part 2, then fix or file.
- `navigate_through_poses_error_code` in `run_tasks.xml` but in no list (unh_marine_navigation).
- Out of scope by the owner: field-server parameters; CAMP/mission_manager command
  handling; `nav2_bringup` (issue 12); viewers as dependencies.

## Files to Change (part 1)

| File | Change |
|---|---|
| `package.xml` | exec_depends (section 3), `ament_cmake` buildtool, test_depends |
| `config/nav2_params.yaml` | prefixes key; remove result timeout and `dock_backwards`; behaviour comment |
| `launch/navigation_launch.py` | `legacy_error_code_params` and its use on both `bt_task_navigator` nodes |
| `CMakeLists.txt` | `BUILD_TESTING` block for the two tests |
| `test/test_launch_and_config.py`, `test/test_bt_navigator_params.py` | new |
| `.github/workflows/ci.yml` | new, Jazzy job |

## Prior Art

Taken: rolker/ben_description's test-against-install and its CI shape; #415's strict
`rosdep` over the `--packages-up-to` closure; the `__has_include` idea of cause 13
(choose by what is installed), here as a Nav2 version test. Nav2 itself keeps a
separate `nav2_params.yaml` per branch (option D's shape), rejected as the default
for the same-code rule.

## Serves

None written down: the project has no goals or design document.

## Design Picture

No change to the design picture.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | ROS 1 files and `nav2_bringup` untouched; three removals are behaviour-neutral |
| Test what breaks | each check proved by a deliberate break; step 7 covers the Jazzy side |
| Verify before claiming | Rolling claims run here; Jazzy stated as CI-only |
| Name the rule before bending it | option A is a version test in a launch file; option D named as the alternative |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0013 progress entries | Yes | Plan Authored entry; records stay out of code commits |
| 0002 worktree isolation | Yes | work in the issue worktree; builds outside `src` |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `package.xml` deps | CI skip-keys | Yes |
| error-code key | launch file, test | Yes |
| ROS 1 leftovers, bringup copies | follow-up issue | No, follow-up |

## Open Questions

1. Option A (version test in `navigation_launch.py`) or D (branches differ)?
   Recommend A.
2. Declare the Nav2 packages explicitly beside `nav2_bringup`? Recommend yes (no
   install change).
3. Keep step 7 (a test that starts one stock node in Jazzy CI)? Recommend yes.
4. Remove `action_server_result_timeout` and `dock_backwards`, comment the behaviour
   settings? Recommend as tabled.

## Estimated Scope

Part 1: one pull request, 8 commits, all backported. Part 2: a second pull request later.
