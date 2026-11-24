---
issue: 30
---

# Issue #30 — Check and port ben_project11 on ROS 2 Rolling

## Issue Review
**Status**: complete
**When**: 2026-11-24 10:40 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #30

### Scope Assessment

**Well-scoped?** Partly. The blocker (manda_coverage) is gone and the "Done when" lines are clear for the first half of the work. The last clause ("launch and parameter files are checked on Rolling as far as the plan defines, with what could not be checked stated") is deliberately open, and that is the right wording, because most of what this package starts cannot run on Rolling yet (map below). The owner picks the cut (see Recommendations). Checked against the tree at `rolling` = `jazzy` tip `b8a53a3`, Nav2 source `9113449e` (2026-11-23; the underlay's `src/navigation2` is a symlinked install, so source and install are the same commit; `origin/main` in that clone is the same commit; Jazzy side is its local `origin/jazzy`).
**Right repo?** Yes.
**Serves**: none written down (the project has no goals or design document). Design picture: no change.
**Design picture?** No change. One owner-level question is raised under Recommendations (the `nav2_bringup` dependency, open issue #12).
**Dependencies**: all blockers named in the issue are met except what the first real launch needs: rolker/unh_marine_navigation (six Nav2 packages) and rolker/s57_tools part 2 (`s57_grids`, `s57_layer`, rolker/s57_tools#52). Related open issues here: #12 (is `nav2_bringup` needed), #28 (planner budget vs bond timeout, a parameter-value issue, not a port issue).

#### 1. What the package contains and whether it builds

- 3 Python launch files: `ben_core_launch.py` (192 lines), `nav2_bringup_launch.py` (181), `navigation_launch.py` (419). The last two are copies of Nav2's bringup files from before Jazzy's current form, edited for BEN.
- 23 ROS 1 XML `.launch` files (`asv_perception`, `costmap*`, `get_*`, `log_*`, `move_base`, `navigator`, `operator_*`, `remote_operator`, `replay_bag`, `sim_*`, `test_radar_target_tracker`, `view_pano_thumbnails`). They use `rosservice`, `rosbag`, `costmap_2d`, `move_base`, `nodelet`, `$(find project11)`, `project11_navigation`, `ben_gazebo`, `ben_hardware`, `ccomjhc_project11`, `camp`. Last touched 2022 to 2023. None can run on ROS 2; `replay_bag`, `operator_mystique` and `remote_operator` also include `$(find ben_description)/launch/load.launch`, which does not exist (carried-over item 2). They are installed by `install(DIRECTORY launch ...)`.
- Config: ROS 2 files `nav2_params.yaml`, `ben.yaml`, `ben_sim.yaml`, `ben.rviz` (uses `rviz_default_plugins`). ROS 1 leftovers: `costmap.yaml`, `costmap_chart.yaml`, `costmap_common.yaml`, `costmap_radar.yaml`, `navigator.yaml`, `navigator_ccom_planner.yaml`, `dubins_planner.yaml`, `path_follower.yaml`, `platform.yaml`, `navigator.rviz` and `operator.rviz` (`rviz/...` classes), `operator.perspective` (rqt), three `.desktop` files. `scripts/` holds three tmux scripts using `roscore`/`rosmon`. One ROS 1 launch file loads `config/costmap_chart_voyage.yaml` and another `parameters/ben.yaml`; neither exists.
- No behaviour-tree XML in this package. The behaviour tree is Nav2's default plus the plugin library named in `nav2_params.yaml:23` (`marine_nav_behavior_tree_bt_plugins`).
- `CMakeLists.txt` installs `launch/` and `config/` only (not `scripts/`), finds `ament_cmake`, no `ament_target_dependencies`, no code. `package.xml` has `buildtool_depend ament_cmake_ros` while CMake uses `ament_cmake` (small mismatch).
- **Build on Rolling today**: `./build.sh --parallel-workers 4 --packages-up-to ben_project11` gives `Finished <<< ben_project11 [1.40s]`, `Summary: 1 package finished`; the only stderr is the unused `CMAKE_EXPORT_COMPILE_COMMANDS` warning. `marine_autonomy` and `manda_coverage` are in the core install, `nav2_bringup` is in the underlay install (all 14 launch files plus params), so the "up to" build is just this package. `ament_xmllint package.xml`: valid. `ros2 launch ben_project11 {ben_core,nav2_bringup,navigation}_launch.py --show-args` all print their arguments without error.

#### 2. Dependency map

Launch files (Python):

| What is started or included | Source | State on Rolling |
|---|---|---|
| `marine_autonomy/robot_core_launch.py` (udp_bridge, command_bridge, helm_manager, mission_manager, platform_sender) | unh_marine_autonomy | ported, built |
| `ben_description/publish_state_launch.py` | ben_description | ported, built (not in `package.xml`; passes no `namespace`, rolker/ben_description#19) |
| `mru_transform` node, `sea_surface_estimator_launch.py`, `chart_datum_launch.py` | mru_transform | ported, built |
| `s57_grids/s57_grids_launch.py` | s57_tools | not ported (rolker/s57_tools#52, part 2) |
| Nav2: controller_server, smoother_server, planner_server, behavior_server, bt_navigator (as `bt_task_navigator`), waypoint_follower, velocity_smoother, collision_monitor, opennav_docking, lifecycle_manager, rclcpp_components container | Nav2 | all present in the underlay install |
| `manda_coverage_action_server` | manda_coverage | ported, built |

Parameter files:

| File | Configures | Plugins named | State |
|---|---|---|---|
| `nav2_params.yaml` `bt_task_navigator` | Nav2 `bt_navigator` executable | `marine_nav_bt_task_navigator::TaskNavigator`, BT library `marine_nav_behavior_tree_bt_plugins` | plugins not ported (unh_marine_navigation) |
| `controller_server` | Nav2 | `nav2_controller::SimpleProgressChecker`, `SimpleGoalChecker` (Nav2, present); `marine_nav_crabbing_path_follower::CrabbingPathFollower` | follower not ported (unh_marine_navigation) |
| `local_costmap`, `global_costmap` | Nav2 costmap_2d | `nav2_costmap_2d::InflationLayer` (Nav2); `s57_layer::S57Layer` | S57Layer not ported (s57_tools) |
| `planner_server` | Nav2 | `nav2_smac_planner::SmacPlannerHybrid` | Nav2, present |
| `smoother_server`, `waypoint_follower`, `velocity_smoother`, `collision_monitor`, `docking_server` | Nav2 | `SimpleSmoother`, `WaitAtWaypoint`, `opennav_docking::SimpleChargingDock` | Nav2, present |
| `behavior_server` | Nav2 | Nav2 Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop; `marine_nav_behaviors::Hover` | Hover not ported (unh_marine_navigation) |
| `manda_coverage` | manda_coverage | none | ported |
| `map_saver` | Nav2 map_server | none | no node of that name is launched; dead block |
| `ben.yaml` / `ben_sim.yaml` | `platform_sender`, `mru_transform`, `helm_manager`, `chart_datum`, `s57_grids`, `udp_bridge`, costmap `chart_layer` | none | ported except `s57_grids`/`chart_layer` (s57_tools). Blocks for `navigator` and `asv_helm` have no ROS 2 node in the tree (ROS 1 leftovers or hardware package) |

So on Rolling today the **Nav2 side with stock plugins** can be checked by reading (below) and, in principle, by starting individual stock Nav2 nodes with the file; the **BEN-specific plugins** (task navigator, crabbing follower, Hover, BT plugin library, S57Layer, s57_grids) cannot load until unh_marine_navigation and s57_tools part 2 are ported. The first real launch of the simulated vehicle needs both.

#### 3. Manifest and the other checks

- Cause 11: `package.xml` declares `nav2_bringup` (depend), `marine_autonomy`, `manda_coverage` (exec). Launch/parameter files also name and need: `ben_description` (carried-over item 1), `mru_transform`, `s57_grids`, `s57_layer`, `marine_nav_bt_task_navigator`, `marine_nav_crabbing_path_follower`, `marine_nav_behaviors`, `marine_nav_behavior_tree`, plus Nav2 packages `nav2_controller`, `nav2_smoother`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_waypoint_follower`, `nav2_velocity_smoother`, `nav2_collision_monitor`, `nav2_lifecycle_manager`, `nav2_smac_planner`, `opennav_docking`, `nav2_common`, `nav2_costmap_2d`, and `rclcpp_components`, `launch_ros`, `ament_index_python`. The Nav2 ones are transitively covered while `nav2_bringup` stays.
- `nav2_bringup` is only used as the default `params_file` path in `navigation_launch.py:97` (always overridden) and for its transitive packages. Open issue #12 asks whether to replace it with specific Nav2 packages (it pulls Gazebo and TurtleBot simulation, ~900 MB, onto boat computers). The port is the natural moment, but it is an owner choice.
- Cause 9: `ament_xmllint` clean. No `ament_target_dependencies`, no other CMake cause.
- Carried-over item 3 (`publish_state_launch.py` ignores `namespace`): tracked in rolker/ben_description#19; nothing to change here until that lands, then check `ben_core_launch.py` passes it.

#### 4. Nav2 parameter and plugin drift (Jazzy to Rolling), by reading

Method: Nav2 `origin/jazzy` vs `origin/main` (= underlay checkout), `nav2_bringup/params/nav2_params.yaml` diff, `declare_*parameter*` strings per package, and every key in `nav2_params.yaml` looked up in the Nav2 package sources of both branches. This covers the keys in the file and the stock plugin names; it does not cover value semantics, QoS/topic remapping effects, or anything inside the BEN plugins. Plugin class names of the stock plugins were not diffed in the plugin XML (not found missing by name lookup).

| File:line | Name | Jazzy | Rolling (Nav2 main) | Effect |
|---|---|---|---|---|
| `nav2_params.yaml:26-29` | `bt_task_navigator.error_code_names` | accepted | **configure throws**: `'error_code_names' has been replaced by 'error_code_name_prefixes' and MUST be removed` (`nav2_behavior_tree/bt_action_server_impl.hpp:80`) | breaks the navigator on Rolling. Replacement `error_code_name_prefixes` holds prefixes (`compute_path`, `follow_path`, ...), and `hover` for the custom `hover_error_code`; needs a form that still works on Jazzy (a Jazzy node ignores the new key; the old key must go on the Jazzy side). This is the one hard break found. |
| `nav2_params.yaml:10` and `:234` | `action_server_result_timeout` | declared | no longer declared | silently ignored on Rolling (undeclared override); the 900 s value no longer applies. Needs a decision whether Rolling's behaviour is acceptable. |
| `nav2_params.yaml:216-223` | `behavior_server.simulate_ahead_time`, `max_rotational_vel`, `min_rotational_vel`, `rotational_acc_lim` | read on the server node | read per plugin (`spin.simulate_ahead_time`, `spin.max_rotational_vel`, ...) | top-level values ignored on Rolling; the defaults equal the values given (2.0, 1.0, 0.4, 3.2), so behaviour is unchanged unless values are changed later. |
| `nav2_params.yaml:299` | `docking_server.dock_backwards` | read | deprecated, still read, logs a warning; replacement is `dock_direction` under the dock plugin | works, warning only. |
| `navigation_launch.py:46-57` (local copy) | lifecycle list and manager | n/a | Nav2's own `navigation_launch.py` no longer launches a lifecycle manager (moved to `bringup_launch.py`), and adds `following_server` | no impact: this package carries its own copy and its own manager. The copy is the thing that can drift further; its use of `ReplaceString`, `RewrittenYaml`, `ParameterFile` imports loads fine on Rolling. |

New on Rolling and not set here (all have defaults, so silent): `controller_server.path_handler_plugins`/`PathHandler` (FeasiblePathHandler default), `odom_topic`/`odom_duration` on the controller, `speed_limit_topic`, `transform_staleness_threshold` (several nodes), `bt_navigator.filter_duration`/`default_cancel_timeout`/`bt_search_directories`, `planner_server.allow_partial_planning`, `collision_monitor.enabled`, `SimpleGoalChecker.path_length_tolerance`. Note for the plan: `SmacPlannerHybrid`, `SimpleSmoother` (new `enforce_path_inversion`, `refinement_num`), `Spin`/`BackUp` keys listed in the file were all found by name on both branches. Removed on Rolling and unused here: `stamp_smoothed_velocity_with_smoothing_time`. `enable_stamped_cmd_vel: true` is set explicitly everywhere (default differs, `false` on Jazzy and `true` on main), good. Nothing named in `ben.yaml` or `ben_sim.yaml` is a Nav2 node except the costmap blocks in `ben_sim.yaml` (`tide_invalidate_threshold` belongs to `S57Layer`).

#### 5. Cut

See Recommendations.

#### 6. A Jazzy CI job

A job that builds `ben_project11` alone needs only `nav2_bringup` (a Jazzy binary release) plus rosdep skip-keys for the unreleased runtime packages (`marine_autonomy`, `manda_coverage`, `ben_description`, ...), since nothing is compiled. A job that also launches the stack would have to clone and build rolker/unh_marine_autonomy (and its marine_* chain), rolker/manda_coverage, rolker/unh_marine_navigation, rolker/s57_tools, rolker/mru_transform, rolker/ben_description (plus the underlay `udp_bridge` and friends); the heavy part is the marine chain and S57/GDAL, and no CI job starts a vehicle. Without cloning anything it can usefully check: `ament_xmllint`, `colcon test` linters, Python launch-file import and `ros2 launch ... --show-args` for the three Python files (needs only `nav2_common` and `launch_ros`), YAML parse of every installed parameter file, and a script that checks every `package=` and `FindPackageShare` name in the launch and parameter files against `package.xml` (the cause-11 check, which would have caught carried-over item 1). A key-by-key check of `nav2_params.yaml` against Nav2's declared parameters needs the stock Nav2 nodes' sources or running them, so it stays a by-hand step on Rolling.

### Recommendations

- **Owner decision, the cut**: do a first part now (manifest fixes, the one hard break `error_code_names` in a form that works on both distros, decide the `action_server_result_timeout` and per-plugin behaviour values, remove or document the ROS 1 leftovers, a Jazzy CI job with the static checks above, `rolling`/`jazzy` workflow), and a second part when unh_marine_navigation and s57_tools part 2 are ported (first launch of the simulated vehicle, checking `bt_task_navigator`, `FollowPath`, `hover` and `chart_layer` actually load). Keep this issue open for the second part or split it; the owner chooses. Waiting for everything would leave a known launch-time break (`error_code_names`) unfixed.
- **Owner decision**: the 23 ROS 1 `.launch` files, ROS 1 configs, rqt/rviz leftovers and tmux scripts are about 80 percent of the package by file count and cannot run on ROS 2. A port that makes no big changes should leave them, but `install(DIRECTORY launch ...)` installs them and three include a missing file. Choose between: leave and record them in the follow-up issue (default for a port), or remove the broken three. Deleting or porting them is a large change and not part of this issue.
- **Owner decision, issue #12**: replace `nav2_bringup` with specific Nav2 `exec_depend`s (it is only used for transitive packages and a default path). It changes what the package depends on, so ask before doing it here; otherwise leave it and answer #12 as follow-up.
- Planner: declare in `package.xml` the packages the launch files name directly (carried-over item 1 `ben_description`, plus `mru_transform`, `s57_grids`; the BEN Nav2 plugin packages as exec depends once the plan decides the Jazzy/Rolling skip-keys), and fix `buildtool_depend` `ament_cmake_ros` to match `ament_cmake` only if the plan touches it.
- Planner: fix `error_code_names` first (`nav2_params.yaml:26-29`) with a test on Rolling that `bt_task_navigator` gets past parameter handling (needs the navigator plugin, so by reading until unh_marine_navigation is ported), and write it so it still works on Jazzy.
- Planner: record, not fix, under the follow-up issue: `map_saver` block with no node, `ben.yaml` `navigator`/`asv_helm` blocks, and the local copies of Nav2's two bringup files. Planner: out of scope as stated by the owner: the unreviewed field-server parameters (stall monitor, end-of-line steering, goal-checker selection) and command handling between CAMP and `mission_manager`.
- Planner: the plan should state, per file, how it is checked on Rolling now (build, `--show-args`, YAML parse, name lookup against Nav2 `main`) and what is left for the first whole-vehicle launch; the table in section 4 is a starting point and the reading is not exhaustive for value semantics.

### Actions
- [ ] **Owner decision, the cut**: do a first part now (manifest fixes, the one hard break `error_code_names` in a form that works on both distros, decide the `action_server_result_timeout` and per-plugin behaviour values, remove or document the ROS 1 leftovers, a Jazzy CI job with the static checks above, `rolling`/`jazzy` workflow), and a second part when unh_marine_navigation and s57_tools part 2 are ported (first launch of the simulated vehicle, checking `bt_task_navigator`, `FollowPath`, `hover` and `chart_layer` actually load). Keep this issue open for the second part or split it; the owner chooses. Waiting for everything would leave a known launch-time break (`error_code_names`) unfixed.
- [ ] **Owner decision**: the 23 ROS 1 `.launch` files, ROS 1 configs, rqt/rviz leftovers and tmux scripts are about 80 percent of the package by file count and cannot run on ROS 2. A port that makes no big changes should leave them, but `install(DIRECTORY launch ...)` installs them and three include a missing file. Choose between: leave and record them in the follow-up issue (default for a port), or remove the broken three. Deleting or porting them is a large change and not part of this issue.
- [ ] **Owner decision, issue #12**: replace `nav2_bringup` with specific Nav2 `exec_depend`s (it is only used for transitive packages and a default path). It changes what the package depends on, so ask before doing it here; otherwise leave it and answer #12 as follow-up.
- [ ] Planner: declare in `package.xml` the packages the launch files name directly (carried-over item 1 `ben_description`, plus `mru_transform`, `s57_grids`; the BEN Nav2 plugin packages as exec depends once the plan decides the Jazzy/Rolling skip-keys), and fix `buildtool_depend` `ament_cmake_ros` to match `ament_cmake` only if the plan touches it.
- [ ] Planner: fix `error_code_names` first (`nav2_params.yaml:26-29`) with a test on Rolling that `bt_task_navigator` gets past parameter handling (needs the navigator plugin, so by reading until unh_marine_navigation is ported), and write it so it still works on Jazzy.
- [ ] Planner: record, not fix, under the follow-up issue: `map_saver` block with no node, `ben.yaml` `navigator`/`asv_helm` blocks, and the local copies of Nav2's two bringup files. Planner: out of scope as stated by the owner: the unreviewed field-server parameters (stall monitor, end-of-line steering, goal-checker selection) and command handling between CAMP and `mission_manager`.
- [ ] Planner: the plan should state, per file, how it is checked on Rolling now (build, `--show-args`, YAML parse, name lookup against Nav2 `main`) and what is left for the first whole-vehicle launch; the table in section 4 is a starting point and the reading is not exhaustive for value semantics.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 10:38 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: issue-actions
**Decision**: proceed

The issue review recommended a cut and raised two matters beyond a port; the owner answered both on 2026-11-24. **The cut: "Two parts, static part now (Recommended)".** Part 1, now: manifest fixes (cause 11: `ben_description`, `mru_transform`, `s57_grids`, the BEN Nav2 plugin packages, and whatever else the launch and parameter files name); the one hard break found by reading Nav2's source, `bt_task_navigator.error_code_names` at `config/nav2_params.yaml:26-29`, which Rolling's Nav2 refuses at configure in favour of `error_code_name_prefixes`, written so the file works on both distros; what to do about the settings Rolling silently ignores (`action_server_result_timeout` at `:10` and `:234`, 900 s; the top-level `behavior_server` rotational settings at `:220-223`; the deprecated `docking_server.dock_backwards` at `:299`); and a Jazzy CI job doing static checks (manifest, Python launch files load, parameter files parse, every package named by a launch or parameter file is declared). Part 2, after the six Nav2 navigation packages and rolker/s57_tools part 2 are ported: the first launch of the simulated vehicle on Rolling. Each part is its own pull request saying `Part of #30`. **The leftovers: "Leave both, record them (Recommended)".** The 23 ROS 1 `.launch` files with their ROS 1 configuration, rviz and script files are not touched and are listed for a follow-up issue; the `nav2_bringup` dependency stays as it is (this repository's issue 12). For the planner: the plan covers both parts and details part 1; nothing is launched in part 1 beyond what a single stock Nav2 node needs to accept its parameters, if the plan chooses to check that way; no GUI program is started.

## Plan Authored
**Status**: complete
**When**: 2026-11-24 10:49 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Plan**: `.agent/work-plans/issue-30/plan.md` at `bccb7b8`

Part 1 (now, 8 commits, all backported): declare every package the launch and parameter files name; replace `error_code_names` with `error_code_name_prefixes` [compute_path, follow_path, hover] and have `navigation_launch.py` add the old key for Nav2 older than 1.4 (Rolling throws on the old key, proved by configuring stock `bt_navigator` on domain 173; Jazzy ignores the new one); remove `action_server_result_timeout` and `dock_backwards` (neutral on both), comment the behaviour settings; a static test, a bt_navigator configure test and a Jazzy CI job with 10 skip-keys, no Rolling job. Part 2 outlined (first simulated launch after unh_marine_navigation and s57_tools part 2). Corrects the review: the underlay install is `dcffa939`, source `9113449e`; the difference touches none of the files read.

## Plan Review
**Status**: complete
**When**: 2026-11-24 10:58 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #30 — Check and port ben_project11 on ROS 2 Rolling
**Plan**: `.agent/work-plans/issue-30/plan.md` at `bccb7b8`
**Branch**: `feature/issue-30`

Reviewed Part 1 (static part) against Nav2 source in the underlay clone (`origin/jazzy` `c7046550`, `origin/kilted`, `origin/lyrical`, `origin/main` `9113449e`), unh_marine_navigation `origin/jazzy`, mission_manager on `rolling`, launch_ros and rosdep as installed here, and ben_description's `ci.yml` on `rolling`. Nothing was launched; one `rosdep install --simulate` was run on a scratch copy of the planned manifest.

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One PR, 8 commits, ~7 files; Part 2 outlined with preconditions. Matches the owner's cut. |
| Issue alignment | Good | Covers every Part 1 item of the checkpoint and the issue review's planner actions (manifest, `error_code_names`, ignored settings, Jazzy CI, per-file checking statement). Corrects the review's 23→22 count and its source/install claim. **Serves**: "None written down: the project has no goals or design document." (as checked; not flagged). |
| File targeting | Good | `package.xml`, `nav2_params.yaml`, `navigation_launch.py`, `CMakeLists.txt`, two tests, `ci.yml`. `nav2_bringup_launch.py`/`ben_core_launch.py` need no change (BEN passes `use_composition: 'False'`, `ben_core_launch.py`). |
| Consequences | Needs work | The launch-file wiring of the legacy list (what actually reaches a Jazzy boat) has no check; see finding 1. |
| Principle alignment | Good | Only what's needed (three removals are default-equal on Jazzy, verified below); test what breaks (each check proved by a break); verify before claiming (Jazzy stated as CI-only). |
| ADR compliance | Good | ADR-0013 (records out of code commits), ADR-0002 (worktree, builds outside `src`). No other ADR triggered. |
| ROS conventions | Good | Version-gated parameter in a launch file is unusual but bounded; dict parameters after a `ParameterFile` are a normal launch_ros pattern (later sources override; keys do not overlap here). |

### Verification of the plan's claims

- **Error codes, Jazzy** (`origin/jazzy` `bt_action_server_impl.hpp:77-105,179,364-388`): default `[follow_path_error_code, compute_path_error_code]`; `error_code_names` read at configure into `error_code_names_`. `bt_navigator.cpp:35-37` uses `automatically_declare_parameters_from_overrides(true)`, so `error_code_name_prefixes` in the file is declared and never read. BEN's `TaskNavigator` (`task_navigator.h:13-14`, jazzy) derives from `nav2_core::BehaviorTreeNavigator`, which builds the same `BtActionServer` (`behavior_tree_navigator.hpp:205`), and runs inside the stock `bt_navigator` process as launched (`navigation_launch.py:226-238`). Order of the list does not matter (lowest non-zero code wins).
- **Error codes, boundary**: prefixes and the "MUST be removed" throw landed on main in `7e3ae16d` (2025-03-04, #4341/#4460) while main was 1.3.1, after Jazzy branched (not an ancestor of `origin/jazzy`; Jazzy got only the `error_msg` part, `acea1ef2`). Kilted (1.4.2), Lyrical and main all throw; Jazzy is 1.3.x. `< 1.4` is the right boundary for every released distro.
- **How the list reaches the node**: launch_ros `Node._create_params_file_from_dict` writes a dict parameter as a second `--params-file` keyed by the node's name (or `/**`), after the `RewrittenYaml` file; a list of strings becomes a string array; `LoadComposableNodes` converts dicts to typed `Parameter` messages. Not passed through `RewrittenYaml`, so its rewriting is irrelevant. Correct on paper for both paths.
- **`action_server_result_timeout`**: Jazzy default 900.0 in `BtActionServer` (`:67-68`) and `waypoint_follower.cpp:40`, the only two nodes the file sets it on (`:10`, `:234`); 900 since `0976e631` (in tag 1.3.0), so every Jazzy release. Other Jazzy nodes default 10.0 but are not set by the file. Main has no `result_timeout` anywhere in these packages; rcl default 10 s (`rcl_action/action_server.h:250`). mission_manager calls `get_result_async()` right after the goal is accepted (`mission_manager.py:207`), so expiry of a finished result does not affect it.
- **`dock_backwards`**: Jazzy declares default false (`docking_server.cpp:41`); main still reads it with a deprecation path (`parameter_handler.cpp:56`). Neutral.
- **Behaviour settings**: Jazzy declares them unprefixed per plugin (`spin.hpp`, `drive_on_heading.hpp:209`, default 2.0); main declares `<behavior>.<key>` with 2.0/1.0/0.4/3.2 (`spin.cpp:52-59`). Comment-only: neutral.
- **Manifest/rosdep**: rosdep 0.27 here searches `AMENT_PREFIX_PATH` with `--ignore-src`. Scratch manifest with the planned keys: with only the underlay sourced, `Cannot locate rosdep definition` (the same state `marine_autonomy` is already in today); with `core/src` and `platforms/src` as `--from-paths`, resolves cleanly. Vehicle Jazzy workspaces carry unh_marine_navigation and s57_tools (`core.repos`) and ben_description/mru_transform (`platforms.repos`). None of the new keys (or the six project packages' own jazzy dependencies) pulls a viewer or simulator; the Nav2 node keys are inside `navigation2`, already pulled by `nav2_bringup`.
- **Two by-reading findings** (stray `bt_navigator` and missing `manda_coverage` in the composed branch, `navigation_launch.py:340-353`; `ben_sim.yaml` applied inside a scoped `GroupAction`, `ben_core_launch.py:92-110`, with Nav2 included outside it at `:176`): both confirmed by reading. Neither reaches a vehicle (BEN is launched non-composed; `ben_sim.yaml` is simulator-only, `ben.yaml` has no costmap block). Follow-up/Part 2 is right.

### Findings

1. **[Consequences]** (medium) Step 7 composes its own parameters (installed YAML plus `legacy_error_code_params(...)`), so it proves the helper's output and Nav2's handling of it, but not the wiring in `navigation_launch.py` — that both `bt_task_navigator` descriptions (`LifecycleNode` at `:226` and `ComposableNode` at `:347`) actually carry the helper's result after `configured_params`. That wiring is the only thing that keeps `hover_error_code` on a Jazzy boat after the backport, and dropping it would pass every planned check on Jazzy CI. Add to step 6 an assertion that walks the loaded `LaunchDescription` (or the AST) and finds the helper's result in the `parameters` of both `bt_task_navigator` nodes, with a proof-of-teeth break (remove it from one node).
2. **[ROS conventions]** (medium) The plan does not say what the launch file does when the version lookup fails. Specify: `get_package_share_directory('nav2_bt_navigator')`, parse `<version>` from its `package.xml`, compare as an integer tuple (no `packaging` dependency), and raise with a clear message on any failure. Never fall back to a default: a Rolling default silently drops `hover` on Jazzy; a Jazzy default crashes on Rolling. Add a guard case to step 6 for an unparsable version.
3. **[File targeting]** (low) `nav2_bringup_launch.py:37` calls `get_package_share_directory('ben_project11')` at load time, so step 6's "launch files load" needs `ben_project11` on the ament index. ben_description's pattern does not source `install/setup.bash`; the test's `ament_add_pytest_test` needs `ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}` (or equivalent), and the manifest check must exclude the package's own name (`FindPackageShare('ben_project11')` appears four times in `ben_core_launch.py`).
4. **[Scope]** (low) Step 7 must apply the launch's `ReplaceString` substitutions (or otherwise replace the placeholders): `odom_topic: <robot_namespace>/odom` (`nav2_params.yaml:6`) is not a valid topic name, and `bt_navigator` creates its odometry subscription at configure. State in the plan how the Rolling run in the table handled it so the test does the same.
5. **[Issue alignment]** (low, information) Part 2's check "a RunTasks goal ... result reaches mission_manager after it runs longer than 10 s" will pass trivially: mission_manager requests the result as soon as the goal is accepted (`mission_manager.py:207`), and rcl's `result_timeout` only expires results after the goal finishes. The real loss on Rolling is for a late client (e.g. one asking for a result more than 10 s after completion); say so in the PR record rather than presenting the Part 2 check as covering it.

### Summary

The plan is sound and its key readings hold from source: option A leaves Jazzy's effective parameters identical (the same three `error_code_names`, 900 s result timeouts by default, `dock_backwards` false by default) and fixes the Rolling crash. Ready for implementation once the wiring check (1) and the lookup failure mode (2) are folded into steps 2 and 6.

### Recommended Actions

- [ ] Add to the static test an assertion that both `bt_task_navigator` node descriptions in `navigation_launch.py` receive `legacy_error_code_params(...)` after `configured_params`, with a deliberate-break proof.
- [ ] Specify the version lookup in `navigation_launch.py`: ament index + `package.xml` `<version>`, integer-tuple compare, raise on failure; add a guard case.
- [ ] Make the installed package visible on the ament index for the load test (`AMENT_PREFIX_PATH` in the test's `ENV`), and exclude `ben_project11` itself from the manifest check.
- [ ] In step 7, substitute `<robot_namespace>`/`<tf_prefix>` as the launch does.
- [ ] Reword Part 2's result-timeout check (mission_manager asks for the result at acceptance) and record the late-client case in the PR.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 10:55 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the plan review's verdict is ready; its findings are implementation details the implementer applies; the review confirmed that the planned backport leaves Jazzy's behaviour unchanged, so none of the planner's open questions changes what a vehicle does)
**After**: plan
**Decision**: proceed

Read: the Plan Review at `bccb7b8`, verdict ready, five findings. Part 1 is implemented now. Host decisions on the planner's open questions: **option A** for the error-code fix (one parameter file with `error_code_name_prefixes`; `navigation_launch.py` adds the old `error_code_names` list when the installed `nav2_bt_navigator` is older than 1.4), keeping both branches identical as every other port has; the alternative was to let the branches differ in those lines. The Nav2 node and plugin packages are declared explicitly beside `nav2_bringup` (nothing more is installed). The navigator configure test stays. The three settings changes go in as planned (remove `action_server_result_timeout`, whose value was already Jazzy's default; remove `dock_backwards`, false by default on both; comment the `behavior_server` settings). Review findings, applied as follows: (1) the static test also asserts that **both** `bt_task_navigator` descriptions in `navigation_launch.py` (the lifecycle node and the composable node) carry the helper's result after the configured parameters, proved by removing it from one: that wiring is the only thing keeping `hover_error_code` on a Jazzy vehicle after the backport. (2) The version lookup is specified: the share directory of `nav2_bt_navigator`, the `<version>` of its `package.xml`, compared as an integer tuple, and **an error with a clear message on any failure, never a default** (a wrong default silently drops `hover` on Jazzy or crashes on Rolling); a guard case for an unparsable version. (3) The static test puts this package's install prefix on `AMENT_PREFIX_PATH`, since a launch file looks up `ben_project11`'s own share directory at load, and the manifest check excludes the package's own name. (4) The navigator test applies the launch file's placeholder substitutions (`<robot_namespace>`) as the launch does. (5) Part 2's outline says plainly that on Rolling a finished goal's result is kept for about 10 s, which matters only to a client that asks late; `mission_manager` asks at once, so its goals are not affected; the pull request states this as a known difference on Rolling for the owner.
