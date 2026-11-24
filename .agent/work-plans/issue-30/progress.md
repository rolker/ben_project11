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

## Implementation
**Status**: complete
**When**: 2026-11-24 11:12 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-30 at `301536c`

Part 1 of the plan, eight commits in the plan's order, none touching `.agent/` (each cherry-picks onto `jazzy`, which is `b8a53a3` = `rolling`). Nothing was pushed or posted. **One new Rolling break found by step 9, not fixed (owner decision, below): `planner_server` refuses `GridBased.lookup_table_size: 55.0`.**

### Commits

1. `aae76a0` package.xml: declare what the launch and parameter files use
2. `c02aa8a` Give bt_task_navigator error_code_name_prefixes, and error_code_names on Nav2 < 1.4
3. `934ad6b` nav2_params.yaml: remove action_server_result_timeout
4. `2686472` nav2_params.yaml: remove docking_server.dock_backwards
5. `cd63732` nav2_params.yaml: say which distro reads the behavior_server settings
6. `9c3219d` Add a static test of the launch and parameter files
7. `4b0b4f1` Add a test that configures the stock bt_navigator with BEN's parameters
8. `301536c` Add a Jazzy CI job

### Why Jazzy's behaviour is unchanged, per commit

1. Manifest only: changes what rosdep and colcon know, not what is launched. Every Nav2 package declared is in `navigation2`, which `nav2_bringup` (kept) already pulls on Jazzy; the project packages are already in the vehicle workspaces; nothing adds a viewer or simulator. `ament_cmake` added as a `buildtool_depend` beside `ament_cmake_ros` (kept).
2. The Jazzy navigator (Nav2 1.3.x) gets `error_code_names` = `compute_path_error_code follow_path_error_code hover_error_code`, the file's list of today in its order, through a second parameter file after the first (launch_ros keys it `/ben/bt_task_navigator`, since name and namespace are given); `error_code_name_prefixes` is auto-declared and never read (no reference anywhere in Nav2 `origin/jazzy`). Logging is the same as today: with auto-declaration Jazzy skips its "Error_code parameters were set to" line in both cases.
3. `action_server_result_timeout: 900.0` was Jazzy's default for both nodes it was set on (`bt_action_server_impl.hpp:67-68`, `waypoint_follower.cpp:40` on `origin/jazzy`).
4. `dock_backwards: false` is Jazzy's declared default (`docking_server.cpp:41`).
5. Comment only.
6, 7. Tests only. 8. CI only.

### The error-code fix

- **Helper** (`launch/navigation_launch.py`): `LEGACY_ERROR_CODE_NAMES` (the old list); `nav2_version_tuple(version)` splits on `.` into ints and raises `RuntimeError("ben_project11: cannot parse the nav2_bt_navigator version ...")` for anything not at least two integers; `legacy_error_code_params(version)` returns `[{'error_code_names': [...]}]` below `(1, 4)` and `[]` otherwise; `installed_nav2_bt_navigator_version()` reads `<version>` from `get_package_share_directory('nav2_bt_navigator')/package.xml` and raises with a clear message on any failure or an empty version. No default anywhere. `bt_task_navigator_params = [configured_params] + legacy_error_code_params(installed_nav2_bt_navigator_version())` is the `parameters` of both the `LifecycleNode` and the `ComposableNode` named `bt_task_navigator`. Outputs: `1.2.9`, `1.3.0`, `1.3.14`, `1.3` give the three names; `1.4.0`, `1.4.2`, `1.5.0`, `1.5.3`, `2.0` give `[]`; `''`, `'abc'`, `'1'`, `None`, `'1.4.0rc1'` raise. Installed here: `1.5.0` gives `[]`.
- **Rolling proof** (domain 187, checked empty with `ros2 node list --no-daemon`, `DISPLAY` unset, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`): the stock `bt_navigator` started as `/ben/bt_task_navigator` with the file as committed after the launch files' own substitutions (`nav2_bringup_launch.py`'s `ReplaceString`, giving `odom_topic: /ben/odom`, `global_frame: ben/map`, then `navigation_launch.py`'s `RewrittenYaml` under `ben`), BEN navigator plugin and BT library replaced by stock `navigate_to_pose`: `Creating navigator id navigate_to_pose ...`, `Error_code parameters were set to: compute_path follow_path hover`, configure success, state `inactive`, `error_code_names` undeclared, `error_code_name_prefixes` = the three. With `error_code_names` put back in a scratch copy: `[FATAL] ... Failed to create navigator id navigate_to_pose. Exception: parameter 'error_code_names' has been replaced by  'error_code_name_prefixes' and MUST be removed.`, process exits (-11).
- **Jazzy argument** (Nav2 `origin/jazzy` `c7046550`, `nav2_bt_navigator` 1.3.14, by reading): `bt_navigator.cpp:37` auto-declares parameters from overrides, so both keys from the two files are declared at construction; `bt_action_server_impl.hpp:82` (`if (!node->has_parameter("error_code_names"))`) is then skipped and `:179` reads `error_code_names` into `error_code_names_` = the three names; `error_code_name_prefixes` occurs nowhere in the branch. Effective parameters: the same three names as today, plus one declared key nothing reads. `BtActionServer` is built by `nav2_core::BehaviorTreeNavigator` (`behavior_tree_navigator.hpp:205`), which BEN's `TaskNavigator` derives from.

### Tests

- `test/test_launch_and_config.py` (`ament_add_pytest_test`, 120 s, 26 cases): launch files load with fixed argument lists; the `.yaml` names in the launch files are exactly `nav2_params`, `ben`, `ben_sim`, each parses into `ros__parameters` blocks; every package named by `package=`, `FindPackageShare`, `get_package_share_directory`, non-stdlib imports, `plugin:` and `plugin_lib_names` (table: `marine_nav_behavior_tree_bt_plugins` -> `marine_nav_behavior_tree`, unknown library fails) is a `depend`/`exec_depend`, own name excluded; error-code guard; wiring of both `bt_task_navigator` descriptions (installed version patched to `1.3.14` and `1.5.0`; the first parameter must be the `ParameterFile`, the rest must evaluate to the helper's output). It puts its install prefix first on `AMENT_PREFIX_PATH` and asserts `get_package_share_directory('ben_project11')` is the install under test. The ROS 1 `.launch` files are outside it by extension (stated in its docstring).
- `test/test_bt_navigator_params.py` (`ament_add_ros_isolated_pytest_test`, 120 s; 30 s per service wait, fails at once with the log if the navigator exits): builds the parameters from the launch files' own `ReplaceString` and `bt_task_navigator` `LifecycleNode` parameter list, starts and configures stock `bt_navigator`, reads back the parameters (Nav2 >= 1.4: prefixes are the three and `error_code_names` undeclared; older: `error_code_names` are the three).
- **Breaks** (each on a fresh scratch copy built outside the source tree, `colcon build`, then the test file with `BEN_PROJECT11_INSTALL_PREFIX` at the scratch install; unbroken copy: 26 passed / 1 passed):

| Break | Caught by |
|---|---|
| `error_code_names` put back in `nav2_params.yaml` | `test_error_code_names_is_in_no_parameter_file`; navigator test: FATAL, `bt_navigator exited (-11)` |
| helper removed from the composable node | `test_both_bt_task_navigators_get_the_legacy_parameters[1.3.14]` ("ComposableNode ... Right contains one more item") |
| helper removed from the lifecycle node | same test ("LifecycleNode ...") |
| `ben_description` dropped from `package.xml` | `test_every_named_package_is_declared`: `not declared in package.xml: ['ben_description']` |
| `s57_layer` dropped (a plugin package) | same: `['s57_layer']` |
| broken indent in `nav2_params.yaml` | `test_parameter_file_parses_into_node_blocks[nav2_params.yaml]` (ScannerError) and 6 dependants |
| `declare_log_level_cmd` not added in `navigation_launch.py` | `test_launch_file_loads_with_its_arguments[navigation_launch.py]` ("Right contains one more item: 'log_level'") |
| `import no_such_module` in `ben_core_launch.py` | load test (`ModuleNotFoundError`) and manifest test |
| helper gives the old key below 1.6 | `test_legacy_error_code_params_from_1_4[1.4.0, 1.4.2, 1.5.0]`; navigator test: FATAL |
| helper returns a default on an unparsable version | `test_unparsable_version_is_an_error` x5 (DID NOT RAISE) |
| `hover` dropped from the prefixes | `test_error_code_name_prefixes_hold_hover` + 3 legacy cases; navigator test (prefixes `['compute_path', 'follow_path']`) |

- **Navigator test repeat**: 12 separate runs on Rolling, 12 passed (0.42 to 0.82 s), no failure; no `bt_navigator` left after.

### By-hand Rolling checks (step 9)

Nav2 underlay install built from `dcffa939` (reflog: clone 2026-11-19 08:51; builds 12:06 and 14:45; checkout fast-forwarded to `9113449e` on 2026-11-23 09:01, not rebuilt); `nav2_bt_navigator` 1.5.0 in both.

- Build: `./build.sh --parallel-workers 2 --packages-up-to ben_project11` -> `Finished <<< ben_project11`, `Summary: 1 package finished`.
- Tests: `./test.sh --parallel-workers 1 --packages-select ben_project11 --ctest-args -E test_bt_navigator_params` -> `26 passed`, `100% tests passed, 0 tests failed out of 1`, `Summary: 27 tests, 0 errors, 0 failures, 0 skipped`; it used `platforms_ws/install/ben_project11` (the test asserts the share directory). The navigator test was run outside colcon, with `python3 -m pytest` on domain 187 (12 runs above): colcon's isolated runner on Rolling (`run_rmw_isolated`) picks its own domain (seen 42, 71, 48), below 150, so it was not used here.
- `--show-args`: `ben_core_launch.py`, `nav2_bringup_launch.py`, `navigation_launch.py` all print their arguments, exit 0.
- `ament_xmllint package.xml`: `No problems found`.
- Stock nodes, one at a time on domain 187, each `/ben/<name>` with the launch-substituted file, configured through `change_state`, then stopped (none left running): `controller_server` (with `local_costmap`; `FollowPath` replaced by stock `RegulatedPurePursuitController`; `chart_layer` removed), `smoother_server`, `behavior_server` (`hover` removed), `waypoint_follower`, `velocity_smoother`, `collision_monitor`, `docking_server` (`opennav_docking`): configure success, state `inactive`, no parameter warning (only `LifecycleNode is not shut down` at stop and, on `docking_server`, `Dock database filepath nor dock parameters set`, expected with no docks). **`planner_server` (with `global_costmap`, `chart_layer` removed): configure FAILED: `[FATAL] [ben.planner_server]: Failed to create global planner. Exception: lookup_table_size must be in [0.0, 20.0]`.** With `lookup_table_size: 20.0` in a scratch copy it configures (`inactive`), warnings only `On approach iteration selected as <= 0 ...` and `maximum iteration selected as <= 0 ...` (from `max_on_approach_iterations: -1`, `max_iterations: -1`).

### New finding: `planner_server.GridBased.lookup_table_size: 55.0` (`config/nav2_params.yaml:178`), not fixed

Nav2 `d7bf2ac0` (2026-10-03, #6566, "Validate parameter bounds ...", on `main`, and on `lyrical` as `221cdb28` (#6595); not on `origin/jazzy` or `origin/kilted`) *(corrected in fix round 1, 2026-11-24: first written as "on `main` only"; the Local Review (Pre-Push) at `301536c` found it on `lyrical` too)* throws in `SmacPlannerHybrid` configure when `lookup_table_size` is outside [0, 20]. On Rolling the planner server therefore fails to configure and the lifecycle manager's bring-up fails: a second hard break the plan's reading missed. Jazzy accepts 55 and uses it (the Hybrid-A* heuristic lookup table's extent, in metres). Any fix changes planning on one distro or adds a second version test, so it is the owner's choice, not part of this implementation: (a) a version-gated override in `navigation_launch.py` like the error codes (Rolling gets 20, Jazzy keeps 55); (b) 20 on both (changes Jazzy planning); (c) leave it, record it for Part 2. Worth knowing: if Nav2 backports #6566 to Jazzy, a vehicle's apt upgrade would hit the same failure. Recommend the host raise it before publishing.

### Differences from the plan and the checkpoint

- The navigator test asserts by reading the parameters back, not by a log line: on Jazzy the "Error_code parameters were set to" line is not printed when the key is auto-declared, so the plan's expected Jazzy log line will not appear; the read-back is the check on both distros. CI prints the Jazzy navigator's log in its own step.
- The navigator test builds its parameters from the launch files' own `ReplaceString` and the lifecycle node's parameter list, rather than restating the substitutions.
- Both tests put the install prefix on `AMENT_PREFIX_PATH` in the test (at run time) rather than through CMake `ENV` (configure time), so a stale or differently sourced environment cannot put another `ben_project11` first.
- Proof-of-teeth extended with a dropped plugin package, a default instead of an error, and `hover` dropped.
- The colcon run of the navigator test under the isolation wrapper was not done (domain rule above).

### For the pull request

- **Shown on Rolling**: the build; the static test; the stock navigator configures with the committed file and logs the three prefixes, and refuses the old key; seven of the eight other stock nodes configure with their blocks (BEN plugins replaced or removed); the planner server does not (finding above).
- **Waits for Part 2**: BEN's navigator plugin (`TaskNavigator`), the behaviour-tree library (`marine_nav_behavior_tree_bt_plugins`), the path follower (`CrabbingPathFollower`), the hover behaviour (`Hover`) and the chart layer (`S57Layer`) are not ported, so nothing here shows they load or that a real hover failure reaches the result.
- **Result timeout**: on Rolling a finished goal's result is kept for about 10 s (rcl's default) now that Nav2 has no setting for it; that matters only to a client that asks late; `mission_manager` asks at once (`mission_manager.py:207`).
- **What the tests do not cover**: anything inside BEN plugins, value semantics, topics/QoS, TF-dependent activation, the composed bring-up at run time (only its descriptions), the ROS 1 files, and the other stock nodes' parameter handling (by hand only, above).
- **Unverified on Jazzy** (not installed here: Ubuntu 26.04, no Docker): all of it beyond reading and a `rosdep install --simulate --rosdistro jazzy --os ubuntu:noble` with the ten skip-keys (all other keys resolve; without them, `Cannot locate rosdep definition for [marine_nav_behavior_tree]`). The pull request's first CI run is the first Jazzy evidence and this repository's first CI run: rosdep install, build, both tests, the navigator reading back `error_code_names`. Rehearsed on Rolling: the workflow's build and test steps with only `/opt/ros/rolling` and the underlay sourced, no workspace install (26 passed); the workflow's `rosdep` step on Rolling stops at `nav2_bringup` (no Nav2 release), as the file's header says.

### Left alone (for a follow-up issue)

- 22 ROS 1 `.launch` files (`launch/`): perception/sensors (`asv_perception`, `get_remote_radar`, `get_sensors`, `get_yolo_detects`, `test_radar_target_tracker`, `view_pano_thumbnails`); logging/replay (`log_lidar`, `log_sensors`, `log_temps`, `log_throttled_panos`, `replay_bag`); ROS 1 navigation (`costmap`, `costmap_chart`, `costmap_radar`, `move_base`, `navigator`); operator (`operator_mystique`, `operator_rqt`, `operator_ui`, `remote_operator`); simulation (`sim_ben_sandisland`, `sim_local_cw4`). Name a missing file: `replay_bag`, `operator_mystique`, `remote_operator` (`ben_description/launch/load.launch`), `costmap_chart` (`config/costmap_chart_voyage.yaml`), `sim_local_cw4` (`parameters/ben.yaml`). Companions in `config/`: `costmap.yaml`, `costmap_chart.yaml`, `costmap_common.yaml`, `costmap_radar.yaml`, `navigator.yaml`, `navigator_ccom_planner.yaml`, `dubins_planner.yaml`, `path_follower.yaml`, `platform.yaml`, `navigator.rviz`, `operator.rviz`, `operator.perspective`, the three `.desktop` files; `scripts/` (three tmux scripts).
- `nav2_bringup` dependency (`package.xml`; this repository's issue 12).
- Blocks that configure nothing: `config/nav2_params.yaml` `map_saver`; `config/ben.yaml` `navigator`, `asv_helm`.
- Drift of the local copies of Nav2's bring-up files (`launch/nav2_bringup_launch.py`, `launch/navigation_launch.py`).
- Composed mode in `launch/navigation_launch.py` starts a stray stock `bt_navigator` and omits `manda_coverage` although it is in `lifecycle_nodes`.
- `config/ben_sim.yaml` applied inside a scoped `GroupAction` in `launch/ben_core_launch.py`, so its costmap settings may not reach Nav2.
- `navigate_through_poses_error_code` written by unh_marine_navigation's `run_tasks.xml` but in no list.
- `buildtool_depend ament_cmake_ros` (`package.xml`; now used by the isolated test).
- Out of scope: parameters for the unreviewed field-server navigation work.
- New: `lookup_table_size: 55.0` (above), pending the owner's choice.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 11:22 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: changes-requested

**Branch**: feature/issue-30 at `301536c` (code head; branch head `ee7aa66` adds only the implementation record)
**Base**: origin/rolling
**Depth**: Standard (reason: 7 code files, ~760 code lines, a new CI workflow and a launch-file change that reaches a vehicle; the other ~660 lines are `.agent/` records)
**Must-fix**: 2 | **Suggestions**: 6
**Round**: 1 | **Ship**: continue — round 1: 2 must-fix (both mechanical); first round always re-reviews after fixes

Reviewers: static analysis (ament_flake8, ament_pep257, ament_xmllint; nothing on touched lines worth reporting: a deliberate `except Exception` that re-raises, one C402, two D213), governance and plan drift (by the lead), a fresh Claude adversarial subagent, and cross-model Gemini and Codex (both completed, `EXIT=0`). Run here: the package built and `colcon test --packages-select ben_project11` under the registered wrappers (29 tests, 0 failures; `test_bt_navigator_params` passed under `ament_add_ros_isolated_pytest_test`, which chose its own domain); three breaks of my own on scratch copies (below); `rosdep resolve --rosdistro jazzy --os ubuntu:noble` on every key; gitcloud `field/ben_project11` checked (`jazzy` = GitHub `jazzy` `b8a53a3`; its `feature/issue-10` is already in `jazzy`; no field Nav2 fork there).

### Jazzy vehicle behaviour, per commit (Nav2 `origin/jazzy` `c7046550`, `nav2_bt_navigator` 1.3.14; Jazzy not installed here, so by reading; the first CI run is the first Jazzy run)
- `aae76a0` manifest: unchanged (no launch or parameter change; every non-skipped key resolves on jazzy/noble; the ten skip-keys have no rule on any distro).
- `c02aa8a` error codes: unchanged **for the shipped `nav2_params.yaml`**: Jazzy `BtNavigator` auto-declares overrides (`bt_navigator.cpp:37`), so `has_parameter("error_code_names")` skips the defaults block (`bt_action_server_impl.hpp:82`) and `:179` reads the three names from the second params file launch_ros writes from the dict (`node.py:363-368,440-457`, keyed `/ben/bt_task_navigator`); `error_code_name_prefixes` is read nowhere on `origin/jazzy`. **Changed for an operator-supplied `params_file` that carries its own `error_code_names`**: the dict comes after the file and replaces that list (must-fix 1). The lookup cannot raise on a healthy install (ament installs `package.xml` to `share/<pkg>`; `1.3.14` parses); if it does, the exception is raised when `navigation_launch.py` is loaded: directly, before any node; under `ben_core_launch.py`, when the Nav2 include is visited, so launch reports "Caught exception in launch" and shuts down what it already started; only reachable when `nav2_bt_navigator` is missing or broken, when the navigator could not start anyway.
- `934ad6b` `action_server_result_timeout`: unchanged (900.0 is Jazzy's default on both nodes, `bt_action_server_impl.hpp:67-68`, `waypoint_follower.cpp:40`).
- `2686472` `dock_backwards`: unchanged (`false` is Jazzy's default, `docking_server.cpp:41`).
- `cd63732`, tests, CI: comment, test and CI only.

### `lookup_table_size: 55.0` (implementer's finding, for the owner; facts only)
- Jazzy: `GridBased.lookup_table_size` (m) / (resolution 1.0 × downsampling 1) → table dim 55 (odd, kept); 20 → 20 → 21. Table holds `dim × ceil(dim/2) × 72` Dubins distances: 55 × 28 × 72 = 110,880 floats (~0.43 MB) vs 21 × 11 × 72 = 16,632 (~65 KB). It is the obstacle-free Dubins (turning radius 5 m = 5 cells) heuristic within ±27 cells (55) or ±10 cells (21) of the goal; outside, `getDistanceHeuristic` gives 0 when an obstacle heuristic exists and the heuristic is `max(obstacle, dist)` (`node_hybrid.cpp:443-450,746-781` on jazzy). So a smaller table changes heuristic guidance (search effort and possibly path shape near the goal approach), not feasibility.
- Bound: `d7bf2ac0` (#6566) is on `main`, and **also on `lyrical`** as `221cdb28` (#6595, merged 2026-10-02): the implementation entry's "main only" is wrong. Not on `origin/jazzy` or `origin/kilted`; #6566 carries only the `backport-lyrical` label.
- Rest of that commit against BEN's files: `angle_quantization_bins` in [1, 1024] (72 ok), `analytic_expansion_max_length` in [0, 1000] (25.0 ok), costmap width/height > 0 and cell count < INT_MAX (5000 × 5000 = 25 M ok; local 350), controller/planner fail on a failed costmap configure. `max_iterations: -1` is not bounded there (warning only, as the implementer saw). The other stock nodes configured on Rolling in the implementer's run.

### Breaks of my own (scratch copies, built outside the tree, static test run against each install)
- Helper result placed before `configured_params`: caught by `test_both_bt_task_navigators_get_the_legacy_parameters[1.3.14]` (but see must-fix 1: that order is the better one).
- Boundary changed to `(1, 3)`: caught by `test_legacy_error_code_params_before_1_4[1.3.0, 1.3.14, 1.3]`.
- `marine_nav_behavior_tree` (named only through `plugin_lib_names`) dropped from `package.xml`: caught by `test_every_named_package_is_declared`.

### Findings
- [x] (must-fix) Put `legacy_error_code_params(...)` **before** `configured_params` for both `bt_task_navigator` descriptions, so a Jazzy `params_file` with its own `error_code_names` keeps it (today's behaviour) and the shipped file still gets the three names; update the wiring test and the comment. As written, a custom list is silently replaced on Jazzy (Codex, Gemini) — `launch/navigation_launch.py:135-137`, `test/test_launch_and_config.py:270-273`
- [x] (must-fix) `subprocess.Popen`, `rclpy.init()` and `create_node()` sit outside `try`, so a failure there leaves `bt_navigator` running; start the `try` right after `Popen` and guard the teardown (Claude adversarial, Gemini, Codex) — `test/test_bt_navigator_params.py:163-167`
- [x] (suggestion) CI has no zero-test guard: `colcon test-result` returns 0 with no results, and its `if: always()` step also runs (and fails on the missing `../ws`) after an early failure; gate it like `Test` and assert the expected test count; the navigator-log glob depends on pytest's 30-character truncation — `.github/workflows/ci.yml:124-146`
- [x] (suggestion) The Jazzy read-back writes the extra parameters as its own `/**` YAML with lists, not the file launch_ros writes (`/ben/bt_task_navigator`, tuples); consider using `Node._create_params_file_from_dict` so the test reads what a vehicle receives — `test/test_bt_navigator_params.py:148-153`
- [x] (suggestion) `reaches_parameters` passes an empty node block (`all([])` is true) — `test/test_launch_and_config.py:162-167`
- [x] (suggestion) Comments name versions loosely: "Nav2 1.4 and later (Rolling)" omits Kilted and Lyrical; the `behavior_server` comment's "1.4 and older" misses Lyrical 1.5.3, which also reads the settings on the server (only `main` reads them per behaviour) — `config/nav2_params.yaml:25-28,223-227`
- [x] (suggestion) Correct the implementation record for the pull request: #6566's bound is on `lyrical` too (`221cdb28`, #6595), not `main` only — `.agent/work-plans/issue-30/progress.md` (Implementation, "New finding")
- [x] (suggestion) Owner call already being raised by the host: `lookup_table_size: 55.0` stops `planner_server` configuring on Rolling and Lyrical; facts above — `config/nav2_params.yaml:178`

## Implementation
**Status**: complete
**When**: 2026-11-24 11:43 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-30 at `33fb08f`
**Addressed**: Local Review (Pre-Push) at `301536c` (2026-11-24 11:22 -05:00)
**Commits**: `47f00d3`, `239aae0`, `da1937f`, `3e0dbec`, `55c2a1a`, `3995d83`, `0a5dcac`, `7da4b06` (code, none touching `.agent/`, each ends "Backport to jazzy."); `33fb08f` (records: boxes ticked, implementation record corrected)

Fix round 1. Nothing pushed or posted. Rolling only (Nav2 underlay built from `dcffa939`, which contains `d7bf2ac0`; `nav2_bt_navigator` and `nav2_smac_planner` 1.5.0); every process on domain 187 (checked empty with `ros2 node list --no-daemon`), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `DISPLAY` unset; no navigator or planner left running.

### Actions
- [x] (must-fix) Legacy `error_code_names` now **before** `configured_params` on both `bt_task_navigator` descriptions — `47f00d3`, `launch/navigation_launch.py` (`bt_task_navigator_params`), comments there; wiring test requires the `ParameterFile` last and the helper's result before it. Precedence by reading: launch_ros `node.py:442-462` (Rolling) appends one `--params-file` per evaluated entry in list order; composable nodes: `to_parameters_list` keeps list order. By experiment on Rolling: stock `bt_navigator` via `LaunchService`, `bt_loop_duration` 11 in a dict and 22 in a file: dict first → 22, file first → 11, with the file keyed `/**` and `/ben/order_probe` alike (later wins). Breaks on scratch installs: helper after the file → `test_both_bt_task_navigators_get_the_legacy_parameters[1.3.14]` (`isinstance(parameters[-1], ParameterFile)`); helper missing from the lifecycle node / the composable node → same test (`[] == [{'error_code...`). The navigator test passes the files in the launch file's order.
- [x] (must-fix) `test_bt_navigator_params`: `try` right after `Popen`, `rclpy.init`/`create_node` inside, teardown chained (`destroy_node` → `rclpy.shutdown` → `stop(process)`, each in the previous one's `finally`) — `239aae0`. Forced failures on scratch copies (navigator PID recorded by the patched test, checked with `kill -0`): at `rclpy.init`, `create_node`, the checks, `destroy_node`, `rclpy.shutdown`: each test fails with the forced error and the navigator is gone; control (the previous test forced at `rclpy.init`): navigator still running, stopped by its PID.
- [x] (suggestion) CI — `0a5dcac`: `Test results` gated like `Test`; fails unless the result count equals collected pytest cases (`pytest --collect-only -q src/ben_project11/test`) plus CTest entries (`ctest --test-dir build/ben_project11 -N`), so the expected count is derived in the step and cannot go stale; navigator log found by file name (`find ... -path '*pytest-of-*' -name bt_navigator.log`), not pytest's truncated directory. Rehearsed on Rolling (the two steps' scripts extracted from the workflow, Jazzy setup swapped for `env.sh`): `results: 62; expected: 62 (60 pytest cases + 2 CTest entries)`; with no results `0` vs `61`, with the navigator test unregistered in CMake `60` vs `61`, both exit 1.
- [x] (suggestion) The navigator test now gives the node the files launch_ros itself writes — `da1937f`: `Node._perform_substitutions(context)` after `nav2_bringup_launch.py`'s `PushROSNamespace` is applied, then the `--params-file` arguments from the public `node.cmd`; only the RewrittenYaml file is changed (stock navigator). Choice: `_perform_substitutions` is protected, but launch_ros's own subclasses call it and `_create_params_file_from_dict` needs it anyway (it reads `node_name`); a launch_ros refactor makes the test fail (AttributeError / no `--params-file`), not pass; restating the file format in the test is what drifted. Found on the way: launch_ros writes the list as `!!python/tuple`, which `yaml.safe_load` refuses, so those files are copied verbatim. With the version forced to 1.3.14 on Rolling, that file reaches the node: `[FATAL] ... parameter 'error_code_names' has been replaced by  'error_code_name_prefixes' and MUST be removed.`
- [x] (suggestion) `reaches_parameters` needs a non-empty block — `3e0dbec`. Break: `stray_node: {}` and `stray_ns: {stray_node: {}}` appended to `nav2_params.yaml`: caught now, passed with the previous test.
- [x] (suggestion) Version wording — `55c2a1a`: prefixes read from 1.4 on (Kilted 1.4, Lyrical 1.5, main/Rolling), Jazzy 1.3; `behavior_server` settings read on the server up to Lyrical 1.5 (checked on `origin/lyrical`: `spin.cpp:52` `declare_or_get_parameter("simulate_ahead_time", ...)`), per behaviour on `main` only. Comments only.
- [x] (suggestion) Implementation record corrected in `33fb08f`: #6566 is on `lyrical` too (`221cdb28`, #6595).
- [x] (owner call, decided 2026-11-24 "20 on new Nav2, 55 stays on Jazzy") `lookup_table_size` — `3995d83` (+ `7da4b06`, import order). Below.

### `lookup_table_size`: the switch
- `nav2_smac_planner` `package.xml`: `d7bf2ac0` 1.5.0 and its parent 1.5.0 (main); `221cdb28` 1.5.2 and its parent 1.5.2 (lyrical); tips: `jazzy` `c7046550` 1.3.14, `kilted` `f3ead29f` 1.4.2, `lyrical` `60b44e96` 1.5.3, `main` `9113449e` 1.5.0. Lyrical history: 1.5.0 at branch-off (2026-07-27), 1.5.1 (08-11), 1.5.2 (09-15), limit 10-02, 1.5.3 (10-16).
- **Versions cannot tell an install with the limit from one without** (same number before and after on both branches). Chosen: **nav2_smac_planner ≥ 1.5** (safe side). Lowering to 20 where the limit is not (Lyrical 1.5.0–1.5.2, main before `d7bf2ac0`) only shrinks a heuristic table; leaving 55 where it is stops the planner server. Jazzy 1.3 and Kilted 1.4 are untouched. A future backport of the limit to jazzy or kilted would not be seen (said in the code comment).
- The limit is in `SmacPlannerHybrid` only (`smac_planner_hybrid_impl.hpp:203` on main; Lattice and 2D have none).
- Design: `config/nav2_params.yaml` keeps `lookup_table_size: 55.0` (a YAML comment added; values unchanged). `planner_server_params()` gives both planner descriptions `[configured_params]` (the same RewrittenYaml every node gets) below 1.5; from 1.5 `ParameterFile(LookupTableSizeCapped(rewritten_params), allow_substs=True)`: at launch time it reads the evaluated RewrittenYaml file, and for each planner in any block's `planner_plugins` whose plugin is `SmacPlannerHybrid` (`::` or `/` form, nested or dotted keys) lowers `lookup_table_size` above 20 to 20.0 and logs `[WARNING] [ben_project11.navigation_launch]: planner_server: GridBased.lookup_table_size 55.0 lowered to 20.0 (this nav2_smac_planner refuses more)`; it returns the file itself when nothing is above 20. So a custom file is never raised (10 stays 10), never given a value it did not have, and a legal value is never lowered. An override dict after the file was rejected: it would raise a custom 10 to 20 (a break shaped like it fails three tests).
- Version read the same way as the navigator's (`installed_version(package)`, both wrappers kept), raising on any lookup failure or unparsable version. `python3-yaml` moved from `test_depend` to `exec_depend` (the launch file imports yaml; `rosdep resolve python3-yaml --rosdistro jazzy --os ubuntu:noble` → `python3-yaml`).
- Static tests (33 new cases, 59 in the file): shipped value stays 55.0 with `SmacPlannerHybrid`; limited for 1.5.0/1.5.2/1.5.3/1.6.0/2.0.0, not for 1.3.0/1.3.14/1.3/1.4.0/1.4.2; unparsable (`''`, `abc`, `1`, `1.5.0rc1`, `None`) raises, also from `generate_launch_description`; missing package and `package.xml` without `<version>` raise; the cap on 55/55 (int)/20.5 → 20, 20/20 (int)/10/0 unchanged, dotted + slash plugin, absent value, Lattice, unlisted planner; the substitution returns the source path when nothing changes; wiring: before 1.5 both descriptions' `ParameterFile` wraps the very RewrittenYaml `controller_server` gets; from 1.5 both evaluate to the file other nodes get except `GridBased.lookup_table_size` (shipped → 20.0, custom 10 → 10.0, absent → absent).
- Breaks (scratch installs): boundary (1, 4) → 3 failures; (1, 6) → 4; cap lowers legal values → 6; cap adds a value when absent → 2; composable or lifecycle description missing the cap → `capped_copy_from_1_5[None-20.0]`; append-dict design → 3; unparsable smac version accepted → 6; always capped (Jazzy changed) → 3; any planner type capped → 1; unreadable version defaulting to 1.5.0 → 1.
- **Rolling proof**: the files launch_ros writes from the launch file's own `planner_server` LifecycleNode (after `nav2_bringup_launch.py`'s `ReplaceString` and `PushROSNamespace`), `chart_layer` (s57_layer, not ported) removed from `global_costmap`; stock `planner_server` as `/ben/planner_server`: `ros2 lifecycle set ... configure` → `Transitioning successful`, state `inactive [2]`, `ros2 param get /ben/planner_server GridBased.lookup_table_size` → `Double value is: 20.0`; log: `Created global planner plugin GridBased of type nav2_smac_planner::SmacPlannerHybrid`, `Even sized heuristic lookup table size set 20.000000, increasing size by 1 to make odd`, warnings only for `max_on_approach_iterations`/`max_iterations` -1. Control, version forced to 1.4.2: file value 55.0, `Transitioning failed`, `[FATAL] [ben.planner_server]: Failed to create global planner. Exception: lookup_table_size must be in [0.0, 20.0]`.

### Tests
- Build: `build.sh --parallel-workers 2 --packages-select ben_project11` → `Summary: 1 package finished`.
- Full `colcon test` (`test.sh --parallel-workers 1 --packages-select ben_project11`, domain 187 set; the isolated wrapper picks its own): `59 passed`, `1 passed`, `100% tests passed, 0 tests failed out of 2`, `Summary: 62 tests, 0 errors, 0 failures, 0 skipped` (run after the last code commit).
- Navigator test: 12 separate runs on domain 187, 12 passed (0.62–0.83 s); none left running.
- `ament_flake8 test/`: no problems; `launch/navigation_launch.py`: 19 warnings, the same count as on `rolling` (`7da4b06` removed the one I100 `3995d83` added); `ament_xmllint package.xml`: no problems.

### Why Jazzy's behaviour is unchanged, per code commit (by reading; Jazzy not installed here)
- `47f00d3`: for the shipped file the node gets the same three names (no `error_code_names` in the file, the dict's value stands); a custom `params_file` with its own list keeps it again, as it did before `c02aa8a`. That restores today's behaviour for custom files.
- `239aae0`, `da1937f`, `3e0dbec`: tests only.
- `55c2a1a`: comments only.
- `3995d83`: below 1.5 (`nav2_smac_planner` 1.3.14 on Jazzy) both planner descriptions get `[configured_params]`, the same object as before; `nav2_params.yaml` values unchanged. New on Jazzy: the launch file reads `nav2_smac_planner`'s `package.xml` when loaded and raises if it cannot (only when the planner package is missing or broken, when the planner could not start anyway); `python3-yaml` is now an exec dependency (already present wherever launch_ros is).
- `0a5dcac`: CI only. `7da4b06`: import order only.

### For the pull request
- `lookup_table_size` is the extent (m) of SmacPlannerHybrid's precomputed obstacle-free Dubins heuristic table around the goal: with 1 m cells about ±27 m at 55, about ±10 m at 20 (21 cells after the odd-size bump); outside it the obstacle heuristic guides the search. It changes heuristic guidance (search effort, possibly path shape on the final approach), not feasibility. Nothing was run with a vessel or simulator; **whether 20 plans acceptably on Rolling is a Part 2 check.**
- Nav2 #6566's limit is on `main` and `lyrical` (`221cdb28`, #6595), not `jazzy` or `kilted`; a Lyrical install between 1.5.0 and 1.5.2 without it also gets 20 (version-indistinguishable).

### Unverified (Jazzy not installed here)
- Everything on Jazzy beyond reading: the reordered dict reaching the Jazzy navigator, `Node._perform_substitutions`/`node.cmd` behaving the same in Jazzy's launch_ros, the planner wiring with the real 1.3.14 install, and the CI count (`ctest --test-dir`, junit counting) on the Jazzy image. The first CI run is the first Jazzy evidence.
- Composable precedence (container's parameter overrides, later wins) is by reading launch_ros only; no container was started.
- The CI `Install ROS dependencies`, `Build` and `Test` steps were not re-rehearsed beyond the local build and test above.
- colcon's test runs used pytest's default temporary root (`/tmp/pytest-of-<user>`, pruned by pytest to its last three); my own scratch went to the session scratchpad and is removed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-24 11:53 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-30 at `33fb08f` (code head `7da4b06`; branch head `ac23ea9` adds only the implementation record)
**Base**: origin/rolling
**Depth**: Standard (reason: whole-branch classification as round 1, a launch file that reaches a vehicle and a CI workflow; this round reviews `301536c..33fb08f`, eight code commits, +569 -142)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Reviewers: static analysis (ament_flake8: `navigation_launch.py` 19 warnings vs 20 on `rolling`, tests clean; ament_pep257: D213/D401 docstring style only; ament_xmllint clean), governance and plan drift (lead), a fresh Claude adversarial subagent (completed), cross-model Gemini and Codex (both `EXIT=0`, completed). Codex rated the cap's custom-file misses must-fix and Gemini rated the `action_server_result_timeout` removal must-fix; both downgraded or rejected below.

Hygiene: no `.agent/` file in the eight code commits; each code commit's message ends "Backport to jazzy."; one fix per commit.

### Jazzy vehicle behaviour, per new commit (read against Nav2 `origin/jazzy` and launch_ros/rclcpp/rcl `jazzy` from GitHub; Jazzy is not installed here)
- `47f00d3` order: unchanged for the shipped file (the dict keyed `/ben/bt_task_navigator` gives the three names, which the file does not set). A custom file with its own node-keyed `error_code_names` (the shape RewrittenYaml writes under root key `ben`) keeps them, as before `c02aa8a`. rclcpp `resolve_parameter_overrides` and `parameter_map_from` apply the node-name entries of the merged `rcl_params_t` in order of first appearance, and the same name in a later file overwrites in place. Rolling bt_navigator, domain 173: with `-p use_sim_time` + dict + nested file the file wins; nested file + dict gives the dict. Composable (`to_parameters_list`, Jazzy): one batch per entry in list order, duplicates go to `NodeOptions` overrides, last wins, so the file wins in every key shape.
- `3995d83` cap: on 1.3.14 `planner_server_params` returns `[configured_params]`, the very object every other node gets. New at load: reading `nav2_smac_planner`'s `package.xml` (an existing exec_depend; Jazzy 1.3.14 parses) and `import yaml` (`python3-yaml` resolves on jazzy/noble). The limit is absent on `origin/jazzy` and `origin/kilted` and present on `lyrical` and `main`; `nav2_smac_planner` is 1.5.0 before and after `d7bf2ac0` and 1.5.2 before and after `221cdb28`.
- `239aae0`, `da1937f`, `3e0dbec`: tests only. Jazzy launch_ros has `Node._perform_substitutions` with the same body as Rolling, apart from Rolling's quoted-string representer in `_create_params_file_from_dict`. `_Node__node_name` and `cmd` are also there.
- `0a5dcac`: CI only. `55c2a1a`: comments only, accurate against the branches. `7da4b06`: import order only.

### Checks run
- `colcon test` (`test.sh --parallel-workers 1 --packages-select ben_project11`, domain 174 checked empty): `Summary: 62 tests, 0 errors, 0 failures, 0 skipped`. The CI count snippet gives 60 collected + 2 CTest = 62; collection also works with only the ROS and underlay setup sourced, as in CI.
- Navigator test: 10 separate runs, 10 passed (0.44 to 0.92 s); no navigator left running.
- Capped-file fidelity: ReplaceString → RewrittenYaml (root `ben`) → `LookupTableSizeCapped`, compared scalar by scalar (path, YAML tag, value, quoted or not) and by Python type. On the shipped file, 241 scalars match except `planner_server…GridBased.lookup_table_size` 55.0 → 20.0. On a stress copy with `1.0e-10`, `1e-10`, `True`, `"123"`, `'1.5'`, `y`, `"yes"`, `"on"`, `[]`, empty and null values, a mixed list, nested lists, `-0.0`, `.inf`, a long string, `\n` and `#`/`: ` strings, 264 scalars match with the same single difference.
- Mutations on scratch installs: cap on 1.3 and later → 7 failures (`…not_limited_before_1_5[1.3.14]`, `…get_the_parameter_file_before_1_5[1.3.14]`, …); the cap also changing `expected_planner_frequency` → 6, among them `…capped_copy_from_1_5[None-20.0]` (whole-file comparison) and `…leaves_the_rest_alone`; the legacy dict after the file → `test_both_bt_task_navigators_get_the_legacy_parameters[1.3.14]`.

### Rejected or below threshold
- Gemini: removing `action_server_result_timeout` makes Jazzy default to 10 s. False: Jazzy declares 900.0 (`bt_action_server_impl.hpp:67-68`, `waypoint_follower.cpp:40`), and round 1 settled it.
- Gemini: derive `error_code_names` from the file's prefixes. Out of this round; this is round 1's accepted design.
- Gemini: `packages_named_by_parameter_files` and the test's deadline assert. Not reachable with the shipped files.

### Findings
- [ ] (suggestion) The cap's weight against the owner's option. The design is correct and inert on Jazzy, but it is about 110 code lines in a launch file that vehicles run. It still misses custom-file shapes: `planner_plugins` in one block with the `GridBased.*` settings in a `/**` block or another file, a `$(var …)` value (the cap runs before ParameterFile's substitution), a file with no `planner_plugins` (Nav2 defaults it to `[GridBased]`), and a non-string entry (TypeError). Each of those fails loudly at configure, as it does without the cap. The capped copy `ben_planner_params_*` is never removed, like RewrittenYaml's own file. A dict `{'GridBased.lookup_table_size': 20.0}` after the file, only on `nav2_smac_planner` 1.5 and later, gives "20 on new Nav2" in about 10 lines. It loses only the protection of a custom value below 20 on Rolling and Lyrical, which no vehicle runs. The host decides — `launch/navigation_launch.py:111-220`
- [ ] (suggestion) The comment "a params_file that sets its own error_code_names keeps them" is true for node-keyed lists, not for a `/**`-keyed list when the namespace is empty: SetParameter's `-p use_sim_time` makes `/**` the first node entry, so the dict wins. Shown on Rolling: `-p` + dict + `/**` file gives the dict, and without `-p` the file wins. Vehicles (namespace `ben`) are not affected: there a `/**` key under the root key does not parse at all (`/ben// contains repeated forward slash`, already the case before this branch). Qualify the comment or leave it — `launch/navigation_launch.py:276-280`
- [ ] (suggestion) CI count: `grep -c '::'` counts any collect-only line with `::`, including a warning text that names a C++ type. Take the count from the `N tests collected` line instead. With 0 cases, `grep -c` exits 1 under `pipefail` and the step stops without its message — `.github/workflows/ci.yml:157-159`
- [ ] (suggestion) For the pull request, tell the owner three things. The 1.5 switch cannot tell Lyrical 1.5.0 to 1.5.2 or main before `d7bf2ac0` from builds that have the limit. If Nav2 backports the limit to jazzy, a vehicle's `planner_server` stops configuring after an apt upgrade (the code comment says so; the PR should too). `navigation_launch.py` now needs `nav2_smac_planner` installed to load, even with a params_file that uses no Smac planner — `launch/navigation_launch.py:120-128,274`

### Round-1 items
All eight are closed by `47f00d3`, `239aae0`, `0a5dcac`, `da1937f`, `3e0dbec`, `55c2a1a`, `33fb08f` and `3995d83`, each checked above. Only CI will show the Jazzy evidence: the reordered dict reaching the Jazzy navigator, `_perform_substitutions` on Jazzy's launch_ros, the 1.3.14 planner wiring, and the count step on the Jazzy image.

## Checkpoint
**Status**: complete
**When**: 2026-11-24 12:00 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: host (the round-2 review approved with suggestions; the host takes its recommendation on the planner cap's design, a mechanism choice within the owner's "20 on new Nav2, 55 stays on Jazzy")
**After**: publish
**Decision**: address

Read: the Local Review (Pre-Push) round 2 at `33fb08f`, approved, no must-fix, four suggestions. One more narrow fix pass before publishing: (1) **the planner cap is replaced by the simple override the review recommends**: on `nav2_smac_planner` 1.5 and later, a parameter entry `GridBased.lookup_table_size: 20.0` placed after the parameter file for both `planner_server` descriptions; below 1.5 the planner gets exactly the parameter file, as before. The launch-time rewriting substitution (about 110 lines), its temporary file and the `python3-yaml` exec dependency go. What is given up: on Rolling and Lyrical a custom parameter file's own value for that one key is overridden to 20 (also a legal value below 20), and the key is set when the file has none; no vehicle runs there, the shipped file is what the switch exists for, and the comment says how to change it. The owner's option read "the launch file lowers the value to 20 only where Nav2 has the limit, the same way the error-code fix switches on Nav2's version"; this is that, in about ten lines. (2) The comment on the error-code order is qualified for a `/**`-keyed list with an empty namespace. (3) The CI count takes pytest's "N tests collected" line and cannot stop silently on zero. (4) The pull request tells the owner the three things the review lists (the version cannot mark the limit exactly; a Nav2 backport of the limit to Jazzy would stop `planner_server` on a vehicle after an upgrade and the switch would not see it; the launch file now needs `nav2_smac_planner` installed to load).
