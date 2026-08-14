<!--
Copyright (c) 2026 Kuka
All rights reserved.
-->
# MoveIt.BT: an authoring guide

This library lets you assemble, plan, and execute a MoveIt Task Constructor (MTC)
task from a BehaviorTree.CPP tree instead of from C++. This document covers both
sides of that: **Part 1** is for someone writing a tree, **Part 2** is for someone
adding a node to the library. Read Part 0 first either way — the execution model
is small, and everything else follows from it.

This guide assumes you know what an MTC Task and Stage are. If you do not, read
`docs/authoring_guide.md` in the `moveit_task_constructor` fork first; it is the
general MTC reference and this document does not repeat it.

**Two kinds of statement live here, and they age differently.** The model and the
decisions are stable. The behavioural claims (a line number, a port default, "this
is hardcoded") can go stale under us. Every such claim carries a `file:line`
citation into this repository. Where the code and this document disagree, the code
wins and this document is the defect.

## Source of truth

All citations are relative to the root of **this repository** and were verified
against commit `178c1cf` (branch `main`). Run `./docs/verify_citations.py` to
re-check them all in one pass; it pins each citation to the text that was on that
line and tells you what moved. After you change code that a citation points at,
fix the guide and re-run with `--update`.

The library is `mtc_bt/`; the demo tree, config, and launch file are
`mtc_samples/`. This library is under construction — Part 3 lists the gaps I found
while writing this, and they are real.

---

## 0. The execution model

**An MTC task is built up across several BT ticks and carried on the blackboard.**
That single sentence explains every design choice in the library.

In C++ you would write a function that constructs a `Task`, adds stages to it, and
calls `plan()`. Here that function is spread across BT nodes:

1. A **planner factory** node constructs an MTC solver and writes a
   `solvers::PlannerInterfacePtr` to a blackboard entry.
2. `MTCInitializeTask` constructs a `Task` and writes a `TaskPtr` to a blackboard
   entry.
3. Each **stage builder** node reads that `TaskPtr`, constructs one MTC stage,
   appends it to the task, and returns `SUCCESS`. It does not plan.
4. `MTCPlanTask` reads the `TaskPtr`, calls `init()` then `plan()`, and returns
   `RUNNING` until planning finishes.
5. `MTCExecuteTask` reads the `TaskPtr` and executes its best solution.

Because a `TaskPtr` is a `shared_ptr`, every node in the sequence mutates the same
underlying `Task`. The blackboard is carrying a handle, not a value. This is the
most important thing to internalise: **the tree is a task *builder*, and only steps
4 and 5 are the task *running*.**

A consequence tree authors get wrong: the sequence from `MTCInitializeTask` to
`MTCExecuteTask` is **one** MTC task. Two such sequences in a row are two
independent tasks, each re-seeded from the robot's current state, each planned and
executed to completion before the next begins. That is sequential plan-then-execute
— it is *not* MTC searching across the whole thing. See section 1.4 for what that
costs you and when it is the right call anyway.

---

# Part 1 — Authoring trees

## 1.1 The shape of a task-running tree

The demo tree `mtc_samples/behavior_trees/test_mtc.xml` is the canonical shape:

```xml
<Sequence>
  <!-- Planners are constructed once and reused by every stage below. -->
  <MTCCartesianPlanner planner="{cartesian_planner}"/>
  <MTCPipelinePlanner planner="{pipeline_planner}"/>

  <Sequence name="Move Up">
    <MTCInitializeTask task_name="Move up" task="{mtc_task}"/>
    <MTCCurrentStateStage task="{mtc_task}"/>
    <MTCMoveRelativeStage task="{mtc_task}" planner="{cartesian_planner}"
                          group_name="panda_arm" ik_frame="panda_link8"
                          min_distance="0.1" max_distance="0.2"/>
    <MTCPlanTask task="{mtc_task}" max_solutions="1"/>
    <MTCExecuteTask task="{mtc_task}"/>
  </Sequence>
</Sequence>
```

Three rules follow from the model:

- **Construct planners once, at the top.** They hold no per-task state, and
  rebuilding a `PipelinePlanner` per task is wasted work.
- **Every task sequence starts with `MTCInitializeTask` and a generator stage.**
  In practice the generator is `MTCCurrentStateStage`; without a stage that seeds
  interface states, planning has nothing to propagate from and will fail at
  `init()`.
- **`MTCPlanTask` and `MTCExecuteTask` close the sequence.** Reusing the same
  `{mtc_task}` key for the next task is fine and normal — `MTCInitializeTask`
  overwrites the entry with a fresh `Task`, and the old one is released when the
  last handle to it drops.

## 1.2 Node reference

Registered node names come from `mtc_bt/src/mtc_plugin.cpp:31-39`. Defaults shown
are the port defaults declared in each header; a port with no default is required
and its absence fails the tick.

### Planner factories

| Node | Ports | Notes |
|---|---|---|
| `MTCPipelinePlanner` | `goal_joint_tolerance` (in, `1e-5`), `planner` (out) | Full MoveIt planning pipeline; free-space motion. Needs the ROS node, so it is registered with `params` (`mtc_plugin.cpp:32`). |
| `MTCCartesianPlanner` | `max_velocity_scaling_factor` (in, `1.0`), `max_acceleration_scaling_factor` (in, `1.0`), `step_size` (in, `0.01`), `planner` (out, default key `{cartesian_planner}`) | Straight-line Cartesian interpolation. Reach for it for approach, lift, retreat. |

Neither factory validates its inputs in practice — see Part 3, item 4. Pass sane
values.

### Task lifecycle

| Node | Ports | Notes |
|---|---|---|
| `MTCInitializeTask` | `task_name` (in, required), `task` (out) | Creates the `Task`, names its top-level container, and calls `loadRobotModel` (`mtc_init_task.cpp:46`). Registered with `params` because it needs the ROS node. |
| `MTCPlanTask` | `task` (in/out), `max_solutions` (in, `0`) | `init()` then `plan()` on a worker thread. **`max_solutions="0"` means "search until exhausted"**, not "no solutions" — pass `1` unless you want the full enumeration. |
| `MTCExecuteTask` | `task` (in) | Executes `task->solutions().front()` (`mtc_execute_task.cpp:72`) — the cheapest solution found, since MTC keeps solutions cost-ordered. You cannot currently choose a different one. |

### Stage builders

| Node | Ports | Builds |
|---|---|---|
| `MTCCurrentStateStage` | `task` (in/out) | `CurrentState` wrapped in a `PredicateFilter` named "Start State collision check" (`mtc_current_state.cpp:44-45`) that rejects a start state with colliding links. The rejection message names the links. |
| `MTCMoveToStage` | `stage_name` (in, `MoveToStage`), `group_name` (in, `manipulator`), `goal_name` (in, required), `planner` (in, required), `task` (in/out) | `MoveTo` to a **named** SRDF goal state only. There is no port for a joint map or a pose goal yet. |
| `MTCMoveRelativeStage` | `stage_name` (in, `MoveRelativeStage`), `group_name` (in, `manipulator`), `planner` (in, required), `ik_frame` (in, required), `min_distance` (in, required), `max_distance` (in, `-1.0` = disabled), `marker_ns` (in, `MoveRelativeStage`), `direction_frame_id` (in, `world`), `direction_vector` (in, **ignored**), `task` (in/out) | `MoveRelative`. **The direction is hardcoded to +z of `direction_frame_id`** (`mtc_move_relative.cpp:75-77`). See 1.3. |
| `MTCConnectStage` | `stage_name` (in, `ConnectStage`), `group_name` (in, `manipulator`), `timeout` (in, `1.0`), `mtc_planner` (in, required), `task` (in/out) | `Connect` for a single group. Note the port is **`mtc_planner`**, not `planner` (`mtc_connect.hpp:43`) — every other stage node calls it `planner` (`mtc_move_to.hpp:43`). This inconsistency is a live trap. |

## 1.3 The direction trap in `MTCMoveRelativeStage`

`MTCMoveRelativeStage` declares a `direction_vector` port of type
`geometry_msgs::msg::Vector3Stamped` (`mtc_move_relative.hpp:38`) — and never
reads it. The tick unconditionally builds a +z unit vector stamped in
`direction_frame_id` (`mtc_move_relative.cpp:75-77`).

So today the node can express exactly one family of motions: **+z in a frame you
name**. That is more useful than it sounds, because the frame choice is the
load-bearing decision (this is the frame-of-reference principle from the MTC
guide, section 4.3):

- `direction_frame_id="world"` (the default) → move straight up in world,
  regardless of tool orientation. Good for a lift; silently constrains the grasp
  to be compatible with a vertical motion.
- `direction_frame_id="panda_link8"` (or whatever your tool frame is) → move along
  the tool's own +z. Good for approach and retreat; stays correct for any grasp
  orientation.

What you **cannot** currently express: any negative direction (a retreat that
backs *out* along −z), any non-axis direction, or a joint-space delta. Setting
`direction_vector` in your XML will parse and be ignored — the silent-failure
shape that costs the most debugging time. Until Part 3 item 1 is fixed, get a
downward motion by pointing a frame the other way, not by negating a vector.

## 1.4 The plan/execute boundary, and what it costs

Each `MTCInitializeTask` … `MTCExecuteTask` run is a complete, independent MTC
task. The tree gives you sequencing and reactivity *between* tasks; MTC gives you
search *within* one. Choosing where to draw that line is the main design decision
in a tree.

**What you give up by splitting into several small tasks:** MTC's whole value is
searching over alternative solutions of a multi-stage problem — a grasp candidate
that fails at the lift stage gets pruned, and a different candidate is tried. Split
the grasp and the lift into two tasks and that feedback is gone: the first task
commits to its cheapest grasp, executes it, and the second task discovers the lift
is infeasible with nothing left to backtrack into.

**What you gain:** each task is short, plans fast, and the tree can react to
failure between tasks — retry, fall back, re-perceive, ask a human. That is exactly
what BehaviorTree.CPP is good at and what a monolithic MTC task cannot do.

The rule of thumb: **one task per span of motion that must succeed or fail as a
unit.** A pick is one task (approach, grasp, lift are jointly feasible or not). A
pick followed by a place is usually two, with a tree-level decision in between.

## 1.5 What the BT layer does not expose yet

The library wraps a deliberate subset of MTC. Absent today, all of it reachable
only from C++:

- **Containers.** No `SerialContainer`, `Alternatives`, `Fallbacks`, or `Merger`
  node. Every stage builder appends to the task's top-level serial container, so a
  tree describes a flat stage list. BT `Fallback` is *not* a substitute — it
  chooses between subtrees at tick time, whereas MTC `Fallbacks` chooses between
  stages during the search.
- **Generators beyond `CurrentState`.** No `GenerateGraspPose`,
  `GeneratePlacePose`, `FixedState`.
- **`ComputeIK`.** With no pose generator and no IK wrapper, there is no way to
  express a Cartesian-target motion from a tree.
- **`ModifyPlanningScene`.** No attach/detach, no collision-matrix edits — so no
  grasping in the MTC sense.
- **Cost terms and solution selection.** No `setCostTerm`, and execution always
  takes the cheapest solution.
- **Monitored stages.** Nothing to call `setMonitoredStage` with, which is
  consistent, since there are no monitoring generators either.

A pick-and-place tree is therefore not yet expressible. Section 2 is how to close
that gap.

## 1.6 Debugging

- **Groot2** connects on port `1667` (`mtc_samples/config/sample_mtc_executor.yaml:5`).
  It shows you the tree, which tells you which stage builders ran — not what MTC
  did inside `MTCPlanTask`.
- **The MTC RViz panel** shows the stage tree with per-stage solution and failure
  counts, which is where you diagnose an empty solution set. The demo launch file
  loads the MTC rviz config for exactly this reason.
- **Read the failure counts before anything else.** A stage showing 0 solutions
  and 0 failures never ran — its interface was never fed. A stage showing 0
  solutions and many failures ran and rejected everything, and the failure comments
  say why. This distinction is in the MTC guide, section 7, and it is the single
  highest-value debugging habit.
- **`MTCPlanTask` logs the `MoveItErrorCode` on failure** and the
  `InitStageException` if `init()` fails. An init failure means the stage list you
  assembled does not compose — usually a missing generator at the front.

---

# Part 2 — Adding a node

## 2.1 The four edits

Adding a node touches exactly four places. Miss the last one and the node builds
but does not exist at runtime:

1. `mtc_bt/include/mtc_bt/mtc_<thing>.hpp` — the class and its `providedPorts()`.
2. `mtc_bt/src/mtc_<thing>.cpp` — the tick.
3. `mtc_bt/CMakeLists.txt:18` — add the `.cpp` to the `mtc_plugin` target's source
   list.
4. `mtc_bt/src/mtc_plugin.cpp:31-39` — a `registerNodeType<>` line inside
   `BT_REGISTER_ROS_NODES`.

Registration must live in a `.cpp` (`mtc_plugin.cpp:29`), and the whole library is
built with `BT_PLUGIN_EXPORT` (`mtc_bt/CMakeLists.txt:33`).

## 2.2 Choosing the base class

- **`BT::SyncActionNode`** for anything that finishes within one tick: every
  planner factory and every stage builder. Building an MTC stage is pure
  construction — no I/O, no planning — so it is always synchronous.
- **`BT::StatefulActionNode`** for anything that blocks: planning and execution.
  See 2.5.

If your node needs the ROS node handle — to load a robot model, construct a
`PipelinePlanner`, or talk to a service — take `const BT::RosNodeParams&` as a
third constructor argument, store it, and register it with the `params` argument
(`mtc_plugin.cpp:31-32`). Nodes without that argument are registered without it
(`mtc_plugin.cpp:33`). Getting this wrong is a registration-time error, not a
compile error.

## 2.3 Ports that carry MTC objects

MTC objects move between nodes as `shared_ptr`s in typed ports:

```cpp
BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask)
BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortPlanner)
```

Two conventions to follow, one to understand:

- **Declare port-name constants as `static constexpr auto kPortX`** in the private
  section and use them in both `providedPorts()` and the tick. Every existing node
  does this. It is what stops a typo from becoming a silent missing input.
- **When you write a planner to a port, cast to the interface type.** The concrete
  `std::shared_ptr<solvers::CartesianPath>` is not the port's declared type, and
  the blackboard's type check is exact. Existing nodes `static_cast` to
  `PlannerInterfacePtr` at `setOutput`.
- **`BidirectionalPort<TaskPtr>` on stage builders is a convention, not a
  mechanism.** The stage builders mutate the `Task` through the shared pointer;
  the `setOutput` call at the end of each tick writes back the same pointer value
  and changes nothing. Keep doing it for consistency with the existing nodes, but
  do not believe the write-back is what propagates your stage.

## 2.4 The tick pattern

Every stage builder has the same skeleton, and it is worth copying exactly:

```cpp
BT::NodeStatus MTCThingStage::tick()
{
  // ports with defaults: read without checking, the default is guaranteed
  std::string stage_name, group_name;
  getInput<std::string>(kPortStageName, stage_name);
  getInput<std::string>(kPortGroupName, group_name);

  // ports without defaults: check, log, fail
  TaskPtr task;
  if (!getInput<TaskPtr>(kPortTask, task))
  {
    RCLCPP_ERROR(kLogger, "Missing required input: [%s]", kPortTask);
    return BT::NodeStatus::FAILURE;
  }

  auto stage = std::make_unique<stages::Thing>(stage_name, planner);
  stage->setGroup(group_name);
  stage->properties().configureInitFrom(Stage::PARENT);
  task->add(std::move(stage));

  setOutput(kPortTask, task);
  return BT::NodeStatus::SUCCESS;
}
```

Points that are not obvious:

- **Return `FAILURE` on a missing input; do not throw.** `MTCConnectStage` throws
  `BT::RuntimeError` instead (`mtc_connect.cpp:34`), which aborts the whole tree
  rather than letting a `Fallback` handle it. The `FAILURE` form is the one to
  copy; the throwing form should be brought into line.
- **Use `task->add()`, not `task->insert()`.** `MTCMoveRelativeStage` uses
  `insert` (`mtc_move_relative.cpp:80`); with the default position argument the two
  are equivalent, so this is a consistency wart rather than a bug — but `add` says
  what is meant.
- **`configureInitFrom(Stage::PARENT)` is what lets task-level properties reach
  your stage.** Without it, a property set on the `Task` is invisible to the stage.
- **A logger per file** in an anonymous namespace: `static const auto kLogger =
  rclcpp::get_logger("MTCThingStage")`. Copy the name from your class — two
  existing files have copy-pasted the wrong name (Part 3, item 5).

## 2.5 Blocking work: the threading contract

`MTCPlanTask` and `MTCExecuteTask` are the only blocking nodes, and they share one
pattern. If you add a third (a perception call, a long service), follow it:

- `onStart()` validates inputs, sets `planning_status_ = RUNNING`
  (`mtc_plan_task.cpp:50`), spawns a `std::thread` (`mtc_plan_task.cpp:53`), and
  returns `RUNNING`.
- The worker writes its result into the status member and exits.
- `onRunning()` just returns that member. It does no work.
- `onHalted()` calls `task_->preempt()` (`mtc_plan_task.cpp:65`) so MTC unwinds its
  own search.
- The destructor joins the thread. Nodes outlive individual ticks but not the
  tree, so a thread that outlives its node is a crash.

The status member is written by the worker and read by the tick thread without
synchronisation. It happens to be a plain enum, so this works in practice on the
platforms in use, but it is not formally race-free — if you copy this pattern,
consider `std::atomic<BT::NodeStatus>`.

## 2.6 Install and discovery

The plugin does **not** install to `lib/`. It installs to
`share/mtc_bt/bt_plugins` (`mtc_bt/CMakeLists.txt:56-58`), because the executor
finds plugins by package-relative directory, listed in
`mtc_samples/config/sample_mtc_executor.yaml:8`. Behavior trees are found the same
way (`sample_mtc_executor.yaml:13`) — every `.xml` in a listed directory is loaded
at startup, and the server prints each plugin and tree as it loads them.

To add a tree, drop the `.xml` into `mtc_samples/behavior_trees/` and rebuild; no
registration needed. To use it, name its `BehaviorTree ID` as the action goal:

```bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: TestMtc}"
```

The action name comes from `sample_mtc_executor.yaml:3`.

## 2.7 Keeping the `TreeNodesModel` current

Each tree XML carries a `<TreeNodesModel>` block that Groot2 reads. It is written
by hand and is already incomplete — `test_mtc.xml` omits `MTCExecuteTask`,
`MTCConnectStage`, and most ports of the nodes it does list. Groot2 can regenerate
it from a running server; prefer that over hand-editing, and regenerate after
adding ports.

---

# Part 3 — Known gaps

Found while writing this guide, ordered by how much they cost a user. These are
defects in the library, not in the documentation of it.

1. **`direction_vector` is declared but never read**
   (`mtc_move_relative.hpp:38`, `mtc_move_relative.cpp:75-77`). The node can only
   move +z of a named frame. Wiring the port up is the single highest-value fix in
   the library: it turns one hardcoded motion into the general case.

2. **Panda controller names are hardcoded in three places**
   (`mtc_init_task.cpp:47`, `mtc_move_to.cpp:67`, `mtc_move_relative.cpp:78`). Any
   robot that is not the demo Panda cannot execute. These belong on a port, or on
   the task and inherited via `configureInitFrom(Stage::PARENT)`.

3. **`MTCConnectStage` names its planner port `mtc_planner`**
   (`mtc_connect.hpp:43`) while every other node uses `planner`
   (`mtc_move_to.hpp:43`). Rename it; the cost of the inconsistency is paid by
   every tree author, once each.

4. **Input validation in the planner factories does nothing.** The guard reads
   `if (!getInput(...) && value <= 0.001 && value >= 1.0)`
   (`mtc_cartesian_planner.cpp:36`) — the two range tests cannot both hold, and
   `getInput` succeeds anyway because the port has a default, so the branch is
   dead and out-of-range values pass through unchecked. The same shape appears in
   `mtc_pipeline_planner.cpp`. It also reads the value before it is assigned on
   the failure path.

5. **Two loggers carry the wrong name.** `mtc_cartesian_planner.cpp` logs as
   `MTCCurrentState` and `mtc_pipeline_planner.cpp` logs as `MTCConnectStage`.
   Cheap to fix, and expensive not to when reading a log.

6. **A debug print in a destructor.** `~MTCPlanTask` writes
   `~MTCPlanTask****************************` to `std::cout`
   (`mtc_plan_task.hpp:38`).

7. **`MTCConnectStage` throws where the others return `FAILURE`**
   (`mtc_connect.cpp:34`), which takes down the tree instead of letting it react.

8. **No tests.** Neither package has any; `BUILD_TESTING` wires up `ament_lint_auto`
   only. The stage builders are pure functions from ports to a `Task` structure —
   they are unusually easy to test without a robot, by ticking a node and asserting
   on the resulting stage list.

9. ~~**`moveit_bt.repos` pins `moveit_task_constructor` to `humble`** while the
   development checkout tracks `ros2`.~~ **Fixed** — it now points at the
   `MarqRazz` fork on `marq-devel`. The file also carries `moveit2` on `main`,
   because MoveIt has no binary release for Lyrical: only `moveit_common`,
   `moveit_msgs`, `moveit_configs_utils` and the `moveit_resources_*` set are
   published, so the core libraries MTC links against must be built from source.
