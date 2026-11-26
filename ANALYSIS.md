# ROS 1 vs ROS 2 ABB driver: safeguards lost in the port

This fork hardens `abb_ros2` (PickNikRobotics, `rolling` @ e6486f7) by porting
back the field-validated safeguards of the ROS 1 `abb_robot_driver`
(ros-industrial). Both drivers sit on the **same** ABB libraries, which this
fork does not modify:

| layer | ROS 1 | ROS 2 (this repo) |
|---|---|---|
| EGM protocol, UDP server | `abb_libegm` | `abb_libegm` (2c9350a) |
| RWS HTTP client | `abb_librws` | `abb_librws` (9ba0837) |
| EGM/RWS managers, motion data | `abb_egm_rws_managers` | same package (7e554e5) |
| glue | `abb_egm_hardware_interface` (ros_control), `egm_controller_stopper`, `abb_egm_state_controller`, `abb_rws_state_publisher`, `abb_rws_service_provider` | `abb_hardware_interface` (ros2_control), `abb_rws_client` (one node) |

So every difference in behaviour comes from the glue layer, and every fix here
lives in `abb_hardware_interface`, `abb_rws_client` or `abb_bringup`.

Goal: a driver that is robust and safe in normal operation, as the ROS 1 driver
was, and where rare cases **fail safe** (hold, deactivate, an operator re-arms).
A few general mechanisms are preferred over per-case logic.

Line references: **ROS 1** = `abb_robot_driver` master,
`abb_egm_hardware_interface/src/egm_hardware_interface.cpp` unless noted;
**ROS 2** = this repository at e6486f7; **libegm** = `abb_libegm` 2c9350a;
**managers** = `abb_egm_rws_managers` 7e554e5; **r2c** = ros2_control 4.48.1 (Jazzy).

## Library facts the safeguards rest on

1. **libegm replies with whatever it last got.** For each inbound message the
   UDP callback publishes the inputs, then, if motors are on, RAPID is running
   and EGM is running, waits up to `WRITE_TIMEOUT_MS` = 24 ms for a new output.
   Without one it replies with the **previous output** (libegm
   `egm_controller_interface.cpp:188-192`, `.h:157`). Only a session's first
   message (seqno 0) seeds the output from feedback (`egm_base_interface.cpp:273-299`).
2. **libegm serves one datagram at a time.** While it waits those 24 ms, the
   robot keeps sending every 4 ms, so datagrams queue in the socket and are
   later read **old first** (libegm #53, #60). libegm #49: the interface expects
   one write per received message.
3. **The managers count reads, not time.** A channel goes inactive after 5 reads
   with an unchanged sequence number (managers `egm_manager.cpp:47,111-121`), and
   `write()` forwards nothing while it is inactive (`:159-165`). ROS 1 reads once
   per message (50 ms timeout), so that is about 250 ms of silence. A fixed-rate
   ROS 2 controller manager at 250-500 Hz makes it **10-20 ms**: ordinary
   jitter flips the channel, and libegm replays the last output meanwhile.
4. **Per-read transitions and the raw input are public**:
   `was_activated_or_deactivated`, `egm_client_state_changed`,
   `rapid_execution_status_changed`, and `input` (header seqno, status,
   utilisation) in `MotionData::...::egm_channel_data` (`egm_manager.cpp:128-137`).
   `EGMManager::read()` also returns true for a duplicate (`:135-153`).
5. **The managers throw on NaN/Inf or out-of-bounds commands**
   (`egm_manager.cpp:373-389`); r2c catches the exception and puts the hardware
   into ERROR.
6. **r2c calls `read()` in ACTIVE *and* INACTIVE, and `write()` only in ACTIVE.**
   `write()` returning `DEACTIVATE` makes the component INACTIVE and deactivates
   every controller claiming its command interfaces
   (`hardware_component.cpp:372-437`, `resource_manager.cpp:2525-2620`).
7. **librws logs out only in `~RWSClient()`** (`rws_client.h:357-360`), and each
   `RWSManager` holds two interfaces, so up to two sessions (`rws_manager.cpp:55-71`).

## Observed behaviour (stock driver + the imported commits)

Measured against a RobotStudio virtual controller (IRC5, RW 6.16) over Wi-Fi,
and with an IRC5 RWS+EGM simulator.

| id | behaviour | measured |
|---|---|---|
| O1 resume snap | after a stall, RAPID restarted without PP-to-Main resumes the same EGM session; the arm drives to the driver's stale last command, then re-seeds from libegm's queued old messages | 9-27 deg at up to 43 deg/s, no goal; then a 4-12 deg jump at up to 73 deg/s |
| O2 post-gap catch-up | after an inbound gap of 10 ms or more, the first reply carries everything the open-loop JTC advanced during the gap | 0.65-11.7 deg in one 4 ms packet; controller fault "required torque too high", axis 1 |
| O3 stale session | a supervisor trusting the RWS-polled `egm_action` re-activates on a dead or resumed session | across RAPID stop, motors off, comm timeout |
| O4 speedRef overshoot | the JTC velocity goes out as EGM speedRef (`pos_corr_gain` 0.6); fast moves overshoot | 10 % at 60 deg/s, 43 % (8.6 deg) at 120 deg/s with reversal; position-only 0.3-0.8 %, +140 ms lag |
| O5 loss grace | a 50 ms EGM-loss grace trips every ~6 s on Wi-Fi, 0 packets lost (60-310 ms bunching) | 250 ms still trips; 500 ms holds |
| O6 RWS starvation | at the launch default of 5 Hz polling over a slow link, no `rws_client` service answers, including `stop_egm` | indefinite; fine at 2 Hz |
| O7 RWS sessions | repeated driver restarts fill the controller's session table | "503 Too many sessions 70/70"; the hardware then blocks "waiting indefinitely" |
| O8 activation zero jump | the first read of a session is discarded, so command = 0 | axis 5 from 30 deg to 0 deg (imported fix; upstream #44/#78) |

The validation link is deliberately laggy: Wi-Fi + WireGuard, RTT 8-130 ms,
bursts up to ~900 ms, 1.6 % loss, EGM inter-arrival bunching up to 213 ms.
**A laggy link may degrade tracking but must never cause unsafe motion.**

## Comparison, ranked by safety impact

### 1. Idle = hold at actual [HIGH]

- **ROS 1** `:231-238`: every `read()` sets `command.position = state.position`
  and `command.velocity = 0`; a running controller overwrites it in `update()`.
- **ROS 2** `abb_hardware_interface.cpp:297-307`: `read()` only reads. The last
  command any controller wrote is resent forever.
- **Risk.** Whenever no controller writes, the robot is driven to an old target
  (O1).
- **Port.** The same seed in `read()`. While the hardware is INACTIVE, r2c still
  calls `read()` (fact 6), so `read()` also *writes* hold-at-actual then. libegm
  never replays a stale command and never blocks 24 ms, so no backlog builds
  (facts 1, 2; this is O1's second half).

### 2. Session changes: deactivate, restart from actual [HIGH]

- **ROS 1** `:189-247` + `resetNeeded()` → `cm.update(..., reset)`
  (`egm_hardware_interface_main.cpp:123`): any channel (de)activation, EGM client
  state change or RAPID state change resets all controllers (and the soft-limit
  state, `:246`). `prepareSwitch()` `:281-337` refuses to start controllers while
  EGM is inactive or not running. `egm_controller_stopper.cpp:133-199` stops
  controllers when EGM goes inactive.
- **ROS 2.** Nothing upstream. The imported deactivate-on-loss commit covers the
  stopper, but:
  - it applies its grace to *every* cause, including what the robot itself
    reports, so a stop + resume inside the grace goes unnoticed;
  - it does not see a new session (seqno restart);
  - `on_activate()` accepts any message, not a running session;
  - a controller can be activated while EGM is not running.
- **Risk.** O3, O1.
- **Port.** ROS 2 has no reset hook from the hardware; the equivalent is to
  self-deactivate (the claiming controllers go too), then activate again from
  actual.
  - Robot-reported changes (EGM ≠ RUNNING, RAPID ≠ RUNNING, motors off, seqno
    restart) deactivate **at once**. Only *silence* (no new sequence number,
    measured in time, not reads: fact 3) gets `egm_loss_grace_ms`.
  - `on_activate()` drains queued messages, seeds from the newest, and requires
    a running session.
  - `prepare_command_mode_switch()` refuses command claims otherwise.

### 3. Per-packet step limit [HIGH, parameter, default off]

- **ROS 1.** One write per received message (`egm_hardware_interface_main.cpp:117-125`,
  period = the measured wait), so its per-period velocity clamp *is* a per-packet
  clamp.
- **ROS 2.** The imported soft-limit clamp is per controller cycle against the
  last *written* command. During a gap the writes are dropped (fact 3), but the
  reference still advances, so the first reply after the gap carries the whole
  catch-up (O2).
- **Port.** `max_step_per_packet` (scale of max_velocity × EGM period, 0 = off):
  - the command is limited relative to the last command **sent**, i.e. the last
    write before a new sequence number arrived;
  - an inbound gap longer than the grace holds and deactivates (item 2), never
    extrapolates.

  Off by default until the owner decides.

### 4. Velocity feed-forward [HIGH impact, parameter, default unchanged]

- **ROS 1.** The rigs it ran on used a position-only JTC; velocity stays 0
  through item 1.
- **ROS 2.** It requires position + velocity command interfaces
  (`abb_hardware_interface.cpp:36-55`), and the bringup JTC claims both
  (`abb_controllers.yaml`), so speedRef is fed (O4).
- **Port.** `velocity_command_mode`: `feedforward` (default) | `position_only` |
  `scaled` (`velocity_feedforward_scale`). The criterion for choosing is no
  oscillation on step trajectories, from a tuning matrix on the controller.

### 5. Joint limits from the robot controller [MEDIUM]

- **ROS 1** `:621-705`: the ROS limits are intersected with the limits the robot
  controller reports; acceleration limits are deliberately ignored (`:670-686`).
- **ROS 2** (imported commit): URDF limits only. A command outside the
  controller's bounds throws (fact 5).
- **Port.** Intersect the URDF limits with the `MotionData` bounds, both minus
  the margin.

### 6. RWS client threading [MEDIUM]

- **ROS 1.** State publisher and service provider are separate processes (one
  `ros::spin()` each), so polling never delays a service.
- **ROS 2.** One node (`rws_client_node.cpp:10-26`): the polling timer
  (`rws_state_publisher_ros.cpp:85`), every service and both subscriptions share
  the default MutuallyExclusive group. When a poll outlasts its period, services
  never run (O6). `polling_rate` is already a parameter
  (`abb_rws_client.launch.py:51`).
- **Port.** Separate callback groups for the timer, the services and the
  subscriptions. The RWSManager mutexes still serialise HTTP.

### 7. RWS sessions [MEDIUM]

- **ROS 1.** The hardware interface gets the controller description through a
  ROS service, with a 30 s timeout (`egm_hardware_interface_main.cpp:156-201`),
  so it opens no RWS session.
- **ROS 2.** `on_init()` opens its own `RWSManager` with a hard-coded nickname
  `"IRB1200"` and an infinite wait (`abb_hardware_interface.cpp:100-101`). Any
  unclean exit leaks the sessions (fact 7, O7).
- **Port.** Log out explicitly as soon as the description is read, and in
  `rws_client` on SIGINT/SIGTERM. Use a bounded wait, as ROS 1 did.

### 8. Diagnostics [LOW]

- **ROS 1** `:221-229`: a warning when EGM utilisation exceeds 100 %.
  `:170-181`: logs for feedback timed out / resumed.
- **ROS 2.** Neither.
- **Port.** Both, plus `egm_loss_grace_ms` exposed in the xacros and the launch
  file (O5).

### No port needed / deferred

- **No port needed.** RWS service preconditions match ROS 1 one-to-one, and the
  EGM-settings services are identical. `egm_controller_stopper` is covered by
  DEACTIVATE (broadcasters claim nothing, so they keep running, like ROS 1's
  `ok_to_keep_running`).
- **Deferred.**
  - A blocking EGM-synchronous `read()` would stall every other hardware on the
    same controller manager; sequence-number dedupe + time-based supervision
    gives the safety part instead.
  - EGM state as state interfaces (ROS 1 `abb_egm_state_controller`).
  - A poll failure publishes the all-false default state (the same in ROS 1).

## PRs (into `jazzy`)

| PR | content | items |
|---|---|---|
| 1 | the four imported commits + this analysis | O8 |
| 2 | Jazzy CI + fake-robot EGM test harness, tests for the imported commits | - |
| 3 (A) | idle and stale safety: hold at actual in `read()` (active and inactive), drain on activate | 1 |
| 4 (B) | session supervision: immediate deactivation on robot-reported changes or seqno restart, time-based grace on silence, seqno dedupe, activation/switch guards, diagnostics, grace in xacro/launch | 2, 8 |
| 5 (C) | per-sent-packet step limit + hold-and-fault on a gap (default off) | 3 |
| 6 (D+E) | command shaping: velocity command mode (default unchanged), URDF ∩ controller-reported joint bounds | 4, 5 |
| 7 (F) | rws_client callback groups + RWS session hygiene | 6, 7 |

## Status

| PR | content | merged |
|---|---|---|
| #1 | imported fixes (zero jump, prefixed joints, EGM-loss deactivation, soft limits) + this analysis | yes |
| #2 | fake-robot EGM test harness, `scripts/test_jazzy.sh`, pinned dependencies | yes |
| A | idle/stale safety: no stale hold target, no velocity residue, hold what was sent, answer EGM while inactive, drain on activate | see PR |
| B | session supervision: robot-reported stops at once, grace only for silence, seqno dedupe, activation/switch guards, diagnostics | see PR |
| C | per-sent-packet step limit (`egm_max_step_scale`, default off) | see PR |
| D+E | `velocity_command_mode` (default unchanged), joint bounds never wider than the controller's | see PR |
| F | rws_client callback groups, one RWS session pair, bounded RWS wait in the hardware | see PR |

### Owner decisions pending
- `egm_max_step_scale` (default off): enable it, at 1.0 (max velocity per EGM sample)?
- `velocity_command_mode` (default `feedforward`): position_only / scaled, after a tuning matrix on the robot (no oscillation on step trajectories).
- `egm_loss_grace_ms` (default 50 ms): a Wi-Fi link needed 500 ms; a longer grace only delays the stop on silence.

### Known limits (not fixed here)
- **Lost replies.** If the link drops replies while robot messages still arrive (e.g. at the start of a blackout), the robot can see more than one step at once. The step limit is relative to what the driver sent; the robot's own EGM speed supervision (`\MaxSpeedDeviation`) is the complement.
- **The first reply after a gap** can carry a write from the gap's first 10 ms (the managers forward writes until 5 reads without a new message). Bounded by 5 controller cycles of motion, or one step with the step limit.
- **Deferred:** EGM state as state interfaces (ROS 1 `abb_egm_state_controller`); an RWS poll failure publishes the all-false default state (as ROS 1).

### Simulator e2e (IRC5 RWS+EGM simulator, laggy EGM link)
60 s idle, then a controller stop mid-move, a RAPID stop mid-move + restart + re-arm, then 25 moves at 60-120 deg/s over netem on the EGM port: 30 +- 30 ms delay each way (paretonormal), 1.6 % loss, 1 % duplicates, reordering, and blackouts of 200-900 ms every 3-8 s. 500 Hz controller manager, `egm_loss_grace_ms` 500. "stock" = the imported fixes (#1); C and F with `egm_max_step_scale` 1.0, Fdef = F with defaults. "Sent step" = between consecutive replies in the driver's own sequence order (what the driver sent); "received step" = in arrival order, which includes the link's reordering. Limit = max velocity x 4 ms. One run per variant (random blackouts): compare orders of magnitude, not single digits.

| variant | idle distinct/replies | travel after JTC stop [deg] | snap after re-arm (RAPID) [deg] | laggy: max sent step [deg] (x limit, n over/n) | laggy: max received step [deg] (n over/n) | faults | travel fault->re-arm [deg] | snap after fault re-arm [deg] | sim errors |
|---|---|---|---|---|---|---|---|---|---|
| stock | 1/15003 | 28.705 | 0.0 | 22.91 (17.802x, 132/5262) | 38.565 (2387/5419) | 2 | 1.283 | 0 | 0 |
| A | 1/14996 | 25.64 | 0.0 | 15.502 (12.503x, 180/4900) | 16.276 (2241/5048) | 0 | 0 | 0 | 0 |
| B | 1/15001 | 19.015 | 0.0 | 34.454 (49.219x, 180/5064) | 34.714 (2223/5208) | 1 | 4.187 | 0 | 0 |
| C | 1/14998 | 21.758 | 0.0 | 2.542 (2.0x, 7/5265) | 12.137 (2012/5416) | 2 | 2.998 | 0 | 0 |
| F | 1/15001 | 26.928 | 0.0 | 4.431 (3.0x, 5/5083) | 14.76 (1809/5226) | 2 | 0.584 | 0 | 0 |
| Fdef | 1/15004 | 28.648 | 0.0 | 2.401 (1.668x, 6/5107) | 14.27 (1914/5227) | 2 | 1.773 | 0 | 0 |

RWS (rws_client at 5 Hz polling, 150 ms each way on the RWS port, `get_speed_ratio` x 3, two driver starts): stock answered 1 of 6 calls (8.1 s), 5 timed out after 20 s; F answered 6 of 6 in 0.5-0.6 s. RWS sessions over the two starts: stock 8, F 4 (both logged out on a clean shutdown).
