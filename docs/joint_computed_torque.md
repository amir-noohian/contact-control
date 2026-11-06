# Joint-space inverse dynamics with PID

Build and run using the robot configuration selected in your shell:

```bash
cmake -S . -B build -DCONTACT_CONTROL_BUILD_TESTS=ON
cmake --build build --target dyncomp_control joint_computed_torque_test -j2
ctest --test-dir build -R '^joint_computed_torque$' --output-on-failure
source scripts/setup_slax.sh
./build/bin/dyncomp_control
```

For the alternative computed-torque controller, which moves feedback into the
inverse-dynamics acceleration command, build and run:

```bash
cmake --build build --target comptorq_control -j2
./build/bin/comptorq_control
```

This variant feeds current measured position and velocity into Dynamics and
uses `ddq_input = ddq_desired + Kp_acc*(q_desired - q_current) +
Kd_acc*(dq_desired - dq_current)`. The per-joint acceleration-domain gains
follow the ratios of the SLAX `joint_position_control` gains. They are
normalized so the maximum `Kp_acc` is `100 s^-2` (values
`[75, 100, 40, 20]`) and maximum `Kd_acc` is `3.3 s^-1` (ratios
`[1, 8/8.3, 3.3/8.3, 0.8/8.3]`). These acceleration gains are not the raw
torque-domain config values and should be tuned cautiously on hardware. The
resulting dynamics torque is passed to the WAM
through `trackReferenceSignal`; Barrett's position controller does not add
another PD torque during this trajectory. On completion, the torque reference
is replaced directly with a configured-position-controller home trajectory.
Both controllers use libbarrett's default home-motion profile.
`dyncomp_control` disconnects its model feedforward before starting that home
trajectory. After the move completes, both controllers leave the position
reference connected to hold the home position; they disconnect it only after
Shift-idle. Fault or interrupted paths still disconnect immediately.

During only the `dyncomp_control` computed-torque trajectory, its configured
joint-position Kp and Kd gains for joints 1–4 are scaled by 0.2, preserving
their configuration ratios. For the current SLAX configuration, this gives
Kp `[150, 200, 80, 40]` and Kd `[1.66, 1.6, 0.66, 0.16]`. Joints 5–7 are
unchanged. The configured gains are restored before the return-home move;
`comptorq_control` and the shared robot configuration are unaffected.

The first Enter authorizes a move to all-zero joint angles at 0.2 rad/s and
0.2 rad/s². The second starts a 30-second trajectory. Joints 2 and 4 follow
identical 0.4 rad, 0.2 Hz sine waves around zero, with 3-second quintic
startup/shutdown envelopes. All other joint references stay zero. The robot
returns home on normal completion; Shift-idle exits. Shift-idle also interrupts
the trajectory. Tracking data is saved as described below.

Desired position is passed to `wam.trackReferenceSignal(reference.position)`.
Barrett's built-in joint position controller supplies feedback torque using
`joint_position_control` from the selected robot configuration (kp, ki, kd,
control_signal_limit and integrator_limit). There is no custom PID or gain override.
The selected SLAX config currently has zero Ki, so its feedback is PD until Ki is configured.

Desired position, velocity and acceleration are also sent to `Dynamics(true)`.
The model's feedforward is applied through `wam.input`. Built-in gravity remains
enabled: the feedforward adapter subtracts Barrett gravity on joints 1–4 so it
is not counted twice. Joints 5–7 retain Barrett gravity and position feedback.

```
wam.input[0:4] = limited_model_torque - Barrett_gravity[0:4]
total_torque[0:4] = limited_model_torque + configured_position_PID
```

The model contribution is limited to 50/40/30/30 Nm. These are feedforward-only
limits, not limits on the sum with PID. The configured position controller
limits its own contribution separately. No custom integral state is maintained.
Default `Dynamics()` retains zero-acceleration behavior for the old compensation
programs. Beta and regressor values are unchanged.

A non-finite input, joint-bound violation, speed above 1.5 rad/s, or tracking
error above 0.35 rad latches a fault. The feedforward correction becomes zero;
main disconnects `wam.input` and stops position tracking. On normal completion,
the model feedforward is disconnected and the WAM follows the home trajectory.
A fault does not initiate a return move. Shift-idle interrupts execution.

The existing gravity mismatch still requires hardware investigation. Software
tests do not establish model accuracy or hardware stability.

## Tracking log

Each run creates `data/joint_computed_torque/joint_tracking_0001.csv` in the
project directory, without overwriting earlier runs. The folder is created
automatically, and the path is independent of the working directory. Run numbers
increase automatically (0001, 0002, etc.), starting above the highest existing
number. New filenames are reserved exclusively to prevent overwrites. Libbarrett `TupleGrouper`, `PeriodicDataLogger` and
`log::RealTimeWriter` capture a binary record every execution-manager cycle.
CSV conversion uses `log::Reader` outside the real-time loop after motion stops.
Recording covers the sine tracking phase, including a fault/interruption up to
shutdown, but excludes initial positioning and the return-home move.

Columns are time in seconds, followed by all joints for each group:
`q_des_rad`, `q_meas_rad`, `dq_des_rad_s`, `dq_meas_rad_s`,
`ddq_des_rad_s2`, `q_error_rad`, `dq_error_rad_s`. Each name has a `_j1`
through `_j4` or `_j7` suffix. Position and velocity errors are desired minus
measured. Only desired acceleration is recorded; no measured acceleration,
acceleration estimate, or acceleration error is calculated.
If CSV export fails, the binary recovery path is printed and retained.

## Plotting

Use `scripts/plot_joint_tracking_csv.py` with Python and matplotlib (available
in the `id` Conda environment). With no file argument it selects the most
recently modified tracking CSV. Both numbered and older random-suffix logs work.

```bash
source /home/hela/amir/miniforge3/etc/profile.d/conda.sh
conda activate id
python scripts/plot_joint_tracking_csv.py --joints 2 4
python scripts/plot_joint_tracking_csv.py data/joint_computed_torque/joint_tracking_0001.csv
```

By default only joints 2 and 4 are shown in one figure, with one row per joint
and columns for position, velocity, position error, velocity error and desired
acceleration. Errors are desired minus measured; no measured acceleration is
inferred. Use `--joints` to select a different set of joints.

The script only opens the figure; it does not save plots or metrics files.
`--show` is accepted for compatibility but is no longer necessary.
