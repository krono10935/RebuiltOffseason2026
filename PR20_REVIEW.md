# Code review: PR #20 "The drivetrain is ready"

`drivetrainRefactor` → `drivetrain` · reviewed at `e155336`

Strict review draft. 🔴 = blocker, 🟠 = high, 🟡 = medium, unmarked = nit.

Top priorities: (1) robot keeps driving after commands end, (2) wheel radius 10× too big, (3) MK5 drive/steer configs swapped, (4) FusedCANcoder ratios wrong. A quick on-robot check (command 1 m/s, point the wheels at 90°) would have caught most of these.

## `frc/robot/subsystems/drivetrain/DrivetrainReal.java`

- **L34**: 🟡 `DrivetrainInputs` is nested in the subclass but used by the abstract base `Drivetrain` (and therefore by `DrivetrainSysID`). Move it to the base / its own file. Also `DrivetrainReal` is a misleading name, since it is what runs in sim too.

- **L53**: Nit: `Supplier<Double>` boxes every loop; use `DoubleSupplier`.

- **L133**: 🟠 `toSwerveModuleStates(new ChassisSpeeds())` returns the kinematics' cached headings, which are never updated (this is the only call), so they are all 0°. Every `stop()` snaps all modules to 0°. Stop at 0 m/s with the current module angles instead.

- **L210**: 🟠 Field-relative uses the raw gyro, but `HomeToSupplierChassisSpeedsCalculator` uses the pose-estimator heading. These diverge (especially after `resetGyro`). Pick one source of truth.

  Also: on red the gyro is reset to 180° but the joystick vector is never flipped, so stick-forward drives toward the driver.

- **L248**: 🔴 **Robot keeps driving after the command ends.** `Commands.run(...)` never finishes, so `.andThen(stopCommand())` never runs; when this is interrupted the sequence is cancelled and `goalSpeeds` keeps its last value. Because `periodic()` calls `drive()` every loop, the robot keeps moving at the last joystick speed (and resumes it after disable → enable). Same for the two other drive commands and for interrupted PathPlanner paths / `drivetrain.idle()` in auto.

  Suggest `Commands.run(...).finallyDo(this::stop)`, clearing `goalSpeeds` on disable, and a default command.

- **L285**: 🟠 `isFieldRelative = false`: translation becomes **robot-relative** while auto-aiming, so the driver's stick meaning changes as the robot rotates. Should almost certainly be field-relative.

- **L287**: Nit: typo `DriveAndHomeTeAngle`.

## `frc/robot/subsystems/drivetrain/DrivetrainSysID.java`

- **L41**: 🔴 This class never calls `GeneralRobotState.setPoseEstimator`, so `Drivetrain.periodic()` → `updatePoseEstimator` and `Robot.robotPeriodic()` → `getEstimatedPose()` NPE every loop. It also can't be swapped in, since `RobotContainer`/`SuperStructure` are typed `DrivetrainReal`.

- **L53**: 🟠 Log consumer is `null`, `SignalLogger` is explicitly stopped in `Robot.initializeLogging`, and nothing logs drive voltage, position or rotor velocity. There will be no usable data for SysID. Either enable SignalLogger for SysID runs or log these via AdvantageKit.

## `frc/robot/subsystems/drivetrain/constants/DrivetrainConstants.java`

- **L19**: 🔴 `null` constraints: `TrapezoidProfile` stores null and NPEs on the first `calculate()` (checked WPILib 2026.2.1). `driveAndHomeToAngleSupplier` crashes robot code. Gains are also all 0. And as a shared `static` instance, every calculator shares state.

## `frc/robot/subsystems/drivetrain/module/constants/SwerveModulesMK5.java`

- **L26**: 🟠 Steer `kI = 5` with no iZone (Phoenix 6 doesn't support it, and `PIDGains` drops it) risks windup/oscillation. The steer kV is irrelevant in plain position control.

- **L86**: 🔴 `0.508` m radius is a 1 m wheel. A 4" wheel is `0.0508` m. Every velocity and position is 10× off (odometry, commanded speeds, PathPlanner).

- **L130**: 🔴 Slot1 ("with balls") is built from the **Slot0** gains; `drivePIDGainsWithBalls` and the other three `...WithBalls` params are never used (and they're all zeros anyway).

- **L201**: 🔴 **Drive and steer configs are swapped.** The record order is `(steerConfig, steerMotor, driveConfig, driveMotor, ...)`; this passes `driveConfig` as steer and `steerConfig` as drive. Steer gets ratio 6.03 + 120 A, drive gets 26.1 + ContinuousWrap. This is COMPBOT, which is the selected chassis. MK4 has it right.

  Also `steerSpeedReduction = 1` makes `getMaxSteerSpeed()` the unreduced motor free speed, so the setpoint generator's steer limit is effectively off.

## `frc/robot/subsystems/drivetrain/module/constants/SwerveModulesMK4.java`

- **L24**: 🟠 `kI = 40` on steer with the iZone/maxAccum silently ignored (`PIDGains` never applies them; Phoenix 6 has no iZone).

- **L27**: 🟡 FL at (−0.29, −0.29) is back-right in WPILib coordinates; all MK4 translations look mirrored. Currently only `getNorm()` uses them (kinematics come from the PP GUI settings), which is two sources of truth.

- **L67**: 🔴 Same as MK5: `0.508` should be `0.0508`.

- **L89**: 🔴 Slot1 is never configured on MK4, so it is all zeros. Whenever `hasBalls()` is true (always when `USE_OBJECT_DETECTION` is false), DEVBOT modules go limp.

## `frc/robot/subsystems/drivetrain/module/SwerveModuleReal.java`

- **L24**: 🔴 **Wrong FusedCANcoder ratios.** With FusedCANcoder the CANcoder is on the mechanism: you need `RotorToSensorRatio = steer gear ratio` and `SensorToMechanismRatio = 1.0`. As written, the steer angle reads as actual/ratio, so the modules will never point where commanded. Sim doesn't use the CANcoder, so it won't catch this.

- **L28**: 🟡 Explicitly set the CANcoder position/velocity update frequencies before `optimizeBusUtilization()`, so the fused sensor signals aren't throttled.

## `frc/robot/subsystems/drivetrain/module/SwerveModuleSim.java`

- **L74**: 🟠 `addRotorPosition`/`setRotorVelocity` expect **rotor** units, but these are mechanism values. Multiply by the gear ratio (same for steer below).

## `frc/lib/math/PIDGains.java`

- **L76**: 🟠 `i_Zone`, `i_maxAccum` and `tolerance` are stored but never applied. The Javadoc claims "if this is zero, I accumulation is disabled", which is false for CTRE. Either drop the fields or make it clear they're unsupported. Nit: `K_D` vs `k_P` casing; errors say "greater than zero" but zero is allowed.

## `frc/lib/simulation/SimulationConfig.java`

- **L63**: 🟠 Dividing kV/kA by 60 assumes per-RPM gains; they aren't. Also note that `DCMotorSim` treats position as radians, while these gains are per rotation.

- **L85**: 🟠 Ignores `momentInertia`, `gearRatio` and `motor` entirely, and `.get()` throws when `motorGains` is empty, which is exactly when this path would be used. Should be `LinearSystemId.createDCMotorSystem(motor, momentInertia, gearRatio)`. The Javadoc also links to a nonexistent `#momentInertiaBased()`.

## `frc/robot/subsystems/drivetrain/constants/SysIDConstants.java`

- **L11**: 🟠 FR and BL are 180° off. Current: FL 135°, FR 225°, BL 45°, BR 315°. For a CCW spin with the MK5 translations you need FL 135°, FR **45°**, BL **225°**, BR 315°. As written, FR and BL drive against FL and BR, so the robot won't spin and the MOI data is garbage. Swap the multipliers: FR `times(3)`, BL `times(1)`. (Defense mode is unaffected; only the axis matters there.)

## `frc/robot/subsystems/drivetrain/module/SwerveModuleCTRE.java`

- **L29**: Nit: override that only calls `super`. Remove it.

- **L36**: 🟡 Signals are refreshed individually and not latency-compensated. Use `BaseStatusSignal.refreshAll(...)` + `getLatencyCompensatedValue` for odometry. A disconnected motor silently returns 0 here (but not in `getDrivePos`); raise an `Alert` instead.

- **L53**: 🟠 **Units mismatch.** In `VelocityTorqueCurrentFOC`/`PositionTorqueCurrentFOC`, kS/kV/kP are in **amps**, but the gains come from voltage SysID (`setVoltage`). The MK5 drive kV ≈ 2.48 also looks like V/(m/s), while TalonFX mechanism units are wheel rps. Either use `VelocityVoltage` (FOC) with the SysID gains, or characterize in torque-current. Also confirm Phoenix Pro licenses (TorqueCurrentFOC + FusedCANcoder).

  🟡 This allocates a new control request for 8 motors every loop (GC pressure). Keep one per motor and use `.withVelocity()`/`.withPosition()`.

- **L88**: 🟡 Applying a full `MotorOutputConfigs` resets `Inverted` (and other fields) to their defaults, and it blocks on 8 config calls inside `disabledExit`/`teleopExit`, which can overrun the loop. Use `setNeutralMode(...)`. No `StatusCode` is checked on any config apply.

## `frc/robot/GeneralRobotState.java`

- **L43**: 🟡 The alliance is cached at construction (robot boot), when the DS/FMS usually isn't connected yet, so it's Blue for the whole match. Query it lazily. The alert is also never cleared.

- **L172**: 🟠 With `USE_OBJECT_DETECTION = false` this is always true, so the drivetrain always uses Slot1 (zeros on MK4, see the comment there).

## `frc/robot/subsystems/drivetrain/chassisSpeedsCalculator/HomeToSupplierChassisSpeedsCalculator.java`

- **L38**: 🟡 Silently dividing max speed by 4 is a hidden behavior; make it an explicit, named parameter.

- **L43**: 🟠 Missing `enableContinuousInput(-Math.PI, Math.PI)`, so it takes the long way around at ±180°. `resetThetaController()` is never called, so the profile starts from state 0.

- **L77**: 🟡 Compares ω in rad/s against the stick deadband (0.1, unitless). It happens to work, but check the raw stick value instead.

- **L95**: 🟠 No `Math.abs`: every negative output becomes 0, so the robot can only turn one direction. This also compares a rad/s output to an angle (1° in rad); the deadband belongs on the error (`setTolerance`/`atGoal`).

## `frc/robot/subsystems/drivetrain/Drivetrain.java`

- **L70**: 🟠 **AdvantageKit replay is broken.** `modulePositions` never go through `processInputs`; they come straight from the IO into the pose estimator. `inputs.moduleStates[i]` also aliases the module's mutable object and is overwritten every loop. And in REPLAY `isReal()` is false, so `SwerveModuleSim` runs physics (line 55). `SwerveModuleIO` should be an IO layer with an `@AutoLog` inputs object.

- **L90**: 🟠 Resetting the gyro without resetting the `SwerveDrivePoseEstimator` makes the estimated heading jump by the reset amount (the estimator tracks gyro deltas). Call `resetRotation`/`resetPose` together with it, or keep a software offset.

- **L121**: Please remove.

## `frc/robot/subsystems/drivetrain/configsStructure/ChassisConstants.java`

- **L73**: 🟡 Pass `e` as the cause. This runs inside the `ChassisType` enum static initializer, so a failure becomes an `ExceptionInInitializerError` with no useful trace. `fromGUISettings()` also means DEVBOT and COMPBOT share mass, MOI, wheel and module locations, and both chassis are always constructed.

## `frc/lib/math/FeedForwardsGains.java`

- **L47**: Nit: the comment says kS can be negative, but the next line throws if it is. The kA error message says "setpointFeedForward".

- **L69**: 🟡 Overloads take parameters in different orders: `(kG,kS,kV,kA)`, `(kA,kV,kS)`, `(kV,kS)`. That's a landmine. Make it an immutable record with named factories/builders.

## `frc/robot/subsystems/drivetrain/gyro/GyroIOPigeon.java`

- **L13**: 🟡 50 Hz yaw is low for odometry; consider 100–250 Hz with synchronized signals.

## `frc/robot/subsystems/drivetrain/chassisSpeedsCalculator/ControllerChassisSpeedsCalculator.java`

- **L17**: Question: exponent 0.5 (sqrt) makes inputs *more* sensitive near zero, the opposite of the usual squared/cubed curves. Intended?

- **L107**: 🟡 A hard cutoff jumps from 0 to 10% output; use `MathUtil.applyDeadband`. The deadband is also per-axis with no radial deadband or vector clamp, so diagonals give √2 × max speed.

- **L115**: 🟡 Reaching into the `RobotContainer` singleton couples the drivetrain to the container (circular). Inject `DoubleSupplier`s for the axes.

## `frc/lib/math/UnitConversions.java`

- **L214**: Nit: copy-pasted Javadoc (documents a `cycleTime` param this method doesn't take). Also casing: `Minuteto` → `MinuteTo`.

## `frc/robot/Constants.java`

- **L16**: Why was `final` removed from `isPit`?
