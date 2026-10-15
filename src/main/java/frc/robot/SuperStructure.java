package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.SuperStructureBase;

import frc.robot.subsystems.drivetrain.DrivetrainReal;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Coordinates the robot's major subsystems and manages the high-level
 * behavior of the robot during teleoperated operation.
 *
 * <p>The SuperStructure combines the intake, indexer, shooter, and drivetrain
 * into a single state machine. This allows driver inputs to control multiple
 * subsystems together without requiring each subsystem to independently
 * determine what the robot is trying to do.
 *
 * <p>The class also contains normal driver bindings, subsystem disable
 * controls, gyro reset, and manual pit-mode bindings for testing individual
 * mechanisms.
 */
public class SuperStructure extends SuperStructureBase {

    /** Driver controller used for normal robot operation. */
    private final CommandXboxController driverXboxController;

    /** Operator controller used for subsystem controls and pit testing. */
    private final CommandXboxController operatorXboxController;

    /** Trigger used to enter shooting mode. */
    public final Trigger SHOOTING_TRIGGER;

    /** Trigger used to deploy the intake and run the intake roller. */
    public final Trigger INTAKE_TRIGGER;

    /** Trigger used to reverse the intake and indexer. */
    public final Trigger OUTTAKE_TRIGGER;

    /** Trigger used to run the indexer while shooting. */
    public final Trigger INDEXER_TRIGGER;

    /** Trigger used to close the intake. */
    public final Trigger CLOSE_INTAKE_TRIGGER;

    /** Trigger used to reset the drivetrain gyro. */
    public final Trigger RESET_GYRO_TRIGGER;

    /** Operator trigger used to toggle the shooter disabled state. */
    public final Trigger DISABLE_SHOOTER_TRIGGER;

    /** Operator trigger used to toggle the intake disabled state. */
    public final Trigger DISABLE_INTAKE_TRIGGER;

    /** Operator trigger used to toggle the indexer disabled state. */
    public final Trigger DISABLE_INDEXER_TRIGGER;

    /** Shooter subsystem controlled by the SuperStructure. */
    private final ShooterSubsystem shooter;

    /** Indexer subsystem controlled by the SuperStructure. */
    private final IndexerSubsystem indexer;

    /** Intake coordinator controlled by the SuperStructure. */
    private final IntakeCoordinator intake;

    /** Drivetrain controlled by the SuperStructure. */
    private final DrivetrainReal drivetrain;

    /** Singleton instance of the SuperStructure. */
    private static SuperStructure instance = null;

    /**
     * Returns the single SuperStructure instance.
     *
     * <p>The instance is created the first time this method is called.
     *
     * @return the SuperStructure singleton instance
     */
    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }

        return instance;
    }

    /**
     * Creates the SuperStructure.
     *
     * <p>This initializes the controllers, subsystem references, driver
     * triggers, disable triggers, SuperStructure state, and normal bindings.
     */
    private SuperStructure() {
        super();

        driverXboxController =
                RobotContainer.getInstance().getDriverXboxController();

        operatorXboxController =
                RobotContainer.getInstance().getOperatorXboxController();

        drivetrain = RobotContainer.getInstance().getDrivetrain();
        shooter = RobotContainer.getInstance().getShooter();
        indexer = RobotContainer.getInstance().getIndexer();
        intake = RobotContainer.getInstance().getIntake();

        SHOOTING_TRIGGER = driverXboxController.rightTrigger();
        INTAKE_TRIGGER = driverXboxController.leftTrigger();
        OUTTAKE_TRIGGER = driverXboxController.leftBumper();
        INDEXER_TRIGGER = SHOOTING_TRIGGER;
        CLOSE_INTAKE_TRIGGER = driverXboxController.b();
        RESET_GYRO_TRIGGER = driverXboxController.start();

        DISABLE_SHOOTER_TRIGGER = operatorXboxController.a();
        DISABLE_INTAKE_TRIGGER = operatorXboxController.b();
        DISABLE_INDEXER_TRIGGER = operatorXboxController.x();

        registerSuperStructureState();

        configureBindings();
    }

    /**
     * Configures the bindings used during normal robot operation.
     *
     * <p>The outer SuperStructure state machine starts in the main
     * SuperStructure state. Gyro reset is scheduled independently so resetting
     * the gyro does not interrupt or reset the internal SuperStructure state
     * machine.
     *
     * <p>The operator controller can also toggle the disabled state of the
     * shooter, indexer, and intake.
     */
    protected void configureBindings() {
        setInitialState(Constants.SUPERSTRUCTURE_STATE_NAME);

        RESET_GYRO_TRIGGER.onTrue(
                drivetrain.resetGyroCommand()
        );

        DISABLE_SHOOTER_TRIGGER.onTrue(
                shooter.toggleDisableShooter()
        );

        DISABLE_INDEXER_TRIGGER.onTrue(
                indexer.toggleDisableIndexer()
        );

        DISABLE_INTAKE_TRIGGER.onTrue(
                intake.toggleDisableIntake()
        );
    }

    /**
     * Configures manual controls used for testing mechanisms in the pit.
     *
     * <p>Each control directly schedules a subsystem command instead of
     * controlling the subsystem through the normal SuperStructure state
     * machine.
     *
     * <p>Intake controls:
     * <ul>
     *     <li>Y - deploy intake and run the roller forward</li>
     *     <li>B - close the intake pivot</li>
     *     <li>A - open the intake pivot with the roller off</li>
     *     <li>X - deploy the intake and reverse the roller</li>
     * </ul>
     *
     * <p>Shooter controls:
     * <ul>
     *     <li>Right Trigger - run the shooter</li>
     *     <li>Left Trigger - disable the shooter</li>
     * </ul>
     *
     * <p>Indexer controls:
     * <ul>
     *     <li>D-pad Right - run the indexer forward</li>
     *     <li>D-pad Left - run the indexer in reverse</li>
     *     <li>D-pad Down - stop the indexer</li>
     * </ul>
     *
     * <p>D-pad Up is passed into the intake commands as their boolean
     * configuration value.
     */
    protected void configurePitBindings() {

        operatorXboxController.y().whileTrue(
                intake.deployIntake(
                        operatorXboxController.povUp().getAsBoolean()
                )
        );

        operatorXboxController.b().whileTrue(
                intake.closePivotOnRoller(
                        operatorXboxController.povUp().getAsBoolean()
                )
        );

        operatorXboxController.a().whileTrue(
                intake.openPivotOffRoller(
                        operatorXboxController.povUp().getAsBoolean()
                )
        );

        operatorXboxController.x().whileTrue(
                intake.deployIntakeReverse(
                        operatorXboxController.povUp().getAsBoolean()
                )
        );

        operatorXboxController.rightTrigger()
                .whileTrue(shooter.shootCommand());

        operatorXboxController.leftTrigger()
                .whileTrue(shooter.disableShooterCommand());

        operatorXboxController.povRight()
                .whileTrue(indexer.turnOnIndexer());

        operatorXboxController.povLeft()
                .whileTrue(indexer.reverseIndexer());

        operatorXboxController.povDown()
                .whileTrue(indexer.turnOffIndexer());
    }

    /**
     * Registers the main SuperStructure command as a state in the outer
     * SuperStructure state machine.
     *
     * <p>The registered command contains the internal state machine responsible
     * for coordinating the intake, indexer, shooter, and drivetrain.
     */
    private void registerSuperStructureState() {
        registerState(
                getSuperStructureStateCommand(),
                Constants.SUPERSTRUCTURE_STATE_NAME
        );
    }

    /**
     * Creates the internal SuperStructure state machine.
     *
     * <p>The state machine coordinates normal intake operation, outtake
     * operation, shooting, and drivetrain behavior.
     *
     * <p>The normal states determine whether the intake is closed, open with
     * the roller off, open with the roller running, or reversing. The shooting
     * states allow the intake to switch between closed and deployed while the
     * shooting trigger remains held.
     *
     * <p>Check states are used when entering and leaving shooting mode to
     * determine which state should be selected based on the driver's currently
     * held controls.
     *
     * @return the command representing the complete internal SuperStructure
     *         state machine
     */
    private Command getSuperStructureStateCommand() {

        StateMachine stateMachine =
                new StateMachine("SuperStructureStateMachine");

        /*
         * Temporary state used after leaving shooting mode.
         * Determines which normal state should be restored based on
         * the driver's currently held controls.
         */
        StateMachine.State check = stateMachine.addState(
                Commands.runOnce(() -> {}),
                Constants.SUPERSTRUCTURE_CHECK
        );

        /*
         * Temporary state used when entering shooting mode.
         * Determines whether the intake should begin open or closed.
         */
        StateMachine.State shootingCheck = stateMachine.addState(
                Commands.runOnce(() -> {}),
                Constants.SUPERSTRUCTURE_SHOOTING_CHECK
        );

        /*
         * Normal operation with the intake closed.
         * Intake roller, indexer, and shooter are disabled.
         */
        StateMachine.State normalClosed = stateMachine.addState(
                Commands.parallel(
                        intake.disableIntake(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_CLOSED
        );

        /*
         * Normal operation with the intake open but the roller stopped.
         */
        StateMachine.State normalOpenOff = stateMachine.addState(
                Commands.parallel(
                        intake.openPivotOffRoller(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OPEN_OFF
        );

        /*
         * Normal intake operation.
         * Intake is open and the roller is running forward.
         */
        StateMachine.State normalOpenOn = stateMachine.addState(
                Commands.parallel(
                        intake.deployIntake(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OPEN_ON
        );

        /*
         * Outtake operation.
         * Intake roller and indexer run in reverse.
         */
        StateMachine.State normalOuttake = stateMachine.addState(
                Commands.parallel(
                        intake.deployIntakeReverse(false),
                        indexer.reverseIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OUTTAKE
        );

        /*
         * Shooting with the intake closed.
         * Shooter and indexer run while the drivetrain uses defense mode.
         */
        StateMachine.State shootingClosed = stateMachine.addState(
                Commands.parallel(
                        shooter.shootCommand(),
                        intake.closePivotOnRoller(true),
                        indexer.turnOnIndexer().onlyWhile(INDEXER_TRIGGER),
                        drivetrain.defenseModeCommand()
                ),
                Constants.SHOOTING_CLOSED
        );

        /*
         * Shooting while simultaneously running the intake.
         * Shooter and indexer remain active and the drivetrain stays
         * in defense mode.
         */
        StateMachine.State shootingIntake = stateMachine.addState(
                Commands.parallel(
                        shooter.shootCommand(),
                        intake.deployIntake(false),
                        indexer.turnOnIndexer().onlyWhile(INDEXER_TRIGGER),
                        drivetrain.defenseModeCommand()
                ),
                Constants.SHOOTING_INTAKE
        );

        // Start the robot with the intake closed.
        stateMachine.setInitialState(normalClosed);

        // ============================================================
        // NORMAL MODE
        // ============================================================

        // LT pressed -> intake open + roller on.
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOuttake
                ).to(normalOpenOn)
                .when(INTAKE_TRIGGER);

        // LT released -> intake stays open + roller off.
        normalOpenOn.switchTo(normalOpenOff)
                .when(INTAKE_TRIGGER.negate());

        // LB pressed -> intake open + roller reverse + indexer reverse.
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn
                ).to(normalOuttake)
                .when(OUTTAKE_TRIGGER);

        /*
         * LB released while LT is not held.
         * Keep the intake open but stop the roller and indexer.
         */
        normalOuttake.switchTo(normalOpenOff)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER.negate())
                );

        /*
         * LB released while LT is held.
         * Return to normal intake operation.
         */
        normalOuttake.switchTo(normalOpenOn)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER)
                );

        // B pressed -> close the intake.
        stateMachine.switchFromAny(
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                ).to(normalClosed)
                .when(CLOSE_INTAKE_TRIGGER);

        // ============================================================
        // SHOOTING MODE
        // ============================================================

        /*
         * RT pressed from any normal state.
         * Enter the shooting check state to determine whether the intake
         * should be open or closed.
         */
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                ).to(shootingCheck)
                .when(SHOOTING_TRIGGER);

        /*
         * RT released while shooting.
         * Enter the check state to determine which normal state should
         * be restored.
         */
        stateMachine.switchFromAny(
                        shootingClosed,
                        shootingIntake
                ).to(check)
                .when(SHOOTING_TRIGGER.negate());

        // LT pressed while shooting -> deploy intake and run roller.
        shootingClosed.switchTo(shootingIntake)
                .when(INTAKE_TRIGGER);

        // LT released while shooting -> close intake and stop roller.
        shootingIntake.switchTo(shootingClosed)
                .when(INTAKE_TRIGGER.negate());

        // ============================================================
        // CHECK STATES
        // ============================================================

        /*
         * When shooting begins with LT held, enter shooting mode with
         * the intake deployed.
         */
        shootingCheck.switchTo(shootingIntake)
                .whenCompleteAnd(INTAKE_TRIGGER);

        /*
         * When shooting begins without LT held, enter shooting mode
         * with the intake closed.
         */
        shootingCheck.switchTo(shootingClosed)
                .whenCompleteAnd(INTAKE_TRIGGER.negate());

        /*
         * Shooting ended while LT is held.
         * Return to normal intake operation.
         */
        check.switchTo(normalOpenOn)
                .whenCompleteAnd(INTAKE_TRIGGER);

        /*
         * Shooting ended while LB is held.
         * Enter normal outtake operation.
         */
        check.switchTo(normalOuttake)
                .whenCompleteAnd(OUTTAKE_TRIGGER);

        /*
         * Shooting ended without LT or LB held.
         * Leave the intake open with the roller stopped.
         */
        check.switchTo(normalOpenOff)
                .whenCompleteAnd(
                        INTAKE_TRIGGER.negate()
                                .and(OUTTAKE_TRIGGER.negate())
                );

        return stateMachine;
    }
}