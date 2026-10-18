package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.SuperStructureBase;

import frc.robot.subsystems.drivetrain.DrivetrainReal;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Coordinates the robot's major subsystems and manages high-level robot
 * behavior during driver operation.
 *
 * <p>The SuperStructure combines the intake, indexer, shooter, and drivetrain
 * into a single state machine. This allows one driver input to control
 * multiple subsystems together.
 */
public class SuperStructure extends SuperStructureBase {

    /** Controller used for normal robot operation. */
    private final CommandXboxController driverXboxController;

    /** Controller used for subsystem controls and pit testing. */
    private final CommandXboxController operatorXboxController;

    /** Driver trigger used to enter shooting mode. */
    public final Trigger SHOOTING_TRIGGER;

    /** Driver trigger used to run the intake forward. */
    public final Trigger INTAKE_TRIGGER;

    /** Driver trigger used to run the intake and indexer in reverse. */
    public final Trigger OUTTAKE_TRIGGER;

    /** Driver button used to close the intake. */
    public final Trigger CLOSE_INTAKE_TRIGGER;

    /** Driver button used to reset the drivetrain gyro. */
    public final Trigger RESET_GYRO_TRIGGER;

    /** Operator button used to disable or enable the shooter. */
    public final Trigger DISABLE_SHOOTER_TRIGGER;

    /** Operator button used to disable or enable the intake. */
    public final Trigger DISABLE_INTAKE_TRIGGER;

    /** Operator button used to disable or enable the indexer. */
    public final Trigger DISABLE_INDEXER_TRIGGER;

    /** Shooter subsystem controlled by the SuperStructure. */
    private final ShooterSubsystem shooter;

    /** Indexer subsystem controlled by the SuperStructure. */
    private final IndexerSubsystem indexer;

    /** Intake subsystem controlled by the SuperStructure. */
    private final IntakeCoordinator intake;

    /** Drivetrain controlled by the SuperStructure. */
    private final DrivetrainReal drivetrain;

    /** Singleton instance of the SuperStructure. */
    private static SuperStructure instance = null;

    /**
     * Returns the singleton instance of the SuperStructure.
     *
     * @return the SuperStructure instance
     */
    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }

        return instance;
    }

    /**
     * Initializes the controllers, subsystem references, driver triggers,
     * and SuperStructure state machine.
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
        CLOSE_INTAKE_TRIGGER = driverXboxController.b();
        RESET_GYRO_TRIGGER = driverXboxController.start();

        DISABLE_SHOOTER_TRIGGER = operatorXboxController.a();
        DISABLE_INTAKE_TRIGGER = operatorXboxController.b();
        DISABLE_INDEXER_TRIGGER = operatorXboxController.x();

        registerSuperStructureState();
        configureBindings();
    }

    /**
     * Configures bindings that operate outside of the main SuperStructure
     * state machine.
     *
     * <p>These bindings handle gyro reset and operator subsystem disable
     * controls.
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
     * Configures manual subsystem controls used for pit testing.
     *
     * <p>These bindings bypass the normal SuperStructure state machine and
     * allow individual mechanisms to be tested directly.
     */
    protected void configurePitBindings() {
        Trigger intakeSpeed = operatorXboxController.povUp();


        operatorXboxController.y().whileTrue(
                intake.deployIntake(
                        intakeSpeed.getAsBoolean()
                )
        );

        operatorXboxController.b().whileTrue(
                intake.closePivotOnRoller(
                        intakeSpeed.getAsBoolean()
                )
        );

        operatorXboxController.a().whileTrue(
                intake.openPivotOffRoller(
                        intakeSpeed.getAsBoolean()
                )
        );

        operatorXboxController.x().whileTrue(
                intake.deployIntakeReverse(
                        intakeSpeed.getAsBoolean()
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
     * Registers the main SuperStructure state machine with the outer
     * SuperStructure state machine.
     */
    private void registerSuperStructureState() {
        registerState(
                getSuperStructureStateCommand(),
                Constants.SUPERSTRUCTURE_STATE_NAME
        );
    }

    /**
     * Creates the command used for normal driving states.
     *
     * <p>During normal operation the shooter is disabled and the drivetrain
     * uses its normal driving command. The intake and indexer commands are
     * provided by the individual state.
     *
     * @param intakeCommand command defining the intake behavior
     * @param indexerCommand command defining the indexer behavior
     * @return command containing the complete normal driving behavior
     */
    private Command normalState(
            Command intakeCommand,
            Command indexerCommand) {

        return Commands.parallel(
                intakeCommand,
                indexerCommand,
                shooter.disableShooterCommand(),
                drivetrain.driveCommand()
        );
    }

    /**
     * Creates the command used for shooting states.
     *
     * <p>During shooting the shooter and indexer are active and the drivetrain
     * uses defense mode. The intake behavior is provided by the individual
     * shooting state.
     *
     * @param intakeCommand command defining the intake behavior
     * @return command containing the complete shooting behavior
     */
    private Command shootingState(Command intakeCommand) {
        return Commands.parallel(
                shooter.shootCommand(),
                intakeCommand,
                indexer.turnOnIndexer().onlyWhile(SHOOTING_TRIGGER),
                drivetrain.defenseModeCommand()
        );
    }

    /**
     * Creates the state machine that controls the robot during normal
     * driver operation.
     *
     * <p>The state machine is divided into normal states and shooting states.
     * Check states are used when entering or leaving shooting mode so that
     * the robot can choose the correct state based on the controls currently
     * being held.
     *
     * @return command containing the complete SuperStructure state machine
     */
    private Command getSuperStructureStateCommand() {
        StateMachine stateMachine =
                new StateMachine("SuperStructureStateMachine");

        /*
         * Check states don't control any subsystem.
         * They only decide which real state should come next.
         */
        State check = stateMachine.addState(
                Commands.none(),
                Constants.SUPERSTRUCTURE_CHECK
        );

        State shootingCheck = stateMachine.addState(
                Commands.none(),
                Constants.SUPERSTRUCTURE_SHOOTING_CHECK
        );

        // ---------------- NORMAL STATES ----------------

        State normalClosed = stateMachine.addState(
                normalState(
                        intake.disableIntake(false),
                        indexer.turnOffIndexer()
                ),
                Constants.NORMAL_CLOSED
        );

        State normalOpenOff = stateMachine.addState(
                normalState(
                        intake.openPivotOffRoller(false),
                        indexer.turnOffIndexer()
                ),
                Constants.NORMAL_OPEN_OFF
        );

        State normalOpenOn = stateMachine.addState(
                normalState(
                        intake.deployIntake(false),
                        indexer.turnOffIndexer()
                ),
                Constants.NORMAL_OPEN_ON
        );

        State normalOuttake = stateMachine.addState(
                normalState(
                        intake.deployIntakeReverse(false),
                        indexer.reverseIndexer()
                ),
                Constants.NORMAL_OUTTAKE
        );

        // ---------------- SHOOTING STATES ----------------

        State shootingClosed = stateMachine.addState(
                shootingState(
                        intake.closePivotOnRoller(true)
                ),
                Constants.SHOOTING_CLOSED
        );

        State shootingIntake = stateMachine.addState(
                shootingState(
                        intake.deployIntake(false)
                ),
                Constants.SHOOTING_INTAKE
        );

        stateMachine.setInitialState(normalClosed);

        // ============================================================
        // NORMAL MODE
        // ============================================================

        // LT pressed -> open intake and run roller.
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOuttake
                )
                .to(normalOpenOn)
                .when(INTAKE_TRIGGER);

        // LT released -> leave intake open but stop roller.
        normalOpenOn.switchTo(normalOpenOff)
                .when(INTAKE_TRIGGER.negate());

        // LB pressed -> reverse intake and indexer.
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn
                )
                .to(normalOuttake)
                .when(OUTTAKE_TRIGGER);

        // LB released without LT -> intake stays open and stops.
        normalOuttake.switchTo(normalOpenOff)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER.negate())
                );

        // LB released while LT is held -> return to intake.
        normalOuttake.switchTo(normalOpenOn)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER)
                );

        // B pressed -> close intake.
        stateMachine.switchFromAny(
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                )
                .to(normalClosed)
                .when(CLOSE_INTAKE_TRIGGER);

        // ============================================================
        // SHOOTING MODE
        // ============================================================

        /*
         * RT pressed -> use shootingCheck to decide whether shooting
         * starts with the intake open or closed.
         */
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                )
                .to(shootingCheck)
                .when(SHOOTING_TRIGGER);

        /*
         * RT released -> use check to restore the correct normal state
         * based on the controls that are still held.
         */
        stateMachine.switchFromAny(
                        shootingClosed,
                        shootingIntake
                )
                .to(check)
                .when(SHOOTING_TRIGGER.negate());

        // LT pressed while shooting -> run intake.
        shootingClosed.switchTo(shootingIntake)
                .when(INTAKE_TRIGGER);

        // LT released while shooting -> close intake.
        shootingIntake.switchTo(shootingClosed)
                .when(INTAKE_TRIGGER.negate());

        // ============================================================
        // SHOOTING CHECK
        // ============================================================

        // RT + LT -> shoot with intake running.
        shootingCheck.switchTo(shootingIntake)
                .whenCompleteAnd(INTAKE_TRIGGER);

        // RT without LT -> shoot with intake closed.
        shootingCheck.switchTo(shootingClosed)
                .whenCompleteAnd(INTAKE_TRIGGER.negate());

        // ============================================================
        // EXIT SHOOTING CHECK
        // ============================================================

        // RT released while LT is held -> return to intake.
        check.switchTo(normalOpenOn)
                .whenCompleteAnd(INTAKE_TRIGGER);

        // RT released while LB is held -> return to outtake.
        check.switchTo(normalOuttake)
                .whenCompleteAnd(OUTTAKE_TRIGGER);

        // Nothing held -> intake stays open with roller off.
        check.switchTo(normalOpenOff)
                .whenCompleteAnd(
                        INTAKE_TRIGGER.negate()
                                .and(OUTTAKE_TRIGGER.negate())
                );

        return stateMachine;
    }
}