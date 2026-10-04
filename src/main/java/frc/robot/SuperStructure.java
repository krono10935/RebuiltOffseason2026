package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.SuperStructureBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Controls the robot's high-level mechanisms and operating modes.
 *
 * <p>The superstructure is divided into two primary modes:
 * <ul>
 *     <li>Intaking mode - controls the intake and intake-side indexer.</li>
 *     <li>Shooting mode - controls the shooter, intake, and shooting indexer.</li>
 * </ul>
 *
 * <p>The active mode is determined by the driver's right trigger.
 */
public class SuperStructure extends SuperStructureBase {

    private final CommandXboxController driverXboxController;
    private final CommandXboxController operatorXboxController;

    public final Trigger SHOOTING_TRIGGER;
    public final Trigger INTAKE_TRIGGER;
    public final Trigger OUTTAKE_TRIGGER;
    public final Trigger INDEXER_TRIGGER;
    public final Trigger CLOSE_INTAKE_TRIGGER;
    public final Trigger RESET_GYRO_TRIGGER;

    private final Drivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final IntakeCoordinator intake;

    private static SuperStructure instance = null;

    /**
     * Returns the singleton instance of the superstructure.
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
     * Initializes the superstructure, subsystems, controller triggers,
     * state machines, and controller bindings.
     */
    private SuperStructure() {
        super();

        driverXboxController = RobotContainer.getInstance().getDriverXboxController();
        operatorXboxController = RobotContainer.getInstance().getOperatorXboxController();

        drivetrain = RobotContainer.getInstance().getDrivetrain();
        shooter = RobotContainer.getInstance().getShooter();
        indexer = RobotContainer.getInstance().getIndexer();
        intake = RobotContainer.getInstance().getIntake();

        SHOOTING_TRIGGER = driverXboxController.rightTrigger();
        INTAKE_TRIGGER = driverXboxController.leftTrigger();
        OUTTAKE_TRIGGER = driverXboxController.leftBumper();
        INDEXER_TRIGGER = SHOOTING_TRIGGER.and(shooter.isReadyToShoot());
        CLOSE_INTAKE_TRIGGER = driverXboxController.b();
        RESET_GYRO_TRIGGER = driverXboxController.start();

        registerIntakingState();
        registerShootingState();
        overrideIdleState();
        registerResetGyroState();

        configureBindings();
    }

    /**
     * Configures the driver's and operator's controller bindings.
     */
    protected void configureBindings() {
//        drivetrain.setDefaultCommand(new );

        bindWhileTrue(
                getRegisterdState(Constants.SHOOTING_MODE_STATE_NAME),
                SHOOTING_TRIGGER
        );

        bindWhileTrue(
                getRegisterdState(Constants.INTAKING_MODE_STATE_NAME),
                SHOOTING_TRIGGER.negate()
        );

        bindWhileTrue(
                getRegisterdState(Constants.RESET_GYRO_NAME),
                RESET_GYRO_TRIGGER
        );

        driverXboxController.a().onTrue(
                drivetrain.driveToPose(
                        new Pose2d(14, 4, new Rotation2d(180))
                )
        );
    }

    /**
     * Configures bindings used while the robot is in pit mode.
     */
    protected void configurePitBinding() {

    }

    /**
     * Registers the shooting mode state.
     */
    private void registerShootingState() {
        registerState(
                getShootingModeStateCommand(),
                Constants.SHOOTING_MODE_STATE_NAME
        );
    }

    /**
     * Registers the intake mode state.
     */
    private void registerIntakingState() {
        registerState(
                getIntakeModeStateCommand(),
                Constants.INTAKING_MODE_STATE_NAME
        );
    }

    /**
     * Registers the idle state.
     *
     * <p>The idle state disables the shooter, intake, and indexer.
     */
    private void overrideIdleState() {
        registerState(
                shooter.disableShooterCommand()
                        .alongWith(intake.disableIntake(false))
                        .alongWith(indexer.turnOffIndexer()),
                Constants.IDLE_STATE_NAME
        );
    }

    /**
     * Registers the gyro reset state.
     */
    private void registerResetGyroState() {
        registerState(
                drivetrain.resetGyro(),
                Constants.RESET_GYRO_NAME
        );
    }

    /**
     * Creates the shooting mode state machine.
     *
     * <p>Shooting mode controls:
     * <ul>
     *     <li>The shooter.</li>
     *     <li>The intake and roller.</li>
     *     <li>The indexer.</li>
     * </ul>
     *
     * @return the shooting mode command
     */
    private Command getShootingModeStateCommand() {
        StateMachine shootingStateMachine = new StateMachine("shootingStateMachine");

        StateMachine.State activateShooting = shootingStateMachine.addState(
                shooter.shootCommand(),
                Constants.ACTIVATE_SHOOTING_NAME
        );

        StateMachine.State disableShooting = shootingStateMachine.addState(
                shooter.disableShooterCommand(),
                Constants.DISABLE_SHOOTING_NAME
        );

        shootingStateMachine.setInitialState(activateShooting);

        disableShooting.switchTo(activateShooting).when(SHOOTING_TRIGGER);
        activateShooting.switchTo(disableShooting).when(SHOOTING_TRIGGER.negate());

        StateMachine intakeShootingStateMachine =
                new StateMachine("intakeShootingStateMachine");

        StateMachine.State checkInputsIntakeShooter =
                intakeShootingStateMachine.addState(
                        Commands.run(() -> {}),
                        Constants.INTAKING_SHOOTING_CHECK_INPUTS_NAME
                );

        StateMachine.State pivotOpenRollerOn =
                intakeShootingStateMachine.addState(
                        intake.deployIntake(false),
                        Constants.PIVOT_OPEN_ROLLER_ON_NAME
                );

        StateMachine.State pivotCloseRollerOff =
                intakeShootingStateMachine.addState(
                        intake.disableIntake(true),
                        Constants.PIVOT_CLOSE_ROLLER_OFF_NAME
                );

        intakeShootingStateMachine.setInitialState(checkInputsIntakeShooter);

        checkInputsIntakeShooter
                .switchTo(pivotOpenRollerOn)
                .when(INTAKE_TRIGGER);

        checkInputsIntakeShooter
                .switchTo(pivotCloseRollerOff)
                .when(INTAKE_TRIGGER.negate().and(shooter.isReadyToShoot()));

        pivotOpenRollerOn
                .switchTo(pivotCloseRollerOff)
                .when(INTAKE_TRIGGER.negate().and(shooter.isReadyToShoot()));

        pivotCloseRollerOff
                .switchTo(pivotOpenRollerOn)
                .when(INTAKE_TRIGGER);

        StateMachine indexerStateMachine =
                new StateMachine("indexerShootingStateMachine");

        StateMachine.State checkInputsIndexerShooter =
                indexerStateMachine.addState(
                        Commands.run(() -> {}),
                        Constants.SHOOTING_INDEXER_CHECK_INPUTS_NAME
                );

        StateMachine.State indexerOn =
                indexerStateMachine.addState(
                        indexer.turnOnIndexer(),
                        Constants.INDEXER_ON_NAME
                );

        StateMachine.State indexerOff =
                indexerStateMachine.addState(
                        indexer.turnOffIndexer(),
                        Constants.INDEXER_OFF_NAME
                );

        indexerStateMachine.setInitialState(checkInputsIndexerShooter);

        checkInputsIndexerShooter
                .switchTo(indexerOn)
                .when(INDEXER_TRIGGER);

        checkInputsIndexerShooter
                .switchTo(indexerOff)
                .when(INDEXER_TRIGGER.negate());

        indexerOff
                .switchTo(indexerOn)
                .when(INDEXER_TRIGGER);

        indexerOn
                .switchTo(indexerOff)
                .when(INDEXER_TRIGGER.negate());

        return shootingStateMachine
                .alongWith(
                        intakeShootingStateMachine.alongWith(indexerStateMachine)
                )
                .withName("SHOOTING MODE");
    }

    /**
     * Creates the intake mode state machine.
     *
     * <p>Intake mode controls:
     * <ul>
     *     <li>Opening and closing the intake.</li>
     *     <li>Forward and reverse intake operation.</li>
     *     <li>The intake-side indexer.</li>
     * </ul>
     *
     * @return the intake mode command
     */
    private Command getIntakeModeStateCommand() {
        StateMachine intakeStateMachine = new StateMachine("intakeStateMachine");

        StateMachine.State checkInputsIntake =
                intakeStateMachine.addState(
                        Commands.run(() -> {}),
                        Constants.INTAKING_CHECK_INPUTS_NAME
                );

        StateMachine.State pivotOpenRollerOn =
                intakeStateMachine.addState(
                        intake.deployIntake(false),
                        Constants.PIVOT_OPEN_ROLLER_ON_NAME
                );

        StateMachine.State pivotOpenRollerOff =
                intakeStateMachine.addState(
                        intake.openPivotOffRoller(false),
                        Constants.PIVOT_OPEN_ROLLER_OFF_NAME
                );

        StateMachine.State pivotCloseRollerOff =
                intakeStateMachine.addState(
                        intake.disableIntake(false),
                        Constants.PIVOT_CLOSE_ROLLER_OFF_NAME
                );

        StateMachine.State pivotOpenRollerReverse =
                intakeStateMachine.addState(
                        intake.deployIntakeReverse(false),
                        Constants.OUTTAKE_NAME
                );

        intakeStateMachine.setInitialState(checkInputsIntake);

        checkInputsIntake
                .switchTo(pivotOpenRollerOn)
                .when(INTAKE_TRIGGER);

        checkInputsIntake
                .switchTo(pivotOpenRollerReverse)
                .when(OUTTAKE_TRIGGER);

        checkInputsIntake
                .switchTo(pivotCloseRollerOff)
                .when(
                        INTAKE_TRIGGER.negate()
                                .and(OUTTAKE_TRIGGER.negate())
                );

        intakeStateMachine
                .switchFromAny(
                        pivotOpenRollerOff,
                        pivotCloseRollerOff,
                        pivotOpenRollerReverse
                )
                .to(pivotOpenRollerOn)
                .when(INTAKE_TRIGGER);

        intakeStateMachine
                .switchFromAny(
                        pivotOpenRollerOff,
                        pivotCloseRollerOff,
                        pivotOpenRollerOn
                )
                .to(pivotOpenRollerReverse)
                .when(OUTTAKE_TRIGGER);

        intakeStateMachine
                .switchFromAny(
                        pivotOpenRollerOff,
                        pivotOpenRollerReverse,
                        pivotOpenRollerOn
                )
                .to(pivotCloseRollerOff)
                .when(CLOSE_INTAKE_TRIGGER);

        intakeStateMachine
                .switchFromAny(
                        pivotOpenRollerReverse,
                        pivotOpenRollerOn
                )
                .to(pivotOpenRollerOff)
                .when(
                        INTAKE_TRIGGER.negate()
                                .and(OUTTAKE_TRIGGER.negate())
                );

        StateMachine indexerStateMachine =
                new StateMachine("indexerIntakingStateMachine");

        StateMachine.State checkInputsIndexerIntake =
                indexerStateMachine.addState(
                        Commands.run(() -> {}),
                        Constants.INTAKING_INDEXER_CHECK_INPUTS_NAME
                );

        StateMachine.State indexerOff =
                indexerStateMachine.addState(
                        indexer.turnOffIndexer(),
                        Constants.INDEXER_OFF_NAME
                );

        StateMachine.State indexerReverse =
                indexerStateMachine.addState(
                        indexer.reverseIndexer(),
                        Constants.INDEXER_REVERSED_NAME
                );

        indexerStateMachine.setInitialState(checkInputsIndexerIntake);

        checkInputsIndexerIntake
                .switchTo(indexerReverse)
                .when(OUTTAKE_TRIGGER);

        checkInputsIndexerIntake
                .switchTo(indexerOff)
                .when(OUTTAKE_TRIGGER.negate());

        indexerOff
                .switchTo(indexerReverse)
                .when(OUTTAKE_TRIGGER);

        indexerReverse
                .switchTo(indexerOff)
                .when(OUTTAKE_TRIGGER.negate());

        return intakeStateMachine.alongWith(indexerStateMachine);
    }
}