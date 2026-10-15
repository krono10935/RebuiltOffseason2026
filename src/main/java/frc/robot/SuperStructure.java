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

public class SuperStructure extends SuperStructureBase {

    private final CommandXboxController driverXboxController;
    private final CommandXboxController operatorXboxController;

    public final Trigger SHOOTING_TRIGGER;
    public final Trigger INTAKE_TRIGGER;
    public final Trigger OUTTAKE_TRIGGER;
    public final Trigger INDEXER_TRIGGER;
    public final Trigger CLOSE_INTAKE_TRIGGER;
    public final Trigger RESET_GYRO_TRIGGER;

    public final Trigger DISABLE_SHOOTER_TRIGGER;
    public final Trigger DISABLE_INTAKE_TRIGGER;
    public final Trigger DISABLE_INDEXER_TRIGGER;

    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final IntakeCoordinator intake;
    private final DrivetrainReal drivetrain;

    private static SuperStructure instance = null;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }

        return instance;
    }

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

    protected void configureBindings() {
        setInitialState(Constants.SUPERSTRUCTURE_STATE_NAME);

        RESET_GYRO_TRIGGER.onTrue(
                drivetrain.resetGyroCommand()
        );

        DISABLE_SHOOTER_TRIGGER.onTrue(shooter.toggleDisableShooter());
        DISABLE_INDEXER_TRIGGER.onTrue(indexer.toggleDisableIndexer());
        DISABLE_INTAKE_TRIGGER.onTrue(intake.toggleDisableIntake());
    }

    protected void configurePitBindings() {

        operatorXboxController.y().whileTrue(intake.deployIntake(operatorXboxController.povUp().getAsBoolean()));
        operatorXboxController.b().whileTrue(intake.closePivotOnRoller(operatorXboxController.povUp().getAsBoolean()));
        operatorXboxController.a().whileTrue(intake.openPivotOffRoller(operatorXboxController.povUp().getAsBoolean()));
        operatorXboxController.x().whileTrue(intake.deployIntakeReverse(operatorXboxController.povUp().getAsBoolean()));

        operatorXboxController.rightTrigger().whileTrue(shooter.shootCommand());
        operatorXboxController.leftTrigger().whileTrue(shooter.disableShooterCommand());

        operatorXboxController.povRight().whileTrue(indexer.turnOnIndexer());
        operatorXboxController.povLeft().whileTrue(indexer.reverseIndexer());
        operatorXboxController.povDown().whileTrue(indexer.turnOffIndexer());

    }

    private void registerSuperStructureState() {
        registerState(
                getSuperStructureStateCommand(),
                Constants.SUPERSTRUCTURE_STATE_NAME
        );
    }


    private Command getSuperStructureStateCommand() {

        StateMachine stateMachine =
                new StateMachine("SuperStructureStateMachine");

        StateMachine.State check = stateMachine.addState(
                Commands.runOnce(() -> {}),
                Constants.SUPERSTRUCTURE_CHECK
        );

        StateMachine.State shootingCheck = stateMachine.addState(
                Commands.runOnce(() -> {}),
                Constants.SUPERSTRUCTURE_SHOOTING_CHECK
        );

        StateMachine.State normalClosed = stateMachine.addState(
                Commands.parallel(
                        intake.disableIntake(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_CLOSED
        );

        StateMachine.State normalOpenOff = stateMachine.addState(
                Commands.parallel(
                        intake.openPivotOffRoller(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OPEN_OFF
        );

        StateMachine.State normalOpenOn = stateMachine.addState(
                Commands.parallel(
                        intake.deployIntake(false),
                        indexer.turnOffIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OPEN_ON
        );

        StateMachine.State normalOuttake = stateMachine.addState(
                Commands.parallel(
                        intake.deployIntakeReverse(false),
                        indexer.reverseIndexer(),
                        shooter.disableShooterCommand(),
                        drivetrain.driveCommand()
                ),
                Constants.NORMAL_OUTTAKE
        );

        StateMachine.State shootingClosed = stateMachine.addState(
                Commands.parallel(
                        shooter.shootCommand(),
                        intake.closePivotOnRoller(true),
                        indexer.turnOnIndexer().onlyWhile(INDEXER_TRIGGER),
                        drivetrain.defenseModeCommand()
                ),
                Constants.SHOOTING_CLOSED
        );

        StateMachine.State shootingIntake = stateMachine.addState(
                Commands.parallel(
                        shooter.shootCommand(),
                        intake.deployIntake(false),
                        indexer.turnOnIndexer().onlyWhile(INDEXER_TRIGGER),
                        drivetrain.defenseModeCommand()
                ),
                Constants.SHOOTING_INTAKE
        );

        // Start inner state machine closed.
        stateMachine.setInitialState(normalClosed);

        // ============================================================
        // NORMAL MODE
        // ============================================================

        // LT pressed -> intake open + roller on
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOuttake
                ).to(normalOpenOn)
                .when(INTAKE_TRIGGER);

        // LT released -> intake stays open + roller off
        normalOpenOn.switchTo(normalOpenOff)
                .when(INTAKE_TRIGGER.negate());

        // LB pressed -> intake open + roller reverse + indexer reverse
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn
                ).to(normalOuttake)
                .when(OUTTAKE_TRIGGER);

        // LB released while LT is not held
        // -> intake stays open + everything off
        normalOuttake.switchTo(normalOpenOff)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER.negate())
                );

        // LB released while LT is held
        // -> intake open + roller on
        normalOuttake.switchTo(normalOpenOn)
                .when(
                        OUTTAKE_TRIGGER.negate()
                                .and(INTAKE_TRIGGER)
                );

        // B -> close intake
        stateMachine.switchFromAny(
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                ).to(normalClosed)
                .when(CLOSE_INTAKE_TRIGGER);

        // ============================================================
        // SHOOTING
        // ============================================================

        // RT pressed from any normal state -> determine shooting state
        stateMachine.switchFromAny(
                        normalClosed,
                        normalOpenOff,
                        normalOpenOn,
                        normalOuttake
                ).to(shootingCheck)
                .when(SHOOTING_TRIGGER);

        // RT released while shooting -> return to check
        stateMachine.switchFromAny(
                        shootingClosed,
                        shootingIntake
                ).to(check)
                .when(SHOOTING_TRIGGER.negate());

        // LT while shooting
        shootingClosed.switchTo(shootingIntake)
                .when(INTAKE_TRIGGER);

        shootingIntake.switchTo(shootingClosed)
                .when(INTAKE_TRIGGER.negate());

        // ============================================================
        // CHECK STATES
        // ============================================================

        // Decide which shooting state to enter
        shootingCheck.switchTo(shootingIntake)
                .whenCompleteAnd(INTAKE_TRIGGER);

        shootingCheck.switchTo(shootingClosed)
                .whenCompleteAnd(INTAKE_TRIGGER.negate());

        // After shooting ends, restore state based on held buttons
        check.switchTo(normalOpenOn)
                .whenCompleteAnd(INTAKE_TRIGGER);

        check.switchTo(normalOuttake)
                .whenCompleteAnd(OUTTAKE_TRIGGER);

        check.switchTo(normalOpenOff)
                .whenCompleteAnd(
                        INTAKE_TRIGGER.negate()
                                .and(OUTTAKE_TRIGGER.negate())
                );

        return stateMachine;
    }
}