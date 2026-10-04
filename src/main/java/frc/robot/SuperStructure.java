package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.SuperStructureBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SuperStructure extends SuperStructureBase{

    private final CommandXboxController driverXboxController ;
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

    public static SuperStructure getInstance(){
        if (instance == null){
            instance = new SuperStructure();
        }

        return instance;
    }

    private SuperStructure(){
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

    protected void configureBindings(){
//        drivetrain.setDefaultCommand(new );
        bindWhileTrue(getRegisterdState(Constants.SHOOTING_STATE_NAME), SHOOTING_TRIGGER);
        bindWhileFalse(getRegisterdState(Constants.INTAKING_STATE_NAME), SHOOTING_TRIGGER);
        bindOnTrue(getRegisterdState(Constants.RESET_GYRO_NAME), RESET_GYRO_TRIGGER);
        driverXboxController.a().onTrue(drivetrain.driveToPose(new Pose2d(14,4,new Rotation2d(180))));
    }

    protected void configurePitBinding(){

    }

    private void registerShootingState(){
        registerState(
                getShootingStateCommand(),
                Constants.SHOOTING_STATE_NAME);
    }

    private void registerIntakingState(){
        registerState(
                getIntakeStateCommand(),
                Constants.INTAKING_STATE_NAME);
    }

    private void overrideIdleState(){
        registerState(shooter.disableShooterCommand().
                alongWith(intake.getDisableIntake(false)).
                        alongWith(indexer.turnOffIndexer()),Constants.IDLE_STATE_NAME);
    }

    private void registerResetGyroState(){
        registerState(
                drivetrain.resetGyro(),
                Constants.RESET_GYRO_NAME);
    }

    private Command getShootingStateCommand(){
        StateMachine shootingStateMachine = new StateMachine("shootingStateMachine");
        StateMachine.State activateShooting = shootingStateMachine.addState(shooter.shootCommand(), Constants.ACTIVATE_SHOOTING_NAME);
        StateMachine.State disableShooting = shootingStateMachine.addState(shooter.disableShooterCommand(), Constants.DISABLE_SHOOTING_NAME);
        shootingStateMachine.setInitialState(disableShooting);
        disableShooting.switchTo(activateShooting).when(SHOOTING_TRIGGER);
        activateShooting.switchTo(disableShooting).when(SHOOTING_TRIGGER.negate());

        StateMachine intakeShootingStateMachine = new StateMachine("intakeShooterStateMachine");
        StateMachine.State pivotOpenRollerOn = intakeShootingStateMachine.addState(intake.getDeployIntake(false), Constants.PIVOT_OPEN_ROLLER_ON_NAME);
        StateMachine.State pivotCloseRollerOff = intakeShootingStateMachine.addState(intake.getDisableIntake(true), Constants.PIVOT_CLOSE_ROLLER_OFF_NAME);
        intakeShootingStateMachine.setInitialState(pivotOpenRollerOn);
        pivotOpenRollerOn.switchTo(pivotCloseRollerOff).when(INTAKE_TRIGGER.negate().and(shooter.isReadyToShoot()));
        pivotCloseRollerOff.switchTo(pivotOpenRollerOn).when(INTAKE_TRIGGER);

        StateMachine indexerStateMachine = new StateMachine("indexerStateMachine");
        StateMachine.State indexerOn = indexerStateMachine.addState(indexer.turnOnIndexer(), Constants.INDEXER_ON_NAME);
        StateMachine.State indexerOff = indexerStateMachine.addState(indexer.turnOffIndexer(), Constants.INDEXER_OFF_NAME);
        indexerStateMachine.setInitialState(indexerOff);
        indexerOff.switchTo(indexerOn).when(INDEXER_TRIGGER);
        indexerOn.switchTo(indexerOff).when(INDEXER_TRIGGER.negate());

        return shootingStateMachine.alongWith(intakeShootingStateMachine.alongWith(indexerStateMachine)).withName("SHOOTING MODE");
    }

    private Command getIntakeStateCommand(){

        StateMachine intakeStateMachine = new StateMachine("intakeStateMachine");
        StateMachine.State pivotOpenRollerOn = intakeStateMachine.addState(intake.getDeployIntake(false), Constants.PIVOT_OPEN_ROLLER_ON_NAME);
        StateMachine.State pivotOpenRollerOff = intakeStateMachine.addState(intake.getOpenPivotOffRoller(false), Constants.PIVOT_OPEN_ROLLER_OFF_NAME);
        StateMachine.State pivotCloseRollerOff = intakeStateMachine.addState(intake.getDisableIntake(false), Constants.PIVOT_CLOSE_ROLLER_OFF_NAME);
        StateMachine.State pivotOpenRollerReverse = intakeStateMachine.addState(intake.getDeployIntakeReverse(false),Constants.OUTTAKE_NAME);
        intakeStateMachine.setInitialState(pivotCloseRollerOff);
        intakeStateMachine.switchFromAny(pivotOpenRollerOff,pivotCloseRollerOff,pivotOpenRollerReverse).to(pivotOpenRollerOn).when(INTAKE_TRIGGER);
        intakeStateMachine.switchFromAny(pivotOpenRollerOff,pivotCloseRollerOff,pivotOpenRollerOn).to(pivotOpenRollerReverse).when(OUTTAKE_TRIGGER);
        intakeStateMachine.switchFromAny(pivotOpenRollerOff,pivotOpenRollerReverse,pivotOpenRollerOn).to(pivotCloseRollerOff).when(CLOSE_INTAKE_TRIGGER);
        intakeStateMachine.switchFromAny(pivotOpenRollerReverse,pivotOpenRollerOn).to(pivotOpenRollerOff).when(INTAKE_TRIGGER.negate().and(OUTTAKE_TRIGGER.negate()));

        return intakeStateMachine;
    }
}
