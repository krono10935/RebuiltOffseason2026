package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.SuperStructureBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SuperStructure extends SuperStructureBase{

    private final CommandXboxController driverXboxController;
    private final CommandXboxController operatorXboxController;


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

        configureBindings();
    }

    protected void configureBindings(){
        //Drivetrain
        //drivetrain.setDefaultCommand(); TODO add the default Command

        //Shooter Bindings
        bindWhileTrue(registerShootState(),driverXboxController.rightTrigger());
        bindOnFalse(registerDisableShootingState(),driverXboxController.rightTrigger());

        //IntakeBindings
        bindWhileTrue(registerDeployIntakeState(),driverXboxController.leftTrigger());
        bindWhileFalse(registerOpenPivotOffRollerState(),driverXboxController.leftTrigger());
        bindWhileTrue(registerOuttakeState(),driverXboxController.leftBumper());
        bindWhileFalse(registerOpenPivotOffRollerState(),driverXboxController.leftBumper());
        bindOnTrue(registerCloseIntakeState(),driverXboxController.b());

        //extra bindings
        bindOnTrue(registerResetGyroState(),driverXboxController.start());

        //Operator Bindings
        //TODO add operator bindings
    }

    protected void configurePitBinding(){

    }

    private StateMachine.State registerDeployIntakeState(){
        return registerState(
                intake.getDeployIntake(false),
                Constants.PIVOT_OPEN_ROLLER_ON_NAME);
    }

    private StateMachine.State registerCloseIntakeState(){
        return registerState(
                intake.getDisableIntake(false),
                Constants.PIVOT_CLOSE_ROLLER_OFF_NAME);
    }

    private StateMachine.State registerOpenPivotOffRollerState(){
        return registerState(
                intake.getOpenPivotOffRoller(false),
                Constants.PIVOT_OPEN_ROLLER_OFF_NAME);
    }

    private StateMachine.State registerShootState(){
        return registerState(
                shooter.shootCommand().
                        alongWith(indexer.turnOnIndexer()).
                            alongWith(intake.getDisableIntake(true)),
                Constants.SHOOT);
    }

    private StateMachine.State registerDisableShootingState(){
        return  registerState(
                shooter.disableShooterCommand().
                        alongWith(indexer.turnOffIndexer()),
                Constants.DISABLE_SHOOTING);
    }

    private StateMachine.State registerOuttakeState(){
        return registerState(
                intake.getDeployIntakeReverse(false).
                        alongWith(indexer.reverseIndexer()),
                Constants.OUTTAKE);
    }

    private StateMachine.State registerResetGyroState(){
        return registerState(
                drivetrain.resetGyro(),
                Constants.RESET_GYRO);
    }
}
