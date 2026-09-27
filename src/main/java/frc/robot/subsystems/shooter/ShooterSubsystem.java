package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;

public class ShooterSubsystem extends SubsystemBase {
    /** flywheel controller */
    private final FlywheelSubsystem flywheel;
    /** hood controller */
    private final HoodSubsystem hood;
    /** one instance for all shooting parameters, gets updated in periodic() */
    private ShootingParameters params;

    public ShooterSubsystem(FlywheelSubsystem flywheel, HoodSubsystem hood) {
        this.flywheel = flywheel;
        this.hood = hood;
    }

    /**
     * Stops both the flywheel and the hood.
     * @return the command to stop the flywheel and hood (sets the hood angle to zero)
     */
    public Command disableShooterCommand() {
        return Commands.parallel(
            flywheel.stopCommand(),
            hood.disableHoodCommand()
        ).withName("DisableHoodCommand");
    }

    /**
     * @return the command to shoot
     */
    public Command shootCommand(){

        // TODO: integrate other parts of the robot like the kicker, indexer, and intake.

        // State Machine for setting the hood angle

        StateMachine setHoodAngleStateMachine = new StateMachine("SetHoodAngle_StateMachine");

        // create the custom commands for the custom parameters
        Command setAngleCommand =  hood.setAngleCommand (() -> params.hoodAngle());
        Command holdAngleCommand = hood.holdAngleCommand(() -> params.hoodAngle());

        // initiate set and hold angle states
        State setAngleState =  setHoodAngleStateMachine.addState(setAngleCommand,  ShooterConstants.SET_ANGLE_STATE_NAME);
        State holdAngleState = setHoodAngleStateMachine.addState(holdAngleCommand, ShooterConstants.HOLD_ANGLE_STATE_NAME);

        // start off by setting the angle quickly
        setHoodAngleStateMachine.setInitialState(setAngleState);

        // set the state to the hold angle state (slower, more accurate PID) when the hood is close enough to the goal
        setAngleState.switchTo(holdAngleState).when(hood::isAtGoal); 


        // State Machine for setting the flywheel speed

        StateMachine setFlywheelSpeedStateMachine = new StateMachine("SetFlywheelSpeed_StateMachine");
        
        // create the custom commands for the custom parameters
        Command setSpeedCommand =  flywheel.spinUpCommand   (() -> params.flywheelSpeed());
        Command holdSpeedCommand = flywheel.holdSpeedCommand(() -> params.flywheelSpeed());
        
        // initiate set and hold speed states
        State setSpeedState =  setFlywheelSpeedStateMachine.addState(setSpeedCommand,  ShooterConstants.SET_SPEED_STATE_NAME);
        State holdSpeedState = setFlywheelSpeedStateMachine.addState(holdSpeedCommand, ShooterConstants.HOLD_SPEED_STATE_NAME);

        // start off by setting the speed quickly
        setFlywheelSpeedStateMachine.setInitialState(setSpeedState);

        // set the state to the hold speed state (slower, more accurate PID) when the flywheel is close enough to the goal
        setSpeedState.switchTo(holdSpeedState).when(flywheel::isAtGoal); 

        // create a custom command that runs both the flywheel speed's command and the hood angle's command
        Command setHoodAndFlywheelCommand = Commands.parallel(setHoodAngleStateMachine, setFlywheelSpeedStateMachine);

        return setHoodAndFlywheelCommand;
    }

    @Override
    public void periodic() {
        params = ShotCalculator.getInstance().getParameters(null, null); // TODO: use RobotState class to get these params after it is implemented.
    }
}


// skebob