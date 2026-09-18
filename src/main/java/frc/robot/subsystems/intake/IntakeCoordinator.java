package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

/**This class is used to coordinate the roller and pivot commands */
public class IntakeCoordinator {
    /**The pivot object we will be using */
    private final PivotSubsystem pivot;
    /**The roller object we will be using */
    private final RollerSubsystem roller;

    /**Create a new IntakeCoordinator */
    public IntakeCoordinator(){
        //TODO - we need to initialize the values in robotContainer
        pivot = new PivotSubsystem();
        roller = new RollerSubsystem();
    }

    /**
     * @return A state machine that opens the pivot and then turns the roller on.
     */
    public Command OpenPivotOnRoller() {
        StateMachine activateIntakeStateMachine= new StateMachine("aopenPivotOnRoller_StateMachine");
        Command turnOnRoller = roller.turnOnRoller();
        Command openPivot = pivot.openPivot();

        State turnOnRollerState = activateIntakeStateMachine.addState(turnOnRoller, RollerConstants.turnOnRollerStateName);
        State openPivotState = activateIntakeStateMachine.addState(openPivot, PivotConstants.openPivotStateName);

        activateIntakeStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(turnOnRollerState).when(pivot::isPivotOpen);
        
        return activateIntakeStateMachine;
    }

    /**
     * @return A state machine that opens the pivot and then turns the roller off.
     */
    public Command openPivotOffRoller() {
        StateMachine openPivotOffRollerStateMachine= new StateMachine("openPivotOffRoller_StateMachine");
        Command turnOffRoller = roller.turnOffRoller();
        Command openPivot = pivot.openPivot();

        State turnOffRollerState = openPivotOffRollerStateMachine.addState(turnOffRoller, RollerConstants.turnOffRollerStateName);
        State openPivotState = openPivotOffRollerStateMachine.addState(openPivot, PivotConstants.openPivotStateName);

        openPivotOffRollerStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(turnOffRollerState).when(pivot::isPivotOpen);
        
        return openPivotOffRollerStateMachine;
    }

    /**
     * @return A state machine that closes the pivot and then turns the roller on.
     */
    public Command closePivotOnRoller() {
        StateMachine closePivotOnRollerStateMachine= new StateMachine("closePivotOnRoller_StateMachine");
        Command turnOnRoller = roller.turnOnRoller();
        Command closePivot = pivot.closePivotSlow();

        State turnOnRollerState = closePivotOnRollerStateMachine.addState(turnOnRoller, RollerConstants.turnOnRollerStateName);
        State closePivotState = closePivotOnRollerStateMachine.addState(closePivot, PivotConstants.closePivotStateName);

        closePivotOnRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(turnOnRollerState).when(pivot::isPivotClose);
        
        return closePivotOnRollerStateMachine;
    }

    /**
     * @return A state machine that closes the pivot and then turns the roller off.
     */
    public Command closePivotOffRoller() {
        StateMachine closePivotOffRollerStateMachine= new StateMachine("closePivotOffRoller_StateMachine");
        Command turnOffRoller = roller.turnOffRoller();
        Command closePivot = pivot.closePivotSlow();

        State turnOffRollerState = closePivotOffRollerStateMachine.addState(turnOffRoller, RollerConstants.turnOffRollerStateName);
        State closePivotState = closePivotOffRollerStateMachine.addState(closePivot, PivotConstants.closePivotStateName);

        closePivotOffRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(turnOffRollerState).when(pivot::isPivotClose);
        
        return closePivotOffRollerStateMachine;
    }

     /**
     * @return A state machine that opens the pivot and then reverses the roller.
     */
    public Command openPivotReverseRoller() {
        StateMachine openPivotReverseRollerStateMachine = new StateMachine("openPivotReverseRoller_StateMachine");
        Command reverseRoller = roller.reverseRoller();
        Command openPivot = pivot.openPivot();

        State reverseRollerState = openPivotReverseRollerStateMachine.addState(reverseRoller, RollerConstants.ReverseRollerStateName);
        State openPivotState = openPivotReverseRollerStateMachine.addState(openPivot, PivotConstants.openPivotStateName);

        openPivotReverseRollerStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(reverseRollerState).when(pivot::isPivotOpen);
        
        return openPivotReverseRollerStateMachine;
    }

     /**
     * @return A state machine that opens the pivot and then reverses the roller.
     */
    public Command closePivotReverseRoller() {
        StateMachine closePivotReverseRollerStateMachine = new StateMachine("closePivotReverseRoller_StateMachine");
        Command reverseRoller = roller.reverseRoller();
        Command closePivot = pivot.closePivotSlow();

        State reverseRollerState = closePivotReverseRollerStateMachine.addState(reverseRoller, RollerConstants.ReverseRollerStateName);
        State closePivotState = closePivotReverseRollerStateMachine.addState(closePivot, PivotConstants.closePivotStateName);

        closePivotReverseRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(reverseRollerState).when(pivot::isPivotClose);
        
        return closePivotReverseRollerStateMachine;
    }
}