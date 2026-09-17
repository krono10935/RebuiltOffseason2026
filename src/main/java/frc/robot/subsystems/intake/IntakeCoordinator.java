package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.StateMachine.StateName;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
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

    public Command activateIntake() {
        StateMachine activateIntakeStateMachine= new StateMachine("activateIntake_StateMachine");
        Command turnOnRoller = roller.turnOnRoller();
        Command openPivot = pivot.openPivot();


        StateName turnOnRollerStateName = new StateName("turnOnRollerState");
        StateName openPivotStateName = new StateName("openPivotState");

        State turnOnRollerState = activateIntakeStateMachine.addState(turnOnRoller, turnOnRollerStateName);
        State openPivotState = activateIntakeStateMachine.addState(openPivot, openPivotStateName);

        activateIntakeStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(turnOnRollerState).when(pivot::isPivotOpen);
        
        return activateIntakeStateMachine;
    }
    
public Command openPivotOffRoller() {
        StateMachine openPivotOffRollerStateMachine= new StateMachine("openPivotOffRoller_StateMachine");
        Command turnOffRoller = roller.turnOffRoller();
        Command openPivot = pivot.openPivot();


        StateName turnOffRollerStateName = new StateName("turnOffRollerState");
        StateName openPivotStateName = new StateName("openPivotState");

        State turnOffRollerState = openPivotOffRollerStateMachine.addState(turnOffRoller, turnOffRollerStateName);
        State openPivotState = openPivotOffRollerStateMachine.addState(openPivot, openPivotStateName);

        openPivotOffRollerStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(turnOffRollerState).when(pivot::isPivotOpen);
        
        return openPivotOffRollerStateMachine;
}

public Command closePivotOnRoller() {
        StateMachine closePivotOnRollerStateMachine= new StateMachine("closePivotOnRoller_StateMachine");
        Command turnOnRoller = roller.turnOnRoller();
        Command closePivot = pivot.closePivot();


        StateName turnOffRollerStateName = new StateName("turnOffRollerState");
        StateName openPivotStateName = new StateName("openPivotState");

        State turnOffRollerState = closePivotOnRollerStateMachine.addState(turnOnRoller, turnOffRollerStateName);
        State closePivotState = closePivotOnRollerStateMachine.addState(closePivot, openPivotStateName);

        closePivotOnRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(turnOffRollerState).when(pivot::isPivotClose);
        
        return closePivotOnRollerStateMachine;
}

public Command closePivotOffRoller() {
        StateMachine closePivotOffRollerStateMachine= new StateMachine("closePivotOffRoller_StateMachine");
        Command turnOffRoller = roller.turnOffRoller();
        Command closePivot = pivot.closePivot();


        StateName turnOffRollerStateName = new StateName("turnOffRollerState");
        StateName closePivotStateName = new StateName("closePivotState");

        State turnOffRollerState = closePivotOffRollerStateMachine.addState(turnOffRoller, turnOffRollerStateName);
        State closePivotState = closePivotOffRollerStateMachine.addState(closePivot, closePivotStateName);

        closePivotOffRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(turnOffRollerState).when(pivot::isPivotClose);
        
        return closePivotOffRollerStateMachine;
}

}
