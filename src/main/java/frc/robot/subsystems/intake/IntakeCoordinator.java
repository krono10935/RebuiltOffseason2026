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
    public IntakeCoordinator(PivotSubsystem pivot, RollerSubsystem roller){
        //TODO - we need to initialize the values in robotContainer
        this.pivot = pivot;
        this.roller = roller;
    }

    /**
     * @return A state machine that opens the pivot and then turns the roller on.
     */
    public Command deployIntake() {
        StateMachine deployIntakeStateMachine= new StateMachine("deployIntake_StateMachine");
        Command onRoller = roller.onRoller();
        Command openPivot = pivot.openPivot();

        State onRollerState = deployIntakeStateMachine.addState(onRoller, RollerConstants.ON_ROLLER_STATE_NAME);
        State openPivotState = deployIntakeStateMachine.addState(openPivot, PivotConstants.OPEN_PIVOT_STATE_NAME);

        deployIntakeStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(onRollerState).when(pivot::isPivotOpen);
        
        return deployIntakeStateMachine;
    }

    /**
     * @return A state machine that opens the pivot and then turns the roller off.
     */
    public Command openPivotOffRoller() {
        StateMachine openPivotOffRollerStateMachine= new StateMachine("openPivotOffRoller_StateMachine");
        Command offRoller = roller.offRoller();
        Command openPivot = pivot.openPivot();

        State offRollerState = openPivotOffRollerStateMachine.addState(offRoller, RollerConstants.OFF_ROLLER_STATE_NAME);
        State openPivotState = openPivotOffRollerStateMachine.addState(openPivot, PivotConstants.OPEN_PIVOT_STATE_NAME);

        openPivotOffRollerStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(offRollerState).when(pivot::isPivotOpen);
        
        return openPivotOffRollerStateMachine;
    }

    /**
     * @return A state machine that closes the pivot and then turns the roller on.
     */
    public Command closePivotOnRoller() {
        StateMachine closePivotOnRollerStateMachine= new StateMachine("closePivotOnRoller_StateMachine");
        Command onRoller = roller.onRoller();
        Command closePivot = pivot.closePivotSlow();

        State onRollerState = closePivotOnRollerStateMachine.addState(onRoller, RollerConstants.ON_ROLLER_STATE_NAME);
        State closePivotState = closePivotOnRollerStateMachine.addState(closePivot, PivotConstants.CLOSE_PIVOT_STATE_NAME);

        closePivotOnRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(onRollerState).when(pivot::isPivotClose);
        
        return closePivotOnRollerStateMachine;
    }

    /**
     * @return A state machine that closes the pivot and then turns the roller off.
     */
    public Command disableIntake() {
        StateMachine disableIntakeStateMachine= new StateMachine("disableIntake_StateMachine");
        Command offRoller = roller.offRoller();
        Command closePivot = pivot.closePivotSlow();

        State offRollerState = disableIntakeStateMachine.addState(offRoller, RollerConstants.OFF_ROLLER_STATE_NAME);
        State closePivotState = disableIntakeStateMachine.addState(closePivot, PivotConstants.CLOSE_PIVOT_STATE_NAME);

        disableIntakeStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(offRollerState).when(pivot::isPivotClose);
        
        return disableIntakeStateMachine;
    }

     /**
     * @return A state machine that opens the pivot and then reverses the roller.
     */
    public Command deployIntakeReverse() {
        StateMachine deployIntakeReverseStateMachine = new StateMachine("deployIntakeReverse_StateMachine");
        Command reverseRoller = roller.reverseRoller();
        Command openPivot = pivot.openPivot();

        State reverseRollerState = deployIntakeReverseStateMachine.addState(reverseRoller, RollerConstants.REVERSE_ROLLER_STATE_NAME);
        State openPivotState = deployIntakeReverseStateMachine.addState(openPivot, PivotConstants.OPEN_PIVOT_STATE_NAME);

        deployIntakeReverseStateMachine.setInitialState(openPivotState);
        openPivotState.switchTo(reverseRollerState).when(pivot::isPivotOpen);
        
        return deployIntakeReverseStateMachine;
    }

     /**
     * @return A state machine that closes the pivot and then reverses the roller.
     */
    public Command closePivotReverseRoller() {
        StateMachine closePivotReverseRollerStateMachine = new StateMachine("closePivotReverseRoller_StateMachine");
        Command reverseRoller = roller.reverseRoller();
        Command closePivot = pivot.closePivotSlow();

        State reverseRollerState = closePivotReverseRollerStateMachine.addState(reverseRoller, RollerConstants.REVERSE_ROLLER_STATE_NAME);
        State closePivotState = closePivotReverseRollerStateMachine.addState(closePivot, PivotConstants.CLOSE_PIVOT_STATE_NAME);

        closePivotReverseRollerStateMachine.setInitialState(closePivotState);
        closePivotState.switchTo(reverseRollerState).when(pivot::isPivotClose);
        
        return closePivotReverseRollerStateMachine;
    }
}