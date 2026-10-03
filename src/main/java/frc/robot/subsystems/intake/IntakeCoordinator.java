package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.pivot.PivotSubsystem.PivotState;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem.RollerState;

/**This class is used to coordinate the roller and pivot commands */
public class IntakeCoordinator {
    /**The pivot object we will be using */
    private final PivotSubsystem pivot;
    /**The roller object we will be using */
    private final RollerSubsystem roller;

    /**Create a new IntakeCoordinator */
    public IntakeCoordinator(PivotSubsystem pivot, RollerSubsystem roller){
        this.pivot = pivot;
        this.roller = roller;
    }

    /**
     * Create a stateMachine that goes the the wanted state.
     * @param rollerTargetState An enum of the wanted roller state: off, on, reverse
     * @param pivotTargetState An enum of the wanted pivot state: open, close
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that goes the the wanted state, according the the parameters. 
     */
    private Command intakeStateMachineFactory(RollerState rollerTargetState, PivotState pivotTargetState, boolean changePivotStateSlow){
        String closeName = changePivotStateSlow ? "Slow" : "";

        String stateMachineName = pivotTargetState.getName() + rollerTargetState.getName() + closeName + "StateMachine";

        StateMachine intakeStateMachine = new StateMachine(stateMachineName);

        Command rollerCommand = switch (rollerTargetState) {
            case ON: yield roller.onRoller();
            case OFF: yield roller.offRoller();
            case REVERSED: yield roller.reverseRoller();
        };
        rollerCommand = rollerCommand.withName(rollerTargetState.getName());
        
        Command pivotCommand = switch (pivotTargetState) {
            case OPEN: yield changePivotStateSlow ? pivot.openPivotSlow(): pivot.openPivot();
            case CLOSE: yield changePivotStateSlow ? pivot.closePivotSlow(): pivot.closePivot();
        };
        pivotCommand = pivotCommand.withName(pivotCommand.getName() + changePivotStateSlow);

        State rollerState = intakeStateMachine.addState(rollerCommand, rollerTargetState.getStateName());
        State pivotState = intakeStateMachine.addState(pivotCommand, pivotTargetState.getStateName());

        intakeStateMachine.setInitialState(pivotState);
        pivotState.switchTo(rollerState).when(
            switch (pivotTargetState) {
                case OPEN: yield (() -> pivot.isPivotOpen());
                case CLOSE: yield (() -> pivot.isPivotClose());
            }
        );

        return intakeStateMachine;
    }

    /**
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that opens the pivot, then turns the roller on.
     */
    public Command getDeployIntake(boolean changePivotStateSlow){
        return intakeStateMachineFactory(RollerState.ON, PivotState.OPEN, changePivotStateSlow);
    }

    /**
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that closes the pivot, then turns the roller off.
     */
    public Command getDisableIntake(boolean changePivotStateSlow){
        return intakeStateMachineFactory(RollerState.OFF, PivotState.CLOSE, changePivotStateSlow);
    }

    /**
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that opens the pivot, then turns the roller off.
     */
    public Command getOpenPivotOffRoller(boolean changePivotStateSlow){
        return intakeStateMachineFactory(RollerState.OFF, PivotState.OPEN, changePivotStateSlow);
    }

    /**
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that opens the pivot, then reversed the roller.
     */
    public Command getDeployIntakeReverse(boolean changePivotStateSlow){
        return intakeStateMachineFactory(RollerState.REVERSED, PivotState.OPEN, changePivotStateSlow);
    }
}