package frc.robot.subsystems.intake;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.StateMachine.StateName;
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
     * Create a stateMachine that goes the wanted state by doing both commands sequentially.
     * @param rollerTargetState An enum of the wanted roller state: off, on, reverse
     * @param pivotTargetState An enum of the wanted pivot state: open, close
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A state machine that goes the the wanted state, according the the parameters. 
     */
    private Command intakeStateSequenceMachineFactory(RollerState rollerTargetState, PivotState pivotTargetState, boolean changePivotStateSlow){
        String closeName = changePivotStateSlow ? "Slow" : "";

        String stateMachineName = pivotTargetState.getName() + rollerTargetState.getName() + closeName + "StateMachine";

        StateMachine intakeStateMachine = new StateMachine(stateMachineName);

        Command rollerCommand = switch (rollerTargetState) {
            case ON: yield roller.turnOnRoller();
            case OFF: yield roller.turnOffRoller();
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
     * Create a parallel command that goes the wanted state.
     * @param rollerTargetState An enum of the wanted roller state: off, on, reverse
     * @param pivotTargetState An enum of the wanted pivot state: open, close
     * @param changePivotStateSlow Weather we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A parallel command that goes the the wanted state, according the the parameters. 
     */
    private Command intakeCommandFactory(RollerState rollerTargetState, PivotState pivotTargetState, boolean changePivotStateSlow){
        //Choose the roller command
        Command rollerCommand = switch (rollerTargetState) {
            case ON: yield roller.turnOnRoller();
            case OFF: yield roller.turnOffRoller();
            case REVERSED: yield roller.reverseRoller();
        };
        rollerCommand = rollerCommand.withName(rollerTargetState.getName());
        
        //Choose the pivot command
        Command pivotCommand = switch (pivotTargetState) {
            case OPEN: yield changePivotStateSlow ? pivot.openPivotSlow(): pivot.openPivot();
            case CLOSE: yield changePivotStateSlow ? pivot.closePivotSlow(): pivot.closePivot();
        };
        pivotCommand = pivotCommand.withName(pivotCommand.getName() + (changePivotStateSlow ? "Slow" : ""));

        //Create the intakeCommand using the pivotCommand and rollerCommand
        Command intakeCommand = Commands.parallel(rollerCommand, pivotCommand);

        return intakeCommand;

    }
    
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller on.(at the same time).
     */
    public Command deployIntake(Boolean changePivotStateSlow){
        return intakeCommandFactory(RollerState.ON, PivotState.OPEN, changePivotStateSlow);
    }
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller on.(at the same time).
     */
    public Command deployIntake(BooleanSupplier changePivotStateSlow){
        return intakeCommandFactory(RollerState.ON, PivotState.OPEN, changePivotStateSlow.getAsBoolean());
    }

    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that closes the pivot and turns the roller off(at the same time).
     */
    public Command disableIntake(Boolean changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.CLOSE, changePivotStateSlow);
    }
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that closes the pivot and turns the roller off(at the same time).
     */
    public Command disableIntake(BooleanSupplier changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.CLOSE, changePivotStateSlow.getAsBoolean());
    }

    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller off(at the same time).
     */
    public Command closePivotOnRoller(Boolean changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.CLOSE, changePivotStateSlow);
    }
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller off(at the same time).
     */
    public Command closePivotOnRoller(BooleanSupplier changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.CLOSE, changePivotStateSlow.getAsBoolean());
    }

    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller off(at the same time).
     */
    public Command openPivotOffRoller(Boolean changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.OPEN, changePivotStateSlow);
    }
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and turns the roller off(at the same time).
     */
    public Command openPivotOffRoller(BooleanSupplier changePivotStateSlow){
        return intakeCommandFactory(RollerState.OFF, PivotState.OPEN, changePivotStateSlow.getAsBoolean());
    }

    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and reverses the roller(at the same time).
     */
    public Command deployIntakeReverse(Boolean changePivotStateSlow){
        return intakeCommandFactory(RollerState.REVERSED, PivotState.OPEN, changePivotStateSlow);
    }
    /**
     * @param changePivotStateSlow Whether we should open/close the pivot using a trapezoid profile. Should be used if the hopper is full with fuel.
     * @return A command that opens the pivot and reverses the roller(at the same time).
     */
    public Command deployIntakeReverse(BooleanSupplier changePivotStateSlow){
        return intakeCommandFactory(RollerState.REVERSED, PivotState.OPEN, changePivotStateSlow.getAsBoolean());
    }
}