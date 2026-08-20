// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.statemachine;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.StateMachine.StateName;
import frc.robot.Constants;

public abstract class SuperStructureBase extends SubsystemBase {
    
    // All registered states of the SuperStructure
    private final List<State> registeredStates;

    // The actual statemachine command
    private final StateMachine statemachine;

    protected SuperStructureBase() {
        statemachine = new StateMachine("SuperStructure");

        registeredStates = new ArrayList<>();

        statemachine.setInitialState(
            registerState(
                this.idle(), 
                Constants.IDLE_STATE_NAME
            )
        );
    }

    /**
     * Initialize and register a state to the superstructure statemachine.
     * @param cmd The command of the State.
     * @param stateName The name of the State which you are registering.
     * @return The state that has just been initialized and registered.
     */
    protected State registerState(Command cmd, StateName stateName){
        var state = statemachine.addState(cmd, stateName);
        registeredStates.add(state);
        return state;
    }

    /**
     * Adds a transition from every state other than the current state to the current state.
     * @param to The state being switched to.
     * @param trigger The trigger for the state transition.
     */
    protected void switchFromAnyOtherThanMyself(State to, Trigger trigger){
        registeredStates.forEach((from) -> {if (from != to) from.switchTo(to).when(trigger);});
    }

    /**
     * Add a binding of a state to a trigger.
     * @param to The state to activate on the trigger activation.
     * @param binding The trigger which trigger's the state.
     */
    protected void configureBinding(State to, Trigger binding){
        switchFromAnyOtherThanMyself(to, binding);
    }

    /**
     * Get the Command to execute the statemachine.
     * @return The command which holds the statemachine.
     */
    public Command getCommand(){
        return statemachine;
    }
}
