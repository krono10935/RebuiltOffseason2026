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

public class SuperStructureBase extends SubsystemBase {
    /** Creates a new SuperStructure. */
    private final List<State> states;

    private final StateMachine statemachine;

    protected SuperStructureBase() {
        statemachine = new StateMachine("SuperStructure");

        states = new ArrayList<>();

        statemachine.setInitialState(
            registerState(
                this.idle(), 
                Constants.IDLE_STATE_NAME
            )
        );
    }

    protected State registerState(Command cmd, StateName stateName){
        var state = statemachine.addState(cmd, stateName);
        states.add(state);
        return state;
    }

    protected void switchFromAnyOtherThanMyself(State to, Trigger trigger){
        for (State from : states){
            if (from == to) {
                continue;
            }

            from.switchTo(to).when(trigger);
        }
    }

    protected void configureBinding(State to, Trigger binding){
        switchFromAnyOtherThanMyself(to, binding);
    }

    public Command getCommand(){
        return statemachine;
    }
}
