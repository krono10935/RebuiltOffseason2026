// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.StateMachine.StateName;

public class HoodSubsystem extends SubsystemBase {

  private final HoodIO io;
  private final HoodInputsAutoLogged inputs;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    io = RobotBase.isReal() ? new HoodIOReal() : new HoodIOSim();
    inputs = new HoodInputsAutoLogged();

  }

    /**
     * command the io to set the wanted hood angle
     * @param angle the wanted angle
     */
    public void setAngle(Rotation2d angle){
      io.setAngle(angle);
    }

    /**
     * commands the io to hold the wanted hood angle
     * @param angle the wanted angle
     */
    public void holdAngle(Rotation2d angle){
      io.holdAngle(angle);
    }

    /**
     * commands the io to stop the hood
     */
    public void stop(){
      io.stop();
    }

    /**
     * @return whether the hood is at goal (within tolerance)
     */
    public boolean isAtGoal(){
      return inputs.isAtGoal;
    }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs); 
  }

    /**
   * @param angleSupplier the wanted angle
   * @return the command the set the hood's angle
   */
  public Command setAngleCommand(Supplier<Rotation2d> angleSupplier){
    return Commands.run(() -> this.setAngle(angleSupplier.get()), this).repeatedly().withName("setHoodAngle");
  }

  /**
   * @param angleSupplier the wanted angle
   * @return the command the hold the hood's angle
   */
  public Command holdAngleCommand(Supplier<Rotation2d> angleSupplier){
    return Commands.run(() -> this.holdAngle(angleSupplier.get()), this).repeatedly().withName("holdHoodAngle");
  }

  /**
  * @return the command to stop the flywheel
  */
  public Command stopCommand(){
    return Commands.run(this::stop, this).withName("stopHood");
  }

  public Command disableHoodCommand()
  {
    StateMachine stateMachine = new StateMachine("DisableHood_StateMachine");

    State zeroHoodState = stateMachine.addState(setAngleCommand(() -> HoodConstants.HOOD_CLOSE_ANGLE), HoodConstants.ZERO_HOOD_STATE_NAME);

    State stopHoodState = stateMachine.addState(stopCommand(), HoodConstants.STOP_HOOD_STATE_NAME);

    stateMachine.setInitialState(zeroHoodState);

    zeroHoodState.switchTo(stopHoodState).when(this::isAtGoal);

    return stateMachine;
  }
}
