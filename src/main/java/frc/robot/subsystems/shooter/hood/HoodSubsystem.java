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
import frc.lib.math.IsNear;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;

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

  /**
   * @return whether the hood is closed (within tolerance)
   */
  public boolean isHoodClosed(){
    return IsNear.isNear(inputs.currentAngle, HoodConstants.HOOD_CLOSE_ANGLE, HoodConstants.DEGREE_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    
    String commandName = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();
    Logger.recordOutput("Hood/Command", commandName);
  }

  /**
  * @param angleSupplier the wanted angle
  * @return the command the set the hood's angle
  */
  public Command setAngleCommand(Supplier<Rotation2d> angleSupplier){
    return Commands.runEnd(() -> this.setAngle(angleSupplier.get()),()->this.setAngle(new Rotation2d(0)), this).withName("setHoodAngle");
  }

  /**
   * @param angleSupplier the wanted angle
   * @return the command the hold the hood's angle
   */
  public Command holdAngleCommand(Supplier<Rotation2d> angleSupplier) {
    return Commands.runEnd(() -> this.holdAngle(angleSupplier.get()),()->this.setAngle(new Rotation2d(0)), this).withName("holdHoodAngle");
  }

  /**
  * @return the command to stop the flywheel
  */
  public Command stopCommand() {
    return Commands.run(this::stop, this).withName("stopHood");
  }

  public Command disableHoodCommand() {
    StateMachine disableHoodStateMachine = new StateMachine("DisableHood_StateMachine");

    State zeroHoodState = disableHoodStateMachine.addState(setAngleCommand(() -> HoodConstants.HOOD_CLOSE_ANGLE), HoodConstants.ZERO_HOOD_STATE_NAME);

    State stopHoodState = disableHoodStateMachine.addState(stopCommand(), HoodConstants.STOP_HOOD_STATE_NAME);

    disableHoodStateMachine.setInitialState(zeroHoodState);

    zeroHoodState.switchTo(stopHoodState).when(this::isHoodClosed);

    return disableHoodStateMachine;
  }
}
