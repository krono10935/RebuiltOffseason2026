// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */

  private final FlywheelIO io;
  private final FlywheelInputsAutoLogged inputs;

  public FlywheelSubsystem() {
    io = RobotBase.isReal() ? new FlywheelIOReal() : new FlywheelIOSim();
    inputs = new FlywheelInputsAutoLogged();

  }

   /**
   * commands the io to set the wanted flywheel speed
   * @param mps the wanted speed in meters/second
   */
  private void spinUp(double mps){
    io.spinUp(mps);
  }

  /**
   * command the io to hold the wanted flywheel speed
   * @param mps the wanted speed in meters/second
   */
  private void holdSpeed(double mps){
    io.holdSpeed(mps);
  }

  /**
   * commands the io to stop the flywheel
   */
  public void stop(){
    io.stop();
  }

  /**
   * @return the current flywheel speed in meters/second
   */
  public double getSpeed(){
    return inputs.speedMPS;
  }

  /**
   * @return whether the flywheel is at the goal's speed +- tolerance
   */
  public boolean isAtGoal()
  {
    return inputs.isAtGoal;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  /**
   * @param mps the wanted speed in meters/second
   * @return the command the set the flywheel's speed
   */
  public Command spinUpCommand(Supplier<Double> mpsSupplier){
    return Commands.run(() -> this.spinUp(mpsSupplier.get()), this).repeatedly().withName("spinUpFlywheel");
  }

  /**
   * @param mps the wanted speed in meters/second
   * @return the command the hold the flywheel's speed
   */
  public Command holdSpeedCommand(Supplier<Double> mpsSupplier){
    return Commands.run(() -> this.holdSpeed(mpsSupplier.get()), this).repeatedly().withName("holdFlywheelSpeed");
  }

  /**
   * @return the command to stop the flywheel
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this).withName("stopFlywheel");
  }
}
