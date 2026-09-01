// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
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
    public void setSpeed(double mps){
      io.setSpeed(mps);
    }

    /**
     * command the io to hold the wanted flywheel speed
     * @param mps the wanted speed in meters/second
     */
    public void holdSpeed(double mps){
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

  
}
