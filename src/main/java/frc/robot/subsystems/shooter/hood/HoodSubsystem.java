// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    
  }
}
