// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  
  private final IndexerIO io;
  private final IndexerInputsAutoLogged inputs;
  
  /** Creates a new Indexer. */
  public IndexerSubsystem() {
    io = RobotBase.isReal() ? new IndexerIOCTRE() : null;

    inputs = new IndexerInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  /**
   * tell the hardware to deactivate
   */
  public void stop(){
    io.stop();
  }

  /**
   * tell the hardware to spin forward
   */
  public void spinForward(){
    io.setDutyCycle(IndexerConstants.DUTY_CYCLE_FORWARD);
  }

  /**
   * tell the hardware to spin backward
   */
  public void spinBackward(){
    io.setDutyCycle(IndexerConstants.DUTY_CYCLE_BACKWARD);
  }

  /**
   * get the indexer speed in units of RPS
   * @return indexer speed in RPS
   */
  public double getSpeedRPS(){
    return inputs.speedRPS;
  }

  /**
   * get the temp of the motors in celsius
   * @return motors temp in celsius
   */
  public double[] getTempCel(){
    return inputs.tempCel.clone();
  }

}
