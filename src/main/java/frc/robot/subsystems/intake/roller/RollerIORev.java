// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.MotorLogger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RollerIORev implements RollerIO {
  private final SparkMax motorOne;
  private final SparkMax motorTwo;

  /** Creates a new RollerIORev. (skebob)*/
  public RollerIORev() {
    motorOne = new SparkMax(RollerConstants.MOTOR_ONE_CANID, MotorType.kBrushless);
    motorTwo = new SparkMax(RollerConstants.MOTOR_TWO_CANID, MotorType.kBrushless);

    motorOne.configure(RollerConstants.getLeadConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorTwo.configure(RollerConstants.getFollowerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }



  @Override
  public void setDutyCycle(double dutyCycle) {
    motorOne.getClosedLoopController().setSetpoint(dutyCycle, ControlType.kDutyCycle);
  }

  @Override
  public void stop() {
    motorOne.stopMotor();
  }

  private void logSubsystem(RollerInputs inputs) {
    // Subsystem logs
    Logger.recordOutput(RollerConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Speed (MPS)", inputs.speedMPS);

    // Motor logs
    MotorLogger.log(RollerConstants.LOGGING_DIRECTORY_MOTORS + "RollerMotor1", motorOne);
    MotorLogger.log(RollerConstants.LOGGING_DIRECTORY_MOTORS + "RollerMotor2", motorTwo);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.motorOneTemperatureC = motorOne.getMotorTemperature();
    inputs.motorTwoTemperatureC = motorTwo.getMotorTemperature();

    inputs.speedMPS = motorOne.getEncoder().getVelocity();

    logSubsystem(inputs);
  }


}
