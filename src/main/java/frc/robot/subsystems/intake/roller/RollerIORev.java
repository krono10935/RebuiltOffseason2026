// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RollerIORev implements RollerIO {
  private final SparkMax motorOne;
  private final SparkMax motorTwo;

  /** Creates a new RollerIORev. */
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

  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.motorOneTempC = motorOne.getMotorTemperature();
    inputs.motorTwoTempC = motorTwo.getMotorTemperature();

    inputs.speedMPS = motorOne.getEncoder().getVelocity();
  }


}
