// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public class GyroInputs {
        public Rotation2d rotation = new Rotation2d(); 
    }

    /**
     * Reset the rotation of the gyro to a different angle
     * @param rotation the new angle of the gyro
     */
    void reset(Rotation2d rotation);

    /**
     * Update the inputs object
     * @param inputs the inputs object to update
     */
    void updateInputs(GyroInputs inputs);    
}
