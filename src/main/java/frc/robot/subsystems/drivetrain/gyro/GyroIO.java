// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public class GyroInputs {
        public Pose2d pose; 
        public Matrix<N3, N1> stdDevs;
    }

    /**
     * Reset the gyro angle to another angle
     */
    void reset(Pose2d pose);

    /**
     * Update gyroInputs
     */
    void updateInputs(GyroInputs inputs);    
}
