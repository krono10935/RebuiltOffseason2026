// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;

public interface GyroIO {

    /**
     * get the estimated position using the gyro's accelerometer
     * @return the position if implemented
     */
    Optional<GyroPoseOutput> getEstimatedPosition();

    /**
     * Reset the gyro angle to another angle
     */
    void reset(Pose2d pose);

    /**
     * @return Get the new gyro angle
     */
    Rotation2d update();
    
    record GyroPoseOutput(Pose2d pose, Matrix<N3, N1> stdDevs){}
}
