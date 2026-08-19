package frc.robot.subsystems.drivetrain.configsStructure.moduleConfig;

import edu.wpi.first.math.geometry.Translation2d;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

/**
 * Simple container record holding all constants required to construct a single
 * swerve module instance.
 *
 * <p>It groups the CANCoder ID and absolute zero offset with the motor configs
 * for driving and steering, along with the module's TRANSLATION from the robot
 * center and a human-readable NAME.</p>
 *
 * @param CAN_CODER_ID      CAN ID of the module's absolute encoder (CANCoder)
 * @param ZERO_OFFSET      absolute angle offset in rotations from the zero position
 * @param DRIVING_CONFIG   configuration for the drive TalonFX motor
 * @param STEERING_CONFIG  configuration for the steer TalonFX motor
 * @param TRANSLATION     module position relative to the robot center
 * @param NAME            descriptive NAME of the module
 */
public record ModuleConstants(
        int CAN_CODER_ID,
        double ZERO_OFFSET, BasicTalonFXConfig DRIVING_CONFIG, BasicTalonFXConfig STEERING_CONFIG,
        Translation2d TRANSLATION, String NAME) {
}



