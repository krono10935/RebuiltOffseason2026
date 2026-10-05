package frc.robot.subsystems.drivetrain.configsStructure.moduleConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.simulation.SimulationConfig;
/**
 * Simple container record holding all constants required to construct a single
 * swerve module instance.
 *
 * <p>It groups the CANCoder ID and absolute zero offset with the motor configs
 * for driving and steering, along with the module's TRANSLATION from the robot
 * center and a human-readable NAME.</p>
 *
 * @param CAN_CODER_ID      CAN ID of the module's absolute encoder (CANCoder)
 * @param STEER_ID          CAN ID of the module's steer motor
 * @param DRIVE_ID          CAN ID of the module's drive motor
 * @param DRIVE_SIM_CONFIG  Drive simulation config
 * @param STEER_SIM_CONFIG  Steer simulation config
 * @param UNIT_CONVERSION   the conversion rate from wheel spins to meters per second
 * @param ZERO_OFFSET      absolute angle offset in rotations from the zero position
 * @param DRIVING_CONFIG   configuration for the drive TalonFX motor
 * @param STEERING_CONFIG  configuration for the steer TalonFX motor
 * @param TRANSLATION     module position relative to the robot center
 * @param NAME            descriptive NAME of the module
 */
public record CTREModuleConstants(
        int CAN_CODER_ID, int STEER_ID, int DRIVE_ID, 
        SimulationConfig DRIVE_SIM_CONFIG, SimulationConfig STEER_SIM_CONFIG, double UNIT_CONVERSION,
        double ZERO_OFFSET, TalonFXConfiguration DRIVING_CONFIG, TalonFXConfiguration STEERING_CONFIG,
        Translation2d TRANSLATION, String NAME) {
}



