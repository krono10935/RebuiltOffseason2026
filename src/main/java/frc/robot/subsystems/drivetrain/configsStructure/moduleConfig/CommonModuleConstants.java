package frc.robot.subsystems.drivetrain.configsStructure.moduleConfig;

import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;

/**
 * Generic per-chassis configuration for swerve modules.
 * <p>
 * This record is intended to be shared by all swerve modules on the same chassis.
 * It bundles together the drive and steer motor config baselines
 * {@link ModuleConfig} used for PP
 *
 * @param DRIVE_CONFIG config for drive motor
 * @param STEER_CONFIG config for steer motor
 * @param STEER_SPEED_REDUCTION reduction of the steer motor speed, in range of 0-1, multiplies the max steer speed
 */
public record CommonModuleConstants(BasicTalonFXConfig DRIVE_CONFIG, BasicTalonFXConfig STEER_CONFIG, double STEER_SPEED_REDUCTION) {

    /**
     *
     * @return max speed of the steer motor in radians per second
     */
    public double maxSteerSpeed(){
        return STEER_CONFIG.motorConfig.motorType.freeSpeedRadPerSec/
                STEER_CONFIG.motorConfig.gearRatio * STEER_SPEED_REDUCTION;
    }


}
