package frc.robot.subsystems.drivetrain.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SysIDConstants {
    private static final Rotation2d FRONT_RIGHT_SPIN_ANGLE = Rotation2d.fromDegrees(135);

    /** If you want to understand the numbers please draw the module angles vectors */
    public static final Rotation2d[] SYSID_SPIN_STEER_ANGLES = {
        FRONT_RIGHT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(0)), // FL
        FRONT_RIGHT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(1)), // FR
        FRONT_RIGHT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(3)), // BL
        FRONT_RIGHT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(2)) // BR
    };

    /**
     * Voltage to use in dynamic mode for SYSID
     */
    public static final double VOLT = 8;

    /**
     * How many volts/second to add per second of the SYSID routine
     */
    public static final double VOLT_RAMP_RATE = 0.5;

    /**
     * How many seconds to perform the test for the sysID routine
     */
    public static final double TIMEOUT = 24;
}
