package frc.robot.subsystems.drivetrain.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class SysIDConstants {

    /** If you want to understand the numbers please draw the module angles vectors */
    public static final Rotation2d[] SYSID_SPIN_STEER_ANGLES = DrivetrainConstants.SPIN_STEER_ANGLES;

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
