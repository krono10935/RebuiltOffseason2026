package frc.robot.subsystems.drivetrain.constants;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DrivetrainConstants {
    public static final SwerveModuleState[] defenseModeStates = getDefenseModeStates();

    public static final SwerveSetpoint defenseModeSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), defenseModeStates, DriveFeedforwards.zeros(4)
    );

    /** Angular PID Controller */
    public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0, 0, 0, null);

    /** Small threshold to eliminate rotation jitter */
    public static final double ANGULAR_DEADBAND =
            Rotation2d.fromDegrees(1).getRadians();

    private static SwerveModuleState[] getDefenseModeStates(){
        SwerveModuleState[] defenseModeStates = new SwerveModuleState[4];

        // For each module the speed needs to be perpendicular to the spin configuration
        // to have the least amount of the velocity vector in the radial direction
        // and since all of the vectors are in opposite directions they cancel out
        for (int i = 0; i < 4; i++){
            defenseModeStates[i] = 
                new SwerveModuleState(0,
                 SysIDConstants.SYSID_SPIN_STEER_ANGLES[i].plus(Rotation2d.kCW_90deg));
        }

        return defenseModeStates;
    }
}
