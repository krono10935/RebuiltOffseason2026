package frc.robot.subsystems.drivetrain.constants;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class DrivetrainConstants {
    private static final Rotation2d FRONT_LEFT_SPIN_ANGLE = Rotation2d.fromDegrees(135);


    /** If you want to understand the numbers please draw the module angles vectors */
    public static final Rotation2d[] SPIN_STEER_ANGLES = {
        FRONT_LEFT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(0)), // FL
        FRONT_LEFT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(1)), // FR
        FRONT_LEFT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(3)), // BL
        FRONT_LEFT_SPIN_ANGLE.minus(Rotation2d.kCW_90deg.times(2)) // BR
    };

    public static final SwerveModuleState[] DEFENSE_MODE_STATES = getDefenseModeStates();

    public static final SwerveSetpoint DEFENSE_MODE_SETPOINT = new SwerveSetpoint(
        new ChassisSpeeds(), DEFENSE_MODE_STATES, DriveFeedforwards.zeros(4)
    );

    public static final int BALLS_CLOSED_LOOP_SLOT = 1;

    /** Angular PID Controller */
    public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0, 0, 0,
     new Constraints(100, 1000));

    /** Small threshold to eliminate rotation jitter */
    public static final double ANGULAR_DEADBAND =
            Rotation2d.fromDegrees(1).getRadians();

    /**
     * Make the states for the swerve to make an x shape with its wheels.
     * @return The configuration of the swerve wheels to make x shape
     */
    private static SwerveModuleState[] getDefenseModeStates(){
        SwerveModuleState[] defenseModeStates = new SwerveModuleState[4];

        Rotation2d PERPINDICULAR = Rotation2d.kCW_90deg;


        // For each module the speed needs to be perpendicular to the spin configuration
        // to have the least amount of the velocity vector in the radial direction
        // and since all of the vectors are in opposite directions they cancel out
        for (int i = 0; i < 4; i++){
            defenseModeStates[i] = 
                new SwerveModuleState(0,
                 SPIN_STEER_ANGLES[i].rotateBy(PERPINDICULAR));
        }

        return defenseModeStates;
    }
}
