package frc.robot.subsystems.drivetrain.configsStructure;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CommonModuleConstants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.ModuleConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * Chassis-level container for swerve drivetrain configuration.
 * <p>
 * This class gathers together:
 * <ul>
 *   <li>Gyro I/O implementation used for robot heading</li>
 *   <li>Per-module constants (IDs, offsets, translations, motor configs)</li>
 *   <li>Chassis speed limits and tuning values</li>
 *   <li>A PathPlanner {@link RobotConfig} derived from the above</li>
 * </ul>
 * The {@link RobotConfig} is constructed from the module translations and generic
 * module configuration so PathPlanner can perform proper kinematics and trajectory
 * constraints for the robot.
 * <p>
 * Example usage:
 * <pre>
 *   ChassisConstants constants = new ChassisConstants(gyro, modules, speedCfg, generic, ppCfg, gyroType, gyroPort(if using pigeon );
 * </pre>
 */
public class ChassisConstants {

    public static final double LOOP_TIME_SECONDS = 0.02;

    public final ModuleConstants[] MODULE_CONSTANTS;

    public final ChassisSpeedConfig SPEED_CONFIG;

    public final RobotConfig ROBOT_CONFIG;

    public final PPChassisConfig PP_CONFIG;

    public final CommonModuleConstants COMMON_MODULE_CONSTANTS;

    public final double MAX_ANGULAR_SPEED; //rad/s

    public final double MIN_ANGULAR_SPEED; //rad/s

    public final int GYRO_PORT;

    public final PathConstraints PATH_FINDING_CONSTRAINTS;

    /**
     * Create chassis constants and derive the PathPlanner {@link RobotConfig} from the
     * provided module and chassis parameters.
     *
     * @param MODULE_CONSTANTS      array of swerve module constants
     * @param SPEED_CONFIG  chassis-level speed limits and tuning parameters
     * @param COMMON_MODULE_CONSTANTS      generic module configuration for all the modules
     * @param PP_CONFIG     chassis parameters for PathPlanner (mass, MOI, etc.)
     * @param GYRO_PORT port/canID number of the gyro.
     */
    public ChassisConstants(ModuleConstants[] MODULE_CONSTANTS,
                            ChassisSpeedConfig SPEED_CONFIG, CommonModuleConstants COMMON_MODULE_CONSTANTS,
                            PPChassisConfig PP_CONFIG, int GYRO_PORT, PathConstraints pathFindingConstraints) {
        this.MODULE_CONSTANTS = MODULE_CONSTANTS;
        this.SPEED_CONFIG = SPEED_CONFIG;
        this.PP_CONFIG = PP_CONFIG;
        this.COMMON_MODULE_CONSTANTS = COMMON_MODULE_CONSTANTS;
        this.GYRO_PORT = GYRO_PORT;
        this.PATH_FINDING_CONSTRAINTS = pathFindingConstraints;

        try{
           ROBOT_CONFIG = RobotConfig.fromGUISettings();
       } catch (IOException | ParseException e){
           throw new IllegalArgumentException("PP GUI settings not configured");

       }

        MAX_ANGULAR_SPEED = SPEED_CONFIG.maxLinearSpeed() / MODULE_CONSTANTS[0].TRANSLATION().getNorm();

        MIN_ANGULAR_SPEED = SPEED_CONFIG.minLinearSpeed() / MODULE_CONSTANTS[0].TRANSLATION().getNorm();
    }

    /**
     * Determine whether autonomous paths should be mirrored for the current alliance.
     * <p>
     * PathPlanner expects paths to be authored from the Blue alliance perspective. This
     * helper returns true for Red, so the path can be flipped at runtime. If the alliance
     * is unknown (e.g., in simulation or before FMS connection), this method returns false
     * and paths are not flipped.
     *
     * @return true when the robot is on Red alliance; false otherwise
     */
    public static boolean shouldFlipPath(){
        //         If no alliance is set, do not flip the path.
        return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    }



    /**
     * Chassis-level linear speed limits for the drivetrain.
     *
     * @param minLinearSpeed minimum commanded linear speed in m/s
     * @param maxLinearSpeed maximum commanded linear speed in m/s
     */
    public record ChassisSpeedConfig(double minLinearSpeed, double maxLinearSpeed) {

    }


    /**
     * record containing the relevant information to PathPlanner
     * @param PID_CONSTANTS
     * @param ANGULAR_PID_CONSTANTS
     */
    public record PPChassisConfig( PIDConstants PID_CONSTANTS,
                                   PIDConstants ANGULAR_PID_CONSTANTS) {

    }



}
