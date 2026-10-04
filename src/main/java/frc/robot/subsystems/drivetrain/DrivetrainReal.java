package frc.robot.subsystems.drivetrain;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.DriveToPoseConstants;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GeneralRobotState;
import frc.robot.subsystems.drivetrain.chassisSpeedsCalculator.ControllerChassisSpeedsCalculator;
import frc.robot.subsystems.drivetrain.chassisSpeedsCalculator.HomeToSupplierChassisSpeedsCalculator;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.drivetrain.constants.DrivetrainConstants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class DrivetrainReal extends Drivetrain {

    @AutoLog
    public static class DrivetrainInputs {
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    }

    /**
     * DriveSpeeds contains a target ChassisSpeeds and whether or not to treat it as Field relative
     */
    public record DriveSpeeds(ChassisSpeeds targetSpeeds, boolean isFieldRelative) {

        /**
         * Construct a Field relative DriveSpeeds object
         * @param goalSpeeds The chassis speeds on the field
         */
        public DriveSpeeds (ChassisSpeeds goalSpeeds){
            this(goalSpeeds, true);
        }
    }

    private final Supplier<Double> batteryVoltageSupplier;

    private final SwerveSetpointGenerator setpointGenerator;

    private SwerveSetpoint previousSetpoint;

    private static final DriveFeedforwards ZEROS = DriveFeedforwards.zeros(4);

    private DriveSpeeds goalSpeeds = new DriveSpeeds(new ChassisSpeeds(), false);

    public DrivetrainReal(Supplier<Double> batteryVoltageSupplier, ChassisConstants constants) {
        super(constants);

        this.batteryVoltageSupplier = batteryVoltageSupplier;


        setpointGenerator = new SwerveSetpointGenerator(
                constants.ROBOT_CONFIG,
                constants.GENERIC_MODULE_CONFIG.getMaxSteerSpeed()

        );

        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), inputs.moduleStates,
                DriveFeedforwards.zeros(inputs.moduleStates.length));

        GeneralRobotState.getInstance().setPoseEstimator(
            new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), modulePositions,
                Pose2d.kZero));

        configPathPlanner(constants.ROBOT_CONFIG);

        setBrakeMode(false);
    }

    /**
     * Configurates the AutoBuilder for pathplannner
     * @param config The robot config taken from pathplanner
     */
    private void configPathPlanner(RobotConfig config){

        // Configure AutoBuilder last
        AutoBuilder.configure(
                GeneralRobotState.getInstance()::getEstimatedPose, // Robot pose supplier
                GeneralRobotState.getInstance()::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
                GeneralRobotState.getInstance().getChassisSpeedsSupplier(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> 
                    setGoalSpeeds(
                        new DriveSpeeds(speeds, false)
                    ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            constants.PP_CONFIG.PID_CONSTANTS(), constants.PP_CONFIG.ANGULAR_PID_CONSTANTS() // Rotation PID constants
                ),
                config, // The robot configuration
                ChassisConstants::shouldFlipPath,
                "driveToPose",
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    var posesArr = poses.toArray(new Pose2d[0]);

                    Logger.recordOutput("drivetrain/PathPlanner/active path", posesArr);
                }
        );

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    Logger.recordOutput("drivetrain/PathPlanner/target pose", pose);
                }
        );
    }

    /**
     * Function which stops the robot immediately
     */
    public void stop(){

        previousSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(), 
            kinematics.toSwerveModuleStates(new ChassisSpeeds()), 
            ZEROS
        );

        for (int i = 0; i < 4; i++){
            io[i].setTargetState(previousSetpoint.moduleStates()[i]);
        }

        Logger.recordOutput("drivetrain/requested speeds", new ChassisSpeeds());
        Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());
    }

    public Command stopCommand(){
        return Commands.run(this::stop, this);
    }

    /**
     * Drives to a pose on the field
     * @param goalPose goal position to drive to
     * @return a command which drives the chasis to a position
     */
    public Command driveToPose(Pose2d goalPose){
        return AutoBuilder.pathfindToPose(goalPose, constants.PATH_FINDING_CONSTRAINTS,
                0, DriveToPoseConstants.DISTANCE_TO_STOP_PP);
    }

    /**
     * @return a command that resets the gyro
     */
    public Command resetGyroCommand(){
        return new InstantCommand(this::resetGyro).ignoringDisable(true);
    }

    /**
     * @return A command which puts the swerve in defense mode
     */
    public Command defenseModeCommand(){
        return Commands.run(this::defenseMode, this);
    }

    /**
     * Put the swerve modules in an X configuration in order to resist movement.
     */
    private void defenseMode(){
        for (int i = 0; i < 4; i++){
            io[i].setTargetState(DrivetrainConstants.DEFENSE_MODE_STATES[i]);
        }

        goalSpeeds = null;
        previousSetpoint = DrivetrainConstants.DEFENSE_MODE_SETPOINT;
    }

    /**
     * Command a new goal speed for the Swerve
     * @param goalSpeeds the speeds that the drivetrain needs to follow
     */
    public void setGoalSpeeds(DriveSpeeds goalSpeeds){
        this.goalSpeeds = goalSpeeds;
    }

    /**
     * Make the swerve move accordingly to the goal speeds.
     */
    private void drive(){
        if (goalSpeeds == null){
            Logger.recordOutput("drivetrain/requested speeds", new ChassisSpeeds());
            Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
            Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());
            return;
        }

        boolean hasBalls = GeneralRobotState.getInstance().hasBalls();

        ChassisSpeeds speeds = goalSpeeds.targetSpeeds();

        if (goalSpeeds.isFieldRelative()){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        }

        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, speeds, null,
                ChassisConstants.LOOP_TIME_SECONDS, batteryVoltageSupplier.get());

        for (int i = 0; i < 4; i++){
            var targetSpeed = previousSetpoint.moduleStates()[i];
            if (hasBalls) io[i].setTargetState(targetSpeed, DrivetrainConstants.BALLS_CLOSED_LOOP_SLOT);
            else io[i].setTargetState(targetSpeed);
        }
        Logger.recordOutput("drivetrain/requested speeds", speeds);
        Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());

    }

    @Override
    public void periodic() {
        super.periodic();
        drive();
    }

    /**
     * Build a command that drives the robot field relative
     * @return A command that when run drives the robot field relative
     */
    public Command driveCommand(){
        final double moduleDistanceFromCenterMeters = constants.MODULE_CONSTANTS[0].TRANSLATION().getNorm();

        ControllerChassisSpeedsCalculator chassisSpeedsCalculator =
            new ControllerChassisSpeedsCalculator(
                constants.SPEED_CONFIG, 
                moduleDistanceFromCenterMeters
            );

        return driveBySpeedsSupplier(
            () -> new DriveSpeeds(
                chassisSpeedsCalculator.getControllerInputs()))
        .withName("DriveCommand");
    }

    /**
     * Build a command that drives robot relative
     * @return A command that when runs drives robot relative.
     */
    public Command driveRobotRelativeCommand(){
        final double moduleDistanceFromCenterMeters = constants.MODULE_CONSTANTS[0].TRANSLATION().getNorm();

        ControllerChassisSpeedsCalculator chassisSpeedsCalculator =
            new ControllerChassisSpeedsCalculator(
                constants.SPEED_CONFIG, 
                moduleDistanceFromCenterMeters
            );

        return driveBySpeedsSupplier(
            () -> new DriveSpeeds(
                chassisSpeedsCalculator.getControllerInputs(), 
                false))
        .withName("DriveRobotRelative");
    }

    /**
     * Build a command that makes the drivetrain home to an angle
     * @param angularSupplier the supplier of angles that the robot should follow
     * @return A command that makes the drivetrain home to an angle
     */
    public Command driveAndHomeToAngleSupplier(Supplier<Rotation2d> angularSupplier){
        final double moduleDistanceFromCenterMeters = constants.MODULE_CONSTANTS[0].TRANSLATION().getNorm();
        HomeToSupplierChassisSpeedsCalculator homeToSupplierChassisSpeedsCalculator = 
            new HomeToSupplierChassisSpeedsCalculator(
                constants.SPEED_CONFIG, 
                moduleDistanceFromCenterMeters, 
                angularSupplier,
                "DriveAndHomeToAngleSupplier");

   
        return driveBySpeedsSupplier(
            () -> new DriveSpeeds(
                homeToSupplierChassisSpeedsCalculator.getControllerInputs(), 
                false))
        .withName("DriveAndHomeToAngle");
    }

    public Command driveBySpeedsSupplier(Supplier<DriveSpeeds> speedsSupplier){
        return Commands.runEnd(() -> setGoalSpeeds(speedsSupplier.get()), this::stop, this)
        .withName("DriveBySpeedsSupplier");
    }

}