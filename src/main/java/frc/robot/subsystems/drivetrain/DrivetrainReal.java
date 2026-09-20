package frc.robot.subsystems.drivetrain;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.DriveToPoseConstants;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;
import frc.lib.field.AllianceFlipUtil;
import frc.robot.GeneralRobotState;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;


import java.util.function.Supplier;


public class DrivetrainReal extends Drivetrain {

    @AutoLog
    public static class DrivetrainInputs {
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    }

    private final Supplier<Double> batteryVoltageSupplier;

    private final SwerveSetpointGenerator setpointGenerator;

    private SwerveSetpoint previousSetpoint;

    private static final DriveFeedforwards ZEROS = DriveFeedforwards.zeros(4);

    public DrivetrainReal(Supplier<Double> batteryVoltageSupplier, ChassisConstants constants) {
        super(constants);

        this.batteryVoltageSupplier = batteryVoltageSupplier;


        setpointGenerator = new SwerveSetpointGenerator(
                constants.ROBOT_CONFIG,
                constants.COMMON_MODULE_CONSTANTS.maxSteerSpeed()

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
     * configures the AutoBuilder for PP
     */
    private void configPathPlanner(RobotConfig config){

        // Configure AutoBuilder last
        AutoBuilder.configure(
                GeneralRobotState.getInstance()::getEstimatedPose, // Robot pose supplier
                GeneralRobotState.getInstance()::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
                GeneralRobotState.getInstance().getChassisSpeedsSupplier(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveWithoutPP(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
     * Drives the robot at relative speed
     *Needs to be called continuously
     * @param speeds the target speed of the robot
     */
    @SuppressWarnings("unused")
    public void drive(ChassisSpeeds speeds) {

        boolean hasBalls = GeneralRobotState.getInstance().hasGamePiece();

        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, speeds, null,
                ChassisConstants.LOOP_TIME_SECONDS, batteryVoltageSupplier.get());

        for (int i = 0; i < 4; i++){
            var targetSpeed = previousSetpoint.moduleStates()[i];
            if (hasBalls) io[i].setTargetStateWithBalls(targetSpeed); 
            else io[i].setTargetState(targetSpeed);
        }
        Logger.recordOutput("drivetrain/requested speeds", speeds);
        Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());
    }

    /**
     * set the speeds which regular kinematics
     * @param speeds the target speed of the robot
     */
    public void driveWithoutPP(ChassisSpeeds speeds) {
        var targetSpeeds = kinematics.toWheelSpeeds(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetSpeeds, constants.SPEED_CONFIG.maxLinearSpeed());
        for (int i = 0; i < 4; i++){
            targetSpeeds[i].optimize(io[i].getState().angle);
            targetSpeeds[i].cosineScale(io[i].getState().angle);
        }
        previousSetpoint = new SwerveSetpoint(speeds,kinematics.toSwerveModuleStates(speeds),ZEROS);

        for (int i = 0; i < 4; i++){

            io[i].setTargetState(targetSpeeds[i]);
        }
        Logger.recordOutput("drivetrain/requested speeds", speeds);
        Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());
    }

    /**
     * Function which stops the robot immediately
     */
    public void stop(){

        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(),kinematics.toSwerveModuleStates(new ChassisSpeeds()),ZEROS);

        for (int i = 0; i < 4; i++){
            io[i].setTargetState(previousSetpoint.moduleStates()[i]);
        }

        Logger.recordOutput("drivetrain/requested speeds", new ChassisSpeeds());
        Logger.recordOutput("drivetrain/target speeds", previousSetpoint.robotRelativeSpeeds());
        Logger.recordOutput("drivetrain/target states", previousSetpoint.moduleStates());
    }


    /**
     * Resets the gyros
     */
    public void resetGyro(){
        gyro.reset(new Pose2d(new Translation2d(), AllianceFlipUtil.apply(new Rotation2d())));
    }

    /**
     * Adds the vision measurement
     *
     * @param pose      the position where the vision think the robot is there
     * @param timestamp the time when the pose was taken
     * @param stdDevs   A Vector with 3 parameters in the following order:
     *                  X standard deviation (in meters).
     *                  Y standard deviation (in meters).
     *                  Theta standard deviation (in radians).
     */
    // @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        Logger.recordOutput("VisionMeasurement/Pose", pose);
        Logger.recordOutput("VisionMeasurement/timestamp", timestamp);
        Logger.recordOutput("VisionMeasurement/stdDevs", stdDevs);

        GeneralRobotState.getInstance().addVisionMeasurement(pose, timestamp, stdDevs);
    }


    /**
     * @return the constants the driveTrain was created with
     */
    public ChassisConstants getConstants() {
        return constants;
    }

    /**
     * Return the latest gyro angle
     * (counterclockwise positive)
     *
     * @return the gyro angle
     */
    public Rotation2d getGyroAngle() {
        return gyroInputs.pose.getRotation();
    }

    /**
     * Return the latest speeds of the robot
     *
     * @return speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return inputs.speeds;
    }

    /**
     * Set if the module is Brake or Coast
     * @param isBrake whether the module motor should resist outside change in disable
     */
    public void setBrakeMode(boolean isBrake){
        for (SwerveModuleIO module : io){
            module.setBrakeMode(isBrake);
        }
    }

    /**
     *
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


    @Override
    public void periodic() {
        this.gyro.updateInputs(gyroInputs);

        for (int i = 0; i < 4; i++){
            io[i].update();
            this.inputs.moduleStates[i] = io[i].getState();
            modulePositions[i] = io[i].getPosition();
        }

        inputs.speeds = kinematics.toChassisSpeeds(this.inputs.moduleStates);

        GeneralRobotState.getInstance().update(getGyroAngle(), modulePositions);

        // this.gyro.getEstimatedPosition().ifPresent((
        //         pose -> poseEstimator.addVisionMeasurement(pose.pose(), Timer.getTimestamp(), pose.stdDevs())));

        Logger.processInputs("drivetrain", inputs);
        Logger.processInputs("drivetrain/gyro", gyroInputs);
        // Logger.recordOutput("drivetrain/estimated pose", 
        //     GeneralRobotState.getInstance().getEstimatedPosition());

        String currentCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();

        Logger.recordOutput("drivetrain/current command", currentCommand);
    }

}

