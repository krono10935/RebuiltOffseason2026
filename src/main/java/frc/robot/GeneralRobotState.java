package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GeneralRobotState {

  private static GeneralRobotState instance = null;

  public static GeneralRobotState getInstance(){
      if (instance == null){
          instance = new GeneralRobotState();
      }
      return instance;
  }

  private final Alliance alliance;
  private final CommandXboxController controller;
  private SwerveDrivePoseEstimator poseEstimator;
  private Supplier<ChassisSpeeds> speedsSupplier;
  private final Field2d field;
  private boolean hasBalls;
  private Alert allianceAbsentAlert;

  private GeneralRobotState(){
    Optional<Alliance> allianceMaybe = DriverStation.getAlliance();

    allianceAbsentAlert = new Alert("Could not get alliance from FMS", AlertType.kWarning);

    if (allianceMaybe.isPresent()){
        alliance = allianceMaybe.get();
    }
    else {
        alliance = Constants.DEFAULT_ALLIANCE;
        allianceAbsentAlert.set(true);
        Logger.recordOutput("DriverStation/Found alliance", false);
    }

    controller = new CommandXboxController(0);

    hasBalls = false;

    field = new Field2d();

    poseEstimator = null; // Wait for drivetrain to configurate the poseEstimator

    speedsSupplier = null; // Wait for the drivetrain to configurate the poseEstimator

    SmartDashboard.putData("robotPose", field);

  }

  /**
   * configure the pose estimator (used from the drivetrain)
   * @param poseEstimator the pose estimator which you want to use
   */
  public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator){
    this.poseEstimator = poseEstimator;
  }

  /**
   * Get the estimated pose of the robot
   * @return the estimated position of the robot on the field
   */
  public Pose2d getEstimatedPose(){
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Update the pose estimator (used from the drivetrain)
   * @param gyroAngle the current gyroAngle
   * @param wheelPositions the current wheel positions
   */
  public void updatePoseEstimator(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions){
    poseEstimator.update(gyroAngle, wheelPositions);
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
  public void addVisionMeasurement(
        Pose2d pose,
        double timeStamp,
        Matrix<N3, N1> stdDevs){

    Logger.recordOutput("VisionMeasurement/Pose", pose);
    Logger.recordOutput("VisionMeasurement/timestamp", timeStamp);
    Logger.recordOutput("VisionMeasurement/stdDevs", stdDevs);

    poseEstimator.addVisionMeasurement(
      pose,
      timeStamp, 
      stdDevs
    );
  }

  /**
   * Reset the position of the pose estimator
   * @param newPose The pose to reset the pose estimator to
   */
  public void resetPoseEstimator(Pose2d newPose){
    poseEstimator.resetPose(newPose);
  }

  /**
   * Get the alliance the robot thinks it's on.
   * @return the alliance the robot thinks its on
   */
  public Alliance getAlliance(){
    return alliance;
  }

  /**
   * Get the controller for the robot
   * @return the controller which commands the robot
   */
  public CommandXboxController getController(){
    return controller;
  }

  /**
   * Get the supplier of the chassis speeds (useful for decoupling the drivetrain with commands)
   * @return The supplier of the chassis speeds
   */
  public Supplier<ChassisSpeeds> getChassisSpeedsSupplier(){
    return speedsSupplier;
  }

  /**
   * sets the chassis speeds supplier for the robot (used by drivetrain)
   * @param speedsSupplier the supplier of the chassis speeds
   */
  public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> speedsSupplier){
    this.speedsSupplier = speedsSupplier;
  }

  /**
   * Get the field that is being displayed to the driver on smartdashboard
   * @return The field
   */
  public Field2d getField(){
    return field;
  }

  /**
   * Update whether or not the robot has a gamepiece
   * @param hasBalls
   */
  public void updateHasBalls(boolean hasBalls){
    this.hasBalls = hasBalls;
  }

  /**
   * Get if the robot has balls
   * @return whether or not the robot has balls
   */
  public boolean hasBalls(){
    return hasBalls || !Constants.USE_OBJECT_DETECTION;
  }

  /**
   * Clear the displayed path
   */
  public void clearFiledPath(){
      field.getObject("path").setPoses();
  }

    /**
   * @param path the path to display
   */
  public void addPathToField(ArrayList<Pose2d> path){
      field.getObject("path").setPoses(path);
  }
}
