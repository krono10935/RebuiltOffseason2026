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
import edu.wpi.first.wpilibj.DriverStation;
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
  private boolean hasGamePiece;

  private GeneralRobotState(){
    Optional<Alliance> allianceMaybe = DriverStation.getAlliance();

    if (allianceMaybe.isPresent()){
        alliance = allianceMaybe.get();
    }
    else {
        alliance = Constants.DEFAULT_ALLIANCE;
        Logger.recordOutput("DriverStation/Found alliance", false);
    }

    controller = new CommandXboxController(0);

    hasGamePiece = false;

    field = new Field2d();

    poseEstimator = null; // Wait for drivetrain to configurate the poseEstimator

    speedsSupplier = null; // Wait for the drivetrain to configurate the poseEstimator

    SmartDashboard.putData("robotPose", field);

  }

  public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator){
    this.poseEstimator = poseEstimator;
  }


  public Pose2d getEstimatedPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions){
    poseEstimator.update(gyroAngle, wheelPositions);
  }

  public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs){

    poseEstimator.addVisionMeasurement(
      visionRobotPoseMeters,
      timestampSeconds, 
      visionMeasurementStdDevs
    );
  }

  public void resetPoseEstimator(Pose2d newPose){
    poseEstimator.resetPose(newPose);
  }

  public Alliance getAlliance(){
    return alliance;
  }

  public CommandXboxController getController(){
    return controller;
  }

  public Supplier<ChassisSpeeds> getChassisSpeedsSupplier(){
    return speedsSupplier;
  }

  public Field2d getField(){
    return field;
  }

  public void setHasGamePiece(boolean hasGamePiece){
    this.hasGamePiece = hasGamePiece;
  }

  public boolean hasGamePiece(){
    return hasGamePiece || !Constants.USE_OBJECT_DETECTION;
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
