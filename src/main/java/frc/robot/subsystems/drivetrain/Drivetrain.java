// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.field.AllianceFlipUtil;
import frc.robot.GeneralRobotState;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.gyro.GyroIOSim;
import frc.robot.subsystems.drivetrain.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.drivetrain.module.SwerveModuleBasic;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;

public abstract class Drivetrain extends SubsystemBase {
  protected final ChassisConstants constants;

  protected final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

  protected final SwerveModuleIO[] io = new SwerveModuleIO[4];

  protected final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  protected final SwerveDriveKinematics kinematics;

  protected final GyroIO gyro;

  protected final GyroInputsAutoLogged gyroInputs;

  public Drivetrain(ChassisConstants constants) {
    this.constants = constants;

    this.gyro = RobotBase.isReal() ? new GyroIOPigeon(constants.GYRO_PORT)
        : new GyroIOSim(GeneralRobotState.getInstance().getChassisSpeedsSupplier());

    this.gyroInputs = new GyroInputsAutoLogged();

    for(int i = 0; i < 4; i++){
      io[i] = new SwerveModuleBasic(constants.MODULE_CONSTANTS[i]);
      inputs.moduleStates[i] = io[i].getState();
      modulePositions[i] = io[i].getPosition();
    }

    kinematics = new SwerveDriveKinematics(constants.ROBOT_CONFIG.moduleLocations);

    GeneralRobotState.getInstance().setChassisSpeedsSupplier(
      () -> kinematics.toChassisSpeeds(inputs.moduleStates)
    );
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

    Logger.processInputs(getName(), inputs);
    Logger.processInputs(getName() + "/gyro", gyroInputs);


    // TODO make a custom periodic which does this in the GeneralRobotState
    // Logger.recordOutput("drivetrain/estimated pose", getEstimatedPosition())
    // field.setRobotPose(getEstimatedPosition());

    String currentCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();

    Logger.recordOutput(getName() + "/current command", currentCommand);


    GeneralRobotState.getInstance().updatePoseEstimator(getGyroAngle(), modulePositions);
  }

  /**
   * Resets the gyro
   */
  public void resetGyro(){
      gyro.reset(new Pose2d(new Translation2d(), AllianceFlipUtil.apply(new Rotation2d())));
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
   * Set if the module is Brake or Coast
   * @param isBrake whether the module motor should resist outside change in disable
   */
  public void setBrakeMode(boolean isBrake){
      for (SwerveModuleIO module : io){
          module.setBrakeMode(isBrake);
      }
  }
}
