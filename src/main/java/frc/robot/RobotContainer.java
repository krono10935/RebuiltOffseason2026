// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import org.json.simple.parser.ParseException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DrivetrainReal;
import frc.robot.subsystems.drivetrain.DrivetrainSysID;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;

public class RobotContainer {

  private static RobotContainer instance = null;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverXboxController;
  // private final CommandXboxController operatorXboxController;
  // private final RollerSubsystem rollerSubsystem;
  private final PivotSubsystem pivotSubsystem;
  // private final IntakeCoordinator intakeCoordinator;

  // private final ShooterSubsystem shooterSubsystem;
  
  private final Vision vision;
  Supplier<Pose2d> lastPoseSupplier = () -> Pose2d.kZero;
  
  private final IndexerSubsystem indexerSubsystem;
  
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  // private final GeneralRobotState generalRobotState;
  
  private final DrivetrainReal drivetrain;

  // private final DrivetrainSysID sysId;
  
    public static RobotContainer getInstance(){
      if (instance == null){
        instance = new RobotContainer();
      }
      return instance;
    }
    
  private RobotContainer() {
    // drivetrain = new DrivetrainReal(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

    // generalRobotState = GeneralRobotState.getInstance();

    // rollerSubsystem = new RollerSubsystem();
    pivotSubsystem = new PivotSubsystem();

    // flywheelSubsystem = new FlywheelSubsystem();
    // hoodSubsystem = new HoodSubsystem();
    // shooterSubsystem = new ShooterSubsystem(flywheelSubsystem, hoodSubsystem);

    // indexerSubsystem = new IndexerSubsystem();

    // intakeCoordinator = new IntakeCoordinator(pivotSubsystem, rollerSubsystem);

    autoChooser = registerNamedCommand();

    driverXboxController = new CommandXboxController(0);

    hoodSubsystem = new HoodSubsystem();
    flywheelSubsystem = new FlywheelSubsystem();

    indexerSubsystem = new IndexerSubsystem();
    
    LoggedNetworkNumber poseX = new LoggedNetworkNumber("PoseX", 3);
    LoggedNetworkNumber poseY = new LoggedNetworkNumber("PoseY",4);
    LoggedNetworkNumber poseYaw = new LoggedNetworkNumber("PoseYaw", 0);

    lastPoseSupplier = () -> new Pose2d(poseX.get(), poseY.get(), Rotation2d.fromDegrees(poseYaw.get()));

    vision = new Vision(GeneralRobotState.getInstance()::addVisionMeasurement, GeneralRobotState.getInstance()::getEstimatedPose);
    

    drivetrain = new DrivetrainReal(
      ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants
    );

    drivetrain.setDefaultCommand(drivetrain.driveCommand());

    driverXboxController.x().whileTrue(pivotSubsystem.openPivot()).onFalse(pivotSubsystem.closePivot());
    driverXboxController.y().onTrue(drivetrain.resetGyroCommand());

    LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("ShooterTweak/HoodAngle", 0);
    LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("ShooterTweak/FlywheelSpeed", 0);

    driverXboxController.a().whileTrue(Commands.parallel(
      flywheelSubsystem.spinUpCommand(flywheelSpeed::get), 
      hoodSubsystem.setAngleCommand(() -> Rotation2d.fromDegrees(hoodAngle.get()))
    ));

    driverXboxController.b().whileTrue(indexerSubsystem.turnOnIndexer()).onFalse(indexerSubsystem.turnOffIndexer());

    // driverXboxController.leftTrigger(0.2).whileTrue(rollerSubsystem.turnOnRoller());
    // driverXboxController.leftBumper().whileTrue(rollerSubsystem.reverseRoller());
    // driverXboxController.b().whileTrue(rollerSubsystem.turnOffRoller());

    // driverXboxController.rightBumper().whileTrue(
    //   flywheelSubsystem.spinUpCommand(() -> 10.0)
    //   .alongWith(hoodSubsystem.setAngleCommand(() -> Rotation2d.fromDegrees(40)),
    //   indexerSubsystem.turnOnIndexer()))
    //   .onFalse(
    //     flywheelSubsystem.stopCommand().alongWith(
    //       hoodSubsystem.disableHoodCommand(), 
    //       indexerSubsystem.turnOffIndexer())
    //   );




    // CommandScheduler.getInstance().schedule(drivetrain.driveCommand().ignoringDisable(true));

    // sysId = new DrivetrainSysID(Constants.CHASSIS_TYPE.COMPBOT.constants);

    // sysId.setBrakeMode(true);


    // driverXboxController.a().onTrue(
    //   Commands.run(() -> sysId.setDriveVoltageAndSpin(SmartDashboard.getNumber("volts", 0))));

    // SmartDashboard.putNumber("volts", 0);

    // driverXboxController.a().whileTrue(sysId.sysIdDynamicDrive(Direction.kForward));
    // driverXboxController.b().whileTrue(sysId.sysIdDynamicDrive(Direction.kReverse));
    // driverXboxController.y().whileTrue(sysId.sysIdQuasistaticDrive(Direction.kForward));
    // driverXboxController.x().whileTrue(sysId.sysIdQuasistaticDrive(Direction.kReverse));

    
    // operatorXboxController = new CommandXboxController(1);

  }

  public CommandXboxController getDriverXboxController() {
      return driverXboxController;
  }

  // public CommandXboxController getOperatorXboxController() {
  //     return operatorXboxController;
  // }

  // public ShooterSubsystem getShooter(){
  //     return shooterSubsystem;
  // }

  // public FlywheelSubsystem getFlywheel() {
  //   return flywheelSubsystem;
  // }

  // public IndexerSubsystem getIndexer(){
  //     return indexerSubsystem;
  // }

  // public IntakeCoordinator getIntake(){
  //     return intakeCoordinator;
  // }

  public DrivetrainReal getDrivetrain(){
    return drivetrain;
  }

  /**
   * @return the chosen autonomous command.
   */
  public Command getAutonomousCommand() {
      var selectedAuto = autoChooser.get();

      // Command autoCommand =
      //         selectedAuto
      //                 .andThen(drivetrain.idle());

      CommandScheduler.getInstance().removeComposedCommand(selectedAuto);

      // return autoCommand.withName(selectedAuto.getName());

      return null;
  }

  /**
   * Displays the path the auto {@code command} takes
   *
   * @param command the command runnning in auto
   */
  private void displayChosenAuto(Command command) {
      if (RobotState.isEnabled()) {
          GeneralRobotState.getInstance().clearFiledPath();
          return;
      }

      List<PathPlannerPath> auto;

      try {
          auto = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
      } catch (IOException | ParseException e) {
          Logger.recordOutput("autoDisplay", e.getMessage());
          GeneralRobotState.getInstance().clearFiledPath();
          return;
      }

      ArrayList<Pose2d> poses = new ArrayList<>();
      for (PathPlannerPath path : auto) {
          path = ChassisConstants.shouldFlipPath() ? path : path.flipPath();
          poses.addAll(path.getPathPoses());
      }

      GeneralRobotState.getInstance().addPathToField(poses);
  }

  /**
   * @return A LoggedDashboardChooser for the auto commands and gives
   * PathPlanner sequences for our auto commands
   */
  public LoggedDashboardChooser<Command> registerNamedCommand() {
      
    // NamedCommands.registerCommand("deployIntake", intakeCoordinator.deployIntake(false));

    // NamedCommands.registerCommand("openPivotOffRoller", intakeCoordinator.openPivotOffRoller(false));

    // NamedCommands.registerCommand("ShootAndAim", Commands.parallel(
    //   shooterSubsystem.shootCommand(),
    //   intakeCoordinator.disableIntake(true),
    //   indexerSubsystem.turnOnIndexer()
    // ));

    // NamedCommands.registerCommand("WaitUntilNoFuel",Commands.waitUntil(() -> !generalRobotState.hasBalls()));
    
    // LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
    // autoChooser.onChange(this::displayChosenAuto);
    // autoChooser.addDefaultOption("idle", drivetrain.idle());

      return null;
  }
}