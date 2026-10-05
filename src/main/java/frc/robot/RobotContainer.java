// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DrivetrainReal;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

public class RobotContainer {

  private static RobotContainer instance = null;

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final IntakeCoordinator intake;
  private final DrivetrainReal drivetrain;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverXboxController;
  private final CommandXboxController operatorXboxController;

  public static RobotContainer getInstance(){
    if (instance == null){
      instance = new RobotContainer();
    }

    return instance;
  }

  private RobotContainer() {
    drivetrain = new DrivetrainReal(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

    shooter =  new ShooterSubsystem(new FlywheelSubsystem(),new HoodSubsystem());
    indexer = new IndexerSubsystem();
    intake = new IntakeCoordinator(new PivotSubsystem(), new RollerSubsystem());

    autoChooser = registerNamedCommand();

    driverXboxController = new CommandXboxController(0);

    operatorXboxController = new CommandXboxController(1);

  }

  public CommandXboxController getDriverXboxController() {
      return driverXboxController;
  }

  public CommandXboxController getOperatorXboxController() {
      return operatorXboxController;
  }

  public ShooterSubsystem getShooter(){
      return shooter;
  }

  public IndexerSubsystem getIndexer(){
      return indexer;
  }

  public IntakeCoordinator getIntake(){
      return intake;
  }

  public DrivetrainReal getDrivetrain(){
    return drivetrain;
  }

  /**
   * @return the chosen autonomous command.
   */
  public Command getAutonomousCommand() {
      var selectedAuto = autoChooser.get();

      Command autoCommand =
              selectedAuto
                      .andThen(drivetrain.idle());

      CommandScheduler.getInstance().removeComposedCommand(selectedAuto);

      return autoCommand.withName(selectedAuto.getName());
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
      LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
      autoChooser.onChange(this::displayChosenAuto);
      autoChooser.addDefaultOption("idle", drivetrain.idle());
      return autoChooser;
  }
}