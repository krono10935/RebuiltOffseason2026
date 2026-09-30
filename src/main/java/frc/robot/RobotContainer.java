// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
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
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class RobotContainer {

  private static RobotContainer instance = null;

  public final Drivetrain drivetrain;

  private final LoggedDashboardChooser<Command> autoChooser;
  /**The pivot object we will be using */
  private final PivotSubsystem pivot=new PivotSubsystem();
  /**The roller object we will be using */
  private final RollerSubsystem roller=new RollerSubsystem();

  IntakeCoordinator intakeControlsCoordinator = new IntakeCoordinator(pivot, roller);

  CommandXboxController driverController = new CommandXboxController(0);

  

  public static RobotContainer getInstance(){
    if (instance == null){
      instance = new RobotContainer();
    }

    return instance;
  }

  private RobotContainer() {
    drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

    autoChooser = registerNamedCommand();
    controllerBindings();
  }

  public Drivetrain getDrivetrain(){
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
          drivetrain.clearFiledPath();
          return;
      }

      List<PathPlannerPath> auto;

      try {
          auto = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
      } catch (IOException | ParseException e) {
          Logger.recordOutput("autoDisplay", e.getMessage());
          drivetrain.clearFiledPath();
          return;
      }

      ArrayList<Pose2d> poses = new ArrayList<>();
      for (PathPlannerPath path : auto) {
          path = ChassisConstants.shouldFlipPath() ? path : path.flipPath();
          poses.addAll(path.getPathPoses());
      }

      drivetrain.addPathToField(poses);
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

  public void controllerBindings(){
    driverController.b().onTrue(intakeControlsCoordinator.disableIntake().withName("disableIntake"));

    driverController.leftBumper().whileTrue(intakeControlsCoordinator.deployIntakeReverse().withName("deployIntakeReverse"));

    driverController.leftTrigger(0.5).whileTrue(intakeControlsCoordinator.deployIntake().withName("deployIntake"));

    driverController.leftBumper().onFalse(intakeControlsCoordinator.openPivotOffRoller().withName("openPivotOffRoller"));

    driverController.leftTrigger(0.5).onFalse(intakeControlsCoordinator.openPivotOffRoller().withName("openPivotOffRoller"));

    driverController.a().whileTrue(roller.onRoller().withName("Open pivot"));
    driverController.x().whileTrue(roller.offRoller().withName("Close pivot"));
    driverController.x().whileTrue(roller.reverseRoller().withName("Reverse"));
  }
}