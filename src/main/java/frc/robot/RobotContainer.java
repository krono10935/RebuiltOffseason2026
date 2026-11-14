// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

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

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class RobotContainer {

  private static RobotContainer instance = null;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverXboxController;
  // private final CommandXboxController operatorXboxController;
  // private final RollerSubsystem rollerSubsystem;
  // private final PivotSubsystem pivotSubsystem;
  // private final IntakeCoordinator intakeCoordinator;

  // private final FlywheelSubsystem flywheelSubsystem;
  // private final HoodSubsystem hoodSubsystem;
  // private final ShooterSubsystem shooterSubsystem;

  // private final IndexerSubsystem indexerSubsystem;

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
    // pivotSubsystem = new PivotSubsystem();

    // flywheelSubsystem = new FlywheelSubsystem();
    // hoodSubsystem = new HoodSubsystem();
    // shooterSubsystem = new ShooterSubsystem(flywheelSubsystem, hoodSubsystem);

    // indexerSubsystem = new IndexerSubsystem();

    // intakeCoordinator = new IntakeCoordinator(pivotSubsystem, rollerSubsystem);

    autoChooser = registerNamedCommand();

    driverXboxController = new CommandXboxController(0);

    drivetrain = new DrivetrainReal(
      ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants
    );

    CommandScheduler.getInstance().schedule(drivetrain.driveCommand().ignoringDisable(true));

    // sysId = new DrivetrainSysID(Constants.CHASSIS_TYPE.COMPBOT.constants);

    // driverXboxController.a().whileTrue(sysId.sysIdDynamicSteer(Direction.kForward));
    // driverXboxController.b().whileTrue(sysId.sysIdDynamicSteer(Direction.kReverse));
    // driverXboxController.y().whileTrue(sysId.sysIdQuasistaticSteer(Direction.kForward));
    // driverXboxController.x().whileTrue(sysId.sysIdQuasistaticSteer(Direction.kReverse));



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

  // public IndexerSubsystem getIndexer(){
  //     return indexerSubsystem;
  // }

  // public IntakeCoordinator getIntake(){
  //     return intakeCoordinator;
  // }

  // public DrivetrainReal getDrivetrain(){
  //   return drivetrain;
  // }

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