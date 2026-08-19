// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.dashboard.Elastic;
import frc.lib.subsystem.VirtualSubSystem;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private Command m_teleopSuperStructre;

  public Robot() {

    initializeLogging();

    new Trigger(()-> DriverStation.isDSAttached()).onTrue(new InstantCommand(() -> Elastic.selectTab("Autonomous")));

    RobotContainer.getInstance();
  }

  private void initializeLogging(){
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    StatusLogger.disableAutoLogging();
    StatusLogger.stop();

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
        case 0 -> "All changes committed";
        case 1 -> "Uncommitted changes";
        default -> "Unknown";
        });

     if(!Constants.IS_COMP){
      if (isReal()) {
          Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
          Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
          Logger.addDataReceiver(new NT4Publisher());
      }
     } else {
      switch (Constants.currentMode) {
        case REAL:
            if (!Constants.isPit){
              File disk = new File("/U");

              if(disk.exists()){
                  Logger.addDataReceiver(new WPILOGWriter());
              }
              else{
                  Logger.addDataReceiver(new WPILOGWriter("/home/lvuser"));
              }
              
          } else {
              Logger.addDataReceiver(new NT4Publisher());
          }
          break;

        case SIM:
          // Running a physics simulator, log to NT
          Logger.addDataReceiver(new NT4Publisher());
          break;

        case REPLAY:
          // Replaying a log, set up replay source
          setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
          break;
      }
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    VirtualSubSystem.virtualperiodic();
    CommandScheduler.getInstance().run();

    var conduit = ConduitApi.getInstance();
    double voltage = conduit.getPDPVoltage();
    for (int i = 0; i < conduit.getPDPChannelCount(); i++){
        Logger.recordOutput("PDH/ChannelPower/" + i, conduit.getPDPChannelCurrent(i) * voltage);
    }
    Logger.recordOutput("PDH/Total Power", conduit.getPDPTotalCurrent() * voltage);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    // RobotContainer.getInstance().drivetrain.setBrakeMode(true);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_teleopSuperStructre = SuperStructure.getInstance().getCommand().repeatedly();

    if (m_teleopSuperStructre != null) {
      CommandScheduler.getInstance().schedule(m_teleopSuperStructre);
    }

    // RobotContainer.getInstance().drivetrain.reset(RobotContainer.getInstance().drivetrain.getEstimatedPosition());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // RobotContainer.getInstance().drivetrain.setBrakeMode(false);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
