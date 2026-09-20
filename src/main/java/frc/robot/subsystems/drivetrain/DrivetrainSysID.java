// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.GeneralRobotState;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.drivetrain.constants.SysIDConstants;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;

public class DrivetrainSysID extends Drivetrain {

  /**
   * SysID routine for Drive
   */
  private SysIdRoutine routineDrive;

  /**
   * SysID routine for Steer
   */
  private SysIdRoutine routineSteer;

  /**
   * SysID routine for Spin
   */
  private SysIdRoutine routineSpin;

  public DrivetrainSysID(ChassisConstants constants){
    super(constants);


    routineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(SysIDConstants.VOLT_RAMP_RATE),
        Volts.of(SysIDConstants.VOLT),
        Second.of(SysIDConstants.TIMEOUT),
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),

      new SysIdRoutine.Mechanism(
        (volt) -> this.setSteerVoltage(volt.baseUnitMagnitude()),
        null,
        this
      )
    );

    routineDrive = new SysIdRoutine(
      new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(SysIDConstants.VOLT_RAMP_RATE),
        Volts.of(SysIDConstants.VOLT),
        Second.of(SysIDConstants.TIMEOUT),
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),

      new SysIdRoutine.Mechanism(
        (volt) -> this.setDriveVoltageAndSteerController(volt.baseUnitMagnitude()),
        null,
        this
      )
    );

    routineSpin = new SysIdRoutine(
      new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(SysIDConstants.VOLT_RAMP_RATE),
        Volts.of(SysIDConstants.VOLT),
        Second.of(SysIDConstants.TIMEOUT),
        (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volt) -> this.setDriveVoltageAndSpin(volt.baseUnitMagnitude()),
        null,
        this
      )
    );
  }
  
 /**
   * Function used by sysID to profile the behavior of the steer motor (find steer FFs)
   * @param voltage the voltage that the steer motor should apply
   */
  public void setSteerVoltage(double voltage){
      for (SwerveModuleIO module : io){
          module.setSteerVoltage(voltage);
      }
  }

  /**
   * Function used by sysID to profile the behavior of the module (find drive FFs)
   * @param voltage the voltage that the drive motor should apply
   * @apiNote (!) Use a controller to make sure the swerve
   *  doesn't ram into a wall or person.
   */
  public void setDriveVoltageAndSteerController(double voltage) {
    CommandXboxController controller = GeneralRobotState.getInstance().getController();
    
    Translation2d leftJoystickPosition = 
      new Translation2d(-controller.getLeftX(), -controller.getLeftY());

    Rotation2d goalAngle = leftJoystickPosition.getAngle();    

    for (int i = 0; i < 4; i++){
        io[i].setDriveVoltageAndSteerAngle(voltage, 
          goalAngle.minus(gyroInputs.pose.getRotation()));
    }
  }


  /**
   * Function used by sysID to profile the behavior of the robot (find the ROBOT MOI)
   * Automatically configurates the swerve modules to the spin angles
   * @param voltage the voltage that the drive motor should apply
   */
  public void setDriveVoltageAndSpin(double voltage){
    for (int i = 0; i < 4; i++){
        io[i].setDriveVoltageAndSteerAngle(
          voltage, 
          SysIDConstants.SYSID_SPIN_STEER_ANGLES[i]
      );
    }
  }

  /**
     * Factory for the Quasistatic Drive command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
      return routineDrive.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Drive command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
      return routineDrive.dynamic(direction);
   }

    /**
     * Factory for the Quasistatic Steer command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
      return routineSteer.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Steer command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
      return routineSteer.dynamic(direction);
   }

    /**
     * Factory for the Quasistatic Spin command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticSpin(SysIdRoutine.Direction direction) {
      return routineSpin.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Spin command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicSpin(SysIdRoutine.Direction direction) {
      return routineSpin.dynamic(direction);
   }
}
