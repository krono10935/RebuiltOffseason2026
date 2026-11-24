package frc.robot.subsystems.drivetrain.module;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logging.MotorLogger;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;
import frc.robot.subsystems.drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.module.constants.SwerveModulesMK4;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelInputs;

public abstract class SwerveModuleCTRE extends SwerveModuleIO {

    protected final TalonFX drivingMotor;
    protected final TalonFX steeringMotor;
    
    protected SwerveModuleCTRE(CTREModuleConstants constants){
        super(constants);
        drivingMotor = new TalonFX(constants.DRIVE_ID(), Constants.canivore);
        drivingMotor.getConfigurator().apply(constants.DRIVING_CONFIG());

        steeringMotor = new TalonFX(constants.STEER_ID(), Constants.canivore);
        steeringMotor.getConfigurator().apply(constants.STEERING_CONFIG());

    }

    @Override
    protected double getDriveVelocity() {
        if (drivingMotor.isConnected())
            return drivingMotor.getVelocity().getValueAsDouble() * constants.UNIT_CONVERSION();
        else
            return 0;
    }

    @Override
    protected double getDriveDistance() {
        return drivingMotor.getPosition().getValueAsDouble() * constants.UNIT_CONVERSION();
    }

    @Override
    protected double getSteerAngle() {
        return steeringMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setTargetState(SwerveModuleState targetState, int slot) {
        drivingMotor.setControl(new VelocityTorqueCurrentFOC(
            targetState.speedMetersPerSecond * 
            constants.DRIVE_GEAR_RATIO() / constants.UNIT_CONVERSION())
            .withSlot(slot));

        steeringMotor.setControl(new PositionTorqueCurrentFOC(
            targetState.angle.getRotations())
            .withSlot(slot));
    }


    @Override
    public void setSteerVoltage(double voltage){
        steeringMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDriveVoltageAndSteerAngle(double voltage, Rotation2d angle) {
        steeringMotor.setControl(new PositionTorqueCurrentFOC(angle.getRotations()));
        drivingMotor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        NeutralModeValue neutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        
        drivingMotor.setNeutralMode(neutralMode);
        steeringMotor.setNeutralMode(neutralMode);
    }

    private void logSubsystem() {
        // Motor logs
        MotorLogger.log(DrivetrainConstants.LOGGING_DIRECTORY_MOTORS + constants.NAME() + "/drive", drivingMotor);
        MotorLogger.log(DrivetrainConstants.LOGGING_DIRECTORY_MOTORS + constants.NAME() + "/steer", steeringMotor);
    }

    @Override
    public void update(){
        super.update();

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/position", 
            steeringMotor.getPosition().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/velocity", 
            steeringMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/voltageOut", 
            steeringMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/setpoint",
            steeringMotor.getClosedLoopReference().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/error",
            steeringMotor.getClosedLoopError().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/dOuput",
            steeringMotor.getClosedLoopDerivativeOutput().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/iOuput",
            steeringMotor.getClosedLoopIntegratedOutput().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/steer/pOuput",
            steeringMotor.getClosedLoopProportionalOutput().getValueAsDouble());


        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/drive/position", 
            drivingMotor.getPosition().getValueAsDouble() );

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/drive/velocity", 
            drivingMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/drive/voltageOut", 
            drivingMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/drive/isBrake", 
        drivingMotor.getControlMode().getValue() == ControlModeValue.StaticBrake);

        logSubsystem();
    }
}
