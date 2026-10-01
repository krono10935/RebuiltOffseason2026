package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;

public abstract class SwerveModuleCTRE extends SwerveModuleIO {

    protected final TalonFX drivingMotor;
    protected final TalonFX steeringMotor;
    
    protected SwerveModuleCTRE(CTREModuleConstants constants){
        super(constants);
        drivingMotor = new TalonFX(constants.DRIVE_ID());
        drivingMotor.getConfigurator().apply(constants.DRIVING_CONFIG());

        steeringMotor = new TalonFX(constants.STEER_ID());
        steeringMotor.getConfigurator().apply(constants.STEERING_CONFIG());

    }

    @Override
    public void update(){
        super.update();
    }

    @Override
    protected double getDriveVelocity() {
        if (drivingMotor.isConnected())
            return drivingMotor.getVelocity().getValueAsDouble() * constants.UNIT_CONVERSION();
        else
            return 0;
    }

    @Override
    protected double getDrivePos() {
        return drivingMotor.getPosition().getValueAsDouble() * constants.UNIT_CONVERSION();
    }

    @Override
    protected double getSteerAngle() {
        return steeringMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        drivingMotor.setControl(new VelocityTorqueCurrentFOC(targetState.speedMetersPerSecond / constants.UNIT_CONVERSION()));
        steeringMotor.setControl(new PositionTorqueCurrentFOC(targetState.angle.getRotations()));
    }

    
    //TODO: tune pid with balls
    @Override
    public void setTargetStateWithBalls(SwerveModuleState targetState){
        drivingMotor.setControl(
            new VelocityTorqueCurrentFOC(
                targetState.speedMetersPerSecond / constants.UNIT_CONVERSION())
            .withSlot(1)
        );


        steeringMotor.setControl(
            new PositionTorqueCurrentFOC(
                targetState.angle.getRotations())
            .withSlot(1)
        );
    }

    @Override
    public void setSteerVoltage(double voltage){
        steeringMotor.setVoltage(voltage);
    }

    @Override
    public void setDriveVoltageAndSteerAngle(double voltage, Rotation2d angle) {
        steeringMotor.setControl(new PositionTorqueCurrentFOC(angle.getRotations()));
        drivingMotor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        MotorOutputConfigs neutralModeConfig = new MotorOutputConfigs()
            .withNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        
        drivingMotor.getConfigurator().apply(neutralModeConfig);

        steeringMotor.getConfigurator().apply(neutralModeConfig);

    }
}
