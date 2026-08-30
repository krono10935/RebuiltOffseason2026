package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOCTRE implements PivotIO {
    private final TalonFX motor;
    public PivotIOCTRE() {
        motor = new TalonFX(PivotConstants.MOTOR_CANID);

        motor.getConfigurator().apply(PivotConstants.getMotorConfig());
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        motor.setControl(new PositionVoltage(rotation.getRotations()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
        inputs.motorTempC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();
    }
    
}
