package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOCTRE implements PivotIO {
    final DutyCycleEncoder dutyCycleEncoder;
    private final TalonFX motor;
    public PivotIOCTRE() {
        motor = new TalonFX(PivotConstants.MOTOR_CANID);
        dutyCycleEncoder = new DutyCycleEncoder(PivotConstants.DUTY_CYCLE_ENCODER_PORT);
        motor.getConfigurator().apply(PivotConstants.getMotorConfig());

        resetMotorEncoder();
    }

    @Override
    public void setRotationSlow(Rotation2d rotation) {
        motor.setControl(new MotionMagicVoltage(rotation.getRotations()).withSlot(1));
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        motor.setControl(new PositionVoltage(rotation.getRotations()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }   

    /**
     * Reset the encoder of the motor, based on the readings of an absoluteEncoder.
     */
    public void resetMotorEncoder() {
        motor.setPosition(dutyCycleEncoder.get() * PivotConstants.MOTOR_TO_ABSOLUTE_ENCODER_RATIO);
    }
        
    
    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
        inputs.motorTemperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();
    }


}