package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.math.IsNear;

public class FlywheelIOReal implements FlywheelIO {

    private final TalonFX[] flywheelMotors;

    public FlywheelIOReal()
    {
        flywheelMotors = new TalonFX[4];

        flywheelMotors[0] = new TalonFX(FlywheelConstants.MOTOR_IDS[0]); // flywheelMotors[0] is the leading motor
        flywheelMotors[0].getConfigurator().apply(FlywheelConstants.getLeadConfig(FlywheelConstants.LEAD_INVERTED));

        for (int i = 1; i < flywheelMotors.length; i++){
            flywheelMotors[i] = new TalonFX(FlywheelConstants.MOTOR_IDS[i]);

            boolean inverted = i >= flywheelMotors.length / 2; // half the motors are inverted

            flywheelMotors[i].getConfigurator().apply(FlywheelConstants.getFollowerConfig(inverted));

            flywheelMotors[i].setControl(new Follower(FlywheelConstants.MOTOR_IDS[0], MotorAlignmentValue.Aligned)); // TODO: understand how aligned works, we set it to inverted above this so does it align with the original or the inverted mode
        }
    }

    /**
     * @return returns the lead motor
     */
    private TalonFX getLeadMotor()
    {
        return flywheelMotors[0];
    }

    @Override
    public void setSpeed(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps).withSlot(0));
    }

    @Override
    public void holdSpeed(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps).withSlot(1));
    }

    @Override
    public void stop() {
        getLeadMotor().stopMotor();
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        inputs.speedMPS = getLeadMotor().getVelocity().getValueAsDouble();

        inputs.isAtGoal = IsNear.isNear(inputs.speedMPS, getLeadMotor().getClosedLoopReference().getValueAsDouble(), FlywheelConstants.MPS_TOLERANCE);
    }

}
