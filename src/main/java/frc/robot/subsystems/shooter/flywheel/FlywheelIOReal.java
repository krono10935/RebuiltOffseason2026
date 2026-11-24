package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.MotorLogger;
import frc.lib.math.IsNear;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodInputs;

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
            flywheelMotors[i].getConfigurator().apply(FlywheelConstants.getLeadConfig(inverted));

            // flywheelMotors[i].setControl(new Follower(FlywheelConstants.MOTOR_IDS[0], MotorAlignmentValue.Opposed)); // TODO: understand how aligned works, we set it to inverted above this so does it align with the original or the inverted mode
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
    public void spinUp(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps / FlywheelConstants.FLYWHEEL_CIRCUMFERENCE).withSlot(0));
    }

    @Override
    public void holdSpeed(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps / FlywheelConstants.FLYWHEEL_CIRCUMFERENCE).withSlot(1));
    }

    @Override
    public void setVoltage(double voltage) {
        getLeadMotor().setVoltage(voltage);
        flywheelMotors[1].setVoltage(voltage);
        flywheelMotors[2].setVoltage(-voltage);
        flywheelMotors[3].setVoltage(-voltage);
    }

    @Override
    public void stop() {
        getLeadMotor().stopMotor();
    }
    
    private void logSubsystem(FlywheelInputs inputs) {
        // Subsystem logs
        Logger.recordOutput(FlywheelConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Position", Rotation2d.fromRotations(inputs.positionRot));
        Logger.recordOutput(FlywheelConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Is At Goal", inputs.isAtGoal);
        Logger.recordOutput(FlywheelConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Speed (MPS)", inputs.speedMPS);

        // Motor logs
        for (int i = 0; i < flywheelMotors.length; i++)
            MotorLogger.log(FlywheelConstants.LOGGING_DIRECTORY_MOTORS + "FlywheelMotor" + i, flywheelMotors[i]);
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {

        inputs.speedMPS = getLeadMotor().getVelocity().getValueAsDouble() * FlywheelConstants.FLYWHEEL_CIRCUMFERENCE;

        inputs.speedRPS = getLeadMotor().getVelocity().getValueAsDouble();

        inputs.isAtGoal = IsNear.isNear(inputs.speedMPS, getLeadMotor().getClosedLoopReference().getValueAsDouble() * FlywheelConstants.FLYWHEEL_CIRCUMFERENCE, FlywheelConstants.MPS_TOLERANCE);

        inputs.positionRot = getLeadMotor().getPosition().getValueAsDouble();

        inputs.voltage = getLeadMotor().getMotorVoltage().getValueAsDouble();

        logSubsystem(inputs);
    }

    @Override
    public void basicSpinUp(double dutyCycle) {
        getLeadMotor().setControl(new DutyCycleOut(dutyCycle));
    }
}
