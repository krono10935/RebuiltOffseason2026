package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logging.MotorLogger;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelInputs;

public class PivotIOCTRE implements PivotIO {
    private final DutyCycleEncoder dutyCycleEncoder;
    private final TalonFX motor;
    private boolean isActive = false;

    private final ProfiledPIDController controller = 
        new ProfiledPIDController(8,3,0.5, new Constraints(5, 5));

    public PivotIOCTRE() {
        motor = new TalonFX(PivotConstants.MOTOR_CANID);
        dutyCycleEncoder = new DutyCycleEncoder(PivotConstants.DUTY_CYCLE_ENCODER_PORT);
        motor.getConfigurator().apply(PivotConstants.getMotorConfig());

        resetMotorEncoder();
    }

    @Override
    public void setRotationSlow(Rotation2d rotation) {
        controller.setGoal(rotation.getRotations());
        isActive = true;
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        controller.setGoal(rotation.getRotations());
        isActive = true;
    }

    @Override
    public void stop() {
        motor.stopMotor();
        isActive = false;
    }   

    /**
     * Reset the encoder of the motor, based on the readings of an absoluteEncoder.
     */
    public void resetMotorEncoder() {
        motor.setPosition(getAbsoluteEncoderPos());
    }

    private double getAbsoluteEncoderPos(){
        return dutyCycleEncoder.get() - PivotConstants.ABS_ENCODER_OFFSET.getRotations();
    }
        
    private void logSubsystem(PivotInputs inputs) {
        // Subsystem logs
        Logger.recordOutput(PivotConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Speed (RPS)", inputs.angularVelocityRPS);
        Logger.recordOutput(PivotConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Angle", inputs.angle);
        Logger.recordOutput(PivotConstants.LOGGING_DIRECTORY_SUBSYSTEM + "Motor Temperature (Celisus)", inputs.motorTemperatureC);

        // Motor logs
        MotorLogger.log(PivotConstants.LOGGING_DIRECTORY_MOTORS + "PivotMotor", motor);
    }
    
    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(getAbsoluteEncoderPos());
        inputs.motorTemperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();

        if (isActive){
            double volts = controller.calculate(getAbsoluteEncoderPos());
            Logger.recordOutput("Pivot/PIDController/TotalOutput", volts);
            motor.setVoltage(volts);
        }

        Logger.recordOutput("Pivot/PIDController/Error", controller.getPositionError());
        Logger.recordOutput("Pivot/PIDController/Goal", controller.getGoal());
        Logger.recordOutput("Pivot/PIDController/Measurment", getAbsoluteEncoderPos());

        logSubsystem(inputs);
    }


}