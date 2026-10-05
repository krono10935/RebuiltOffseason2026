package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.DutyCycleOut;

public interface FlywheelIO {

    @AutoLog
    public class FlywheelInputs {
        boolean isAtGoal; // is the flywheel at speed setpoint
        double speedMPS; // flywheel speed in meters/second
    }

    /**
     * sets the wanted flywheel speed (fast but non accurate PID)
     * @param mps the wanted speed in meters/second
     */
    void spinUp(double mps);

    /**
     * sets the wanted flywheel dutycycle
     * @param dutyCycle the duty cycle to set to
     */
    void spinUp(DutyCycleOut dutyCycle);

    /**
     * holds the wanted flywheel speed (slow but accurate PID)
     * @param mps the wanted speed in meters/second
     */
    void holdSpeed(double mps);

    /**
     * holds the wanted flywheel dutycycle
     * @param dutyCycle the duty cycle to hold
     */
    void holdSpeed(DutyCycleOut dutyCycle);

    /**
     * stops the flywheel
     */
    void stop();

    /**
     * updates the inputs using data from the IO
     * @param inputs the reference to the input object to update
     */
    void updateInputs(FlywheelInputs inputs);
}
