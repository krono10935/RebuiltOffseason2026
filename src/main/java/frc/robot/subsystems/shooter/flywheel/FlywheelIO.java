package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public class FlywheelInputs {
        double speedMPS; // flywheel speed in meters/second
        boolean isAtGoal; // is the flywheel at speed setpoint
    }

    /**
     * sets the wanted flywheel speed
     * @param mps the wanted speed in meters/second
     */
    void setSpeed(double mps);

    /**
     * holds the wanted flywheel speed
     * @param mps the wanted speed in meters/second
     */
    void holdSpeed(double mps);

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
