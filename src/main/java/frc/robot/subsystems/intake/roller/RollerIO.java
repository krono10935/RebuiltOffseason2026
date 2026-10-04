package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    public class RollerInputs{
        /**The temperature of motor one in celius */
        double motorOneTemperatureC;
        /**The temperature of motor two in celius */
        double motorTwoTemperatureC;
        /**speed of the motor meteres per second (skebob) */
        double speedMPS;
    }

    /**
     * Set the amount of effort the roller has to apply.
     * @param dutyCycle The dutyCycle to apply.
     */
    void setDutyCycle(double dutyCycle);

    /**
     * Stops the roller.
     */
    void stop();

    /**
     * Updates the inputs.
     * @param inputs The inputs of the roller.
     */
    void updateInputs(RollerInputs inputs);
}
