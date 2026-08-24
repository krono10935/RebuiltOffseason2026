package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    public class RollerInputs{
        double motorOneTempC;
        double motorTwoTempC;
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
