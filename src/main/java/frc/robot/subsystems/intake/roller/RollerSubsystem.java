package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.roller.RollerIO.RollerInputs;

public class RollerSubsystem extends SubsystemBase {
    private final RollerInputsAutoLogged inputs;
    private final RollerIO io;

    //Create a new PivotSubsystem
    public RollerSubsystem(){
        inputs = new RollerInputsAutoLogged();
        io = new RollerIORev();
    }

    @Override
    public void periodic(){
        updateInputs();
        Logger.processInputs(getName(), inputs);
    }

    /**
     * Get the motor one's temprature
     * @return The temprature of the motor in celsius 
     */
    public double getMotorOneTempC(){
        return inputs.motorOneTempC;
    }

    /**
     * Get the motor two's temprature
     * @return The temprature of the motor in celsius 
     */
    public double getMotorTwoTempC(){
        return inputs.motorTwoTempC;
    }

    /**
     * Get the motor's speed in Meters per second
     * @return The motor's speed in Meters per second
     */
    public double getMotorSpeedMPS(){
        return inputs.speedMPS;
    }

    /**
     * sets the Duty cycle 
     * @param dutyCycle effort of the motors 
     */
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    /**
     * Stops the motor 
     */
    public void stop() {
        io.stop();
    }
    /**
     * updates the inputs object
     */
    public void updateInputs() {
        io.updateInputs(inputs);
    }
}