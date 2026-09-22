package frc.robot.subsystems.intake.roller;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
    private final RollerInputsAutoLogged inputs;
    private final RollerIO io;

    //Create a new RollerSubsystem
    public RollerSubsystem(){
        inputs = new RollerInputsAutoLogged();
        io = RobotBase.isReal() ? new RollerIORev() : new RollerIOSim();
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
        return inputs.motorOneTemperatureC;
    }

    /**
     * Get the motor two's temprature
     * @return The temprature of the motor in celsius 
     */
    public double getMotorTwoTempC(){
        return inputs.motorTwoTemperatureC;
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

    /**
     * A command that turns off the roller
     * @return A command that turns off the roller
     */
    public Command offRoller(){
        return Commands.runOnce(
            this::stop,
            this
        );
    }

    /**
     * A command that reverses the roller
     * @return A command that reverses the roller
     */
    public Command reverseRoller(){
        return Commands.runOnce(
            () -> setDutyCycle(RollerConstants.REVERSED_DUTY_CYCLE),
            this
        );
    }

    /**
     * A command that turns on the roller
     * @return A command that turns on the roller
     */
    public Command onRoller(){
        return Commands.runOnce(
            () -> setDutyCycle(RollerConstants.ON_DUTY_CYCLE),
            this
        );
    }
}