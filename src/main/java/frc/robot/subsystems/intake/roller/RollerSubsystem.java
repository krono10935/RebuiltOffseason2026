package frc.robot.subsystems.intake.roller;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine.StateName;

public class RollerSubsystem extends SubsystemBase {
    private final RollerInputsAutoLogged inputs;
    private final RollerIO io;

    /**An enum of the three possible roller states*/
    public enum RollerState{
        /**The roller is on and can intake fuel into the hopper */
        ON(RollerConstants.ON_DUTY_CYCLE, RollerConstants.ON_ROLLER_STATE_NAME),
        /**The roller is reversed and can outtake fuel from the hopper */
        REVERSED(RollerConstants.REVERSED_DUTY_CYCLE, RollerConstants.REVERSE_ROLLER_STATE_NAME),
        /**The roller is off */
        OFF(RollerConstants.OFF_DUTY_CYCLE, RollerConstants.OFF_ROLLER_STATE_NAME);

        /**Name of the state */
        private StateName stateName;
        /**The wanted duty cycle from the roller */
        private double dutyCycle;

        /**
         * Create a new RollerState
         * @param dutyCycle The duty cycle of the state
         * @param stateName The stateName of the state
         */
        private RollerState(double dutyCycle, StateName stateName){
            this.dutyCycle = dutyCycle;
            this.stateName = stateName;
        }

        /**
         * @return The duty cycle of the enum
         */
        public double getDutyCycle(){
            return dutyCycle;
        }


        /**
         * @return The stateName as a string.
         */
        public String getName(){
            return stateName.toString();
        }

        /**
         * @return The stateName.
         */
        public StateName getStateName(){
            return stateName;
        }
    }

    //Create a new RollerSubsystem
    public RollerSubsystem(){
        inputs = new RollerInputsAutoLogged();
        io = RobotBase.isReal() ? new RollerIORev() : new RollerIOSim();
    }

    @Override
    public void periodic(){
        updateInputs();
        
        Logger.processInputs(getName(), inputs);

        if (getCurrentCommand() == null){
            Logger.recordOutput("Roller command", "null!");
        }else {
            Logger.recordOutput("Roller command", getCurrentCommand().getName());
        }

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
        return Commands.run(
            this::stop,
            this
        );
    }

    /**
     * A command that reverses the roller
     * @return A command that reverses the roller
     */
    public Command reverseRoller(){
        return Commands.run(
            () -> setDutyCycle(RollerConstants.REVERSED_DUTY_CYCLE),
            this
        );
    }

    /**
     * A command that turns on the roller
     * @return A command that turns on the roller
     */
    public Command onRoller(){
        return Commands.run(
            () -> setDutyCycle(RollerConstants.ON_DUTY_CYCLE),
            this
        );
    }
}