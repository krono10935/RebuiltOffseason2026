package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.IsNear;
import frc.lib.statemachine.StateMachine.StateName;

public class PivotSubsystem extends SubsystemBase {
    private final PivotInputsAutoLogged inputs;
    private final PivotIO io;
    private final DutyCycleEncoder dutyCycleEncoder;

    /**An enum of the two possible pivot target states: open and close */
    public enum PivotState{
        /**The pivot is open and can intake/outtake*/
        OPEN(PivotConstants.OPEN_PIVOT_STATE_NAME),
        /**The pivot is closed */
        CLOSE(PivotConstants.CLOSE_PIVOT_STATE_NAME);

        /**The name of the state */
        private StateName stateName;
        
        /**
         * Create a new PivotState
         * @param targetAngle The wanted angle from the pivot
         * @param stateName The stateName of the state
         */
        private PivotState(StateName stateName){
            this.stateName = stateName;
        }

        /**
         * @return The StateName of the pivot
         */
        public StateName getStateName(){
            return stateName;
        }

        /**
         * @return The StateName of the pivot as a string
         */
        public String getName(){
            return stateName.toString();
        }
    }

    //Create a new PivotSubsystem
    public PivotSubsystem(){
        inputs = new PivotInputsAutoLogged();
        io = RobotBase.isReal() ? new PivotIOCTRE() : new PivotIOSim(); 
        dutyCycleEncoder = new DutyCycleEncoder(PivotConstants.DUTY_CYCLE_ENCODER_PORT);
    }

    @Override
    public void periodic(){
        io.resetMotorEncoder(dutyCycleEncoder);
        updateInputs();
        Logger.processInputs(getName(), inputs);
        
        if (getCurrentCommand() == null){
            Logger.recordOutput("Pivot command", "null!");
        }else {
            Logger.recordOutput("Pivot command", getCurrentCommand().getName());
        }
        Logger.recordOutput("Arm angle", inputs.angle.getDegrees());
        

    }

    /**
     * Get the motor temprature
     * @return The temprature of the motor in celsius
     */
    public double getMotorTempC(){
        return inputs.motorTemperatureC;
    }
    /**
     * Get the pivot angle
     * @return The angle of the pivot
     */
    public Rotation2d getAngle(){
        return inputs.angle;
    }

    /**
     * Get the angular velocity of the roller
     * @return The angular velocity of the pivot in RPS
     */
    public double getAngularVelocityRPS(){
        return inputs.angularVelocityRPS;
    }

    /**
     * Command the hardware to go to a rotation
     * @param rotation the wanted rotation
     */

    public void setRotation(Rotation2d rotation){
        io.setRotation(rotation);
    }

    /**
     * Command the hardware to go to a rotation, using a trapezoid profile to slow the pivot. The goal is for the balls to not get stuck between the pivot and the shooter
     * @param rotation The wanted rotation
s    */
    public void setRotationSlow(Rotation2d rotation){
        io.setRotationSlow(rotation);
    }

    /**
     * Stops the pivot
     */
    public void stop(){
        io.stop();
    }

    /**
     * Updates the field values of the input object.
     */
    private void updateInputs(){
        io.updateInputs(inputs);
    }

    /**
     * Checks if the pivot is open
     * @return whether the pivot is open
     */
    public boolean isPivotOpen() {
        return IsNear.isNear(getAngle(), PivotConstants.PIVOT_OPEN_ANGLE, PivotConstants.TOLERANCE);
    }
    /**
     * Checks if the pivot is close
     * @return Whether the pivot is closed
     */
    public boolean isPivotClose() {
        return IsNear.isNear(getAngle(), PivotConstants.PIVOT_CLOSE_ANGLE, PivotConstants.TOLERANCE);
    }

    /**
     * A command that opens the pivot
     * @return A command that opens the pivot
     */
    public Command openPivot(){
        return Commands.runEnd(
            () -> setRotation(PivotConstants.PIVOT_OPEN_ANGLE),
            this::stop,
            this
        );
    }

    /**
     * A command that opens the pivot slowly, using a trapezoid profile.
     * @return A command that opens the pivot slowly, using a trapezoid profile.
     */
    public Command openPivotSlow(){
        return Commands.runEnd(
            () -> setRotationSlow(PivotConstants.PIVOT_OPEN_ANGLE),
            this::stop,
            this
        );
    }

    /**
     * A command that closes the pivot
     * @return A command that closes the pivot
     */
    public Command closePivot(){
        return Commands.runEnd(
            () -> setRotation(PivotConstants.PIVOT_CLOSE_ANGLE),
            this::stop,
            this
        );
    }

    /**
     * A command that opens the pivot slowly, using a trapezoid profile.
     * @return A command that closes the pivot slowly, using a trapezoid profile.
     */
    public Command closePivotSlow(){
        return Commands.runEnd(
            () -> setRotationSlow(PivotConstants.PIVOT_CLOSE_ANGLE),
            this::stop,
            this
        );
    }
}