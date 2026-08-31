package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotInputs;

public class PivotSubsystem extends SubsystemBase {
    private final PivotInputsAutoLogged inputs;
    private final PivotIO io;

    //Create a new PivotSubsystem
    public PivotSubsystem(){
        inputs = new PivotInputsAutoLogged();
        io = new PivotIOCTRE();
    }

    @Override
    public void periodic(){
        updateInputs();
        Logger.processInputs(getName(), inputs);
    }

    /**
     * Get the motor temprature
     * @return The temprature of the motor in celsius 
     */
    public double getMotorTempC(){
        return inputs.motorTempC;
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
     * Stops the pivot
     */
    public void stop(){
        io.stop();
    }

    /**
     * Updates the field values of the input object.
     * @param inputs The input object that we are updating.
     */
    private void updateInputs(){
        io.updateInputs(inputs);
    }
}