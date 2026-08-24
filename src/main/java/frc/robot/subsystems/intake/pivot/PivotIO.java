package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {

    @AutoLog
    public class PivotInputs{
        double motorTempC;
        double angularVelocityRPS;
        Rotation2d angle;
    }

    /**
     * Command the hardware to go to a rotation
     * @param rotation the wanted rotation
     */
    void setRotation(Rotation2d rotation);
    
    /**
     * Stops the pivot
     */
    void stop();
    
    /**
     * Updates the field values of the input object.
     * @param inputs The input object that we are updating.
     */
    void updateInputs(PivotInputs inputs);
}
