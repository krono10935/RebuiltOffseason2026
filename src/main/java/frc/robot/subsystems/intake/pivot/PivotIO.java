package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {

    @AutoLog
    public class PivotInputs{
        /**The temperature of the motor in celius */
        double motorTemperatureC;
        /**The angular velocity of the roller in RPS */
        double angularVelocityRPS;
        /**The current angle of the roller read from the encoder */
        Rotation2d angle;
    }

    /**
     * Command the hardware to go to a rotation
     * @param rotation The wanted rotation
     */
    void setRotation(Rotation2d rotation);

    /**
     * Command the hardware to go to a rotation, using a trapezoid profile to slow the pivot. The goal is for the balls to not get stuck between the pivot and the shooter
     * @param rotation The wanted rotation
s     */
    void setRotationSlow(Rotation2d rotation);
    
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
