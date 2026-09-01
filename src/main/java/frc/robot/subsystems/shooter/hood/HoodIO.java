package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {

    @AutoLog
    public class HoodInputs {
        boolean isAtGoal; // is the hood at angle setpoint
        Rotation2d currentAngle; // hood angle
    }

    /**
     * sets the wanted hood angle
     * @param angle the wanted angle
     */
    void setAngle(Rotation2d angle);

    /**
     * holds the wanted hood angle
     * @param angle the wanted angle
     */
    void holdAngle(Rotation2d angle);

    /**
     * stops the hood
     */
    void stop();

    /**
     * updates the inputs using data from the IO
     * @param inputs the reference to the input object to update
     */
    void updateInputs(HoodInputs inputs);
}
