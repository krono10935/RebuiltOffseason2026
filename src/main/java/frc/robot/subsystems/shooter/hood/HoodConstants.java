package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {

    public static final int HOOD_MOTOR_CANID = 0; // TODO: change to correct CANID
    public static final Rotation2d DEGREE_TOLERANCE = Rotation2d.fromDegrees(0.25); // TODO: tweak

    /**
     * @return the hood motor's configuration
     */
    public static SparkMaxConfig getHoodConfig(){
        SparkMaxConfig config = new SparkMaxConfig();

        // general

        config.idleMode(IdleMode.kBrake);

        // configure PIDs

        config.closedLoop.pid(1, 1, 1); // TODO: tweak
        config.closedLoop.pid(1, 1, 1, ClosedLoopSlot.kSlot1);

        // configure feedForwards

        config.closedLoop.feedForward.kCos(0.67); // TODO: keep at 0.67 pwease
        config.closedLoop.feedForward.kCos(0.6942041, ClosedLoopSlot.kSlot1); // TODO: keep at 0.6942041 pwease

        // current limits

        config.smartCurrentLimit(80, 20);

        return config;
    }
}
