package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodConstants {
    // most beautiful TODOs ever
    public static final int HOOD_MOTOR_CANID = 0;                                               // TODO: change to correct CANID
    public static final Rotation2d DEGREE_TOLERANCE = Rotation2d.fromDegrees(0.25);    // TODO: tweak
    public static final double GEAR_RATIO = 6.7;                                              // TODO: change to correct
    public static final DCMotor GEAR_BOX = DCMotor.getNEO(1);                      // TODO: change to NEO 2 when available
    private static final double MOMENT_OF_INERTIA = 0.001;                                  // TODO: change to correct
    private static final double HOOD_LENGTH_METERS = 0.3;                                  // TODO: change to correct
    private static final Rotation2d HOOD_CLOSE_ANGLE = Rotation2d.fromDegrees(0); // TODO: change to correct
    private static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(45); // TODO: change to correct
    private static final boolean SIMULATE_GRAVITY = false;                              // TODO: change to wanted mode

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

    /**
     * @return get the plant for the sim
     */
    private static LinearSystem<N2, N1, N2> getPlant(){
        return LinearSystemId.createSingleJointedArmSystem(
            GEAR_BOX,
            MOMENT_OF_INERTIA,
            GEAR_RATIO
        );
    }

    /**
     * @return the hood's sim
     */
    public static SingleJointedArmSim getSim() {
        return new SingleJointedArmSim(
            getPlant(),
            GEAR_BOX,
            GEAR_RATIO, 
            HOOD_LENGTH_METERS, 
            HOOD_CLOSE_ANGLE.getRadians(), 
            HOOD_MAX_ANGLE.getRadians(), 
            SIMULATE_GRAVITY, 
            HOOD_CLOSE_ANGLE.getRadians()
        );
    }
}
