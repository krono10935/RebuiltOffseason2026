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
import frc.lib.statemachine.StateMachine.StateName;

public class HoodConstants {
    /** CANID of the hood's motor */
    public static final int HOOD_MOTOR_CANID = 0; // TODO: change to correct CANID

    /** how much we are willing to tolerate differences between set PID angle the and actual angle */
    public static final Rotation2d DEGREE_TOLERANCE = Rotation2d.fromDegrees(0.25); // TODO: tweak

    /** gear ratio between hood motor and the hood */
    public static final double GEAR_RATIO = 20.0/3.0;

    /** gear box of the 1 NEO2 motor (currently unavailable but close enough to .getNEO()) */
    public static final DCMotor GEAR_BOX = DCMotor.getNEO(1); // TODO: change to NEO 2 when available

    /** moment of inertia of the hood */
    private static final double MOMENT_OF_INERTIA = 0.00324; // TODO: get the real value from cad

    /** length of the hood in meters */
    private static final double HOOD_LENGTH_METERS = 0.21; // TODO: make sure it's real value

    /** the angle where is hood is closed (0) */
    public static final Rotation2d HOOD_CLOSE_ANGLE = Rotation2d.fromDegrees(0);

    /** the max angle the hood can reach */
    public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(43);

    /** whether the simulation should apply gravity forces */
    private static final boolean SIMULATE_GRAVITY = true;

    /** the state name of the stop hood state machine command */
    public static final StateName STOP_HOOD_STATE_NAME = new StateName("stop hood");

    /** the state name of the disable hood state machine command */
    public static final StateName ZERO_HOOD_STATE_NAME = new StateName("close hood");

    /**
     * @return the hood motor's configuration
     */
    public static SparkMaxConfig getHoodConfig(){
        SparkMaxConfig config = new SparkMaxConfig();

        // general

        config.idleMode(IdleMode.kBrake);

        // unit conversion
        config.encoder.positionConversionFactor(1.0 / GEAR_RATIO);
        config.encoder.velocityConversionFactor(1.0 / GEAR_RATIO);

        // configure PIDs
        config.closedLoop.pid(2, 1, 0); // TODO: tweak
        config.closedLoop.pid(2, 1, 0.05, ClosedLoopSlot.kSlot1);

        config.closedLoop.iZone(0.05);
        config.closedLoop.iZone(0.05, ClosedLoopSlot.kSlot1);

        // configure feedForwards

        config.closedLoop.feedForward.kCos(0.005); // TODO: tweak
        config.closedLoop.feedForward.kCos(0.005, ClosedLoopSlot.kSlot1); // TODO: tweak

        // current limits

        config.smartCurrentLimit(80, 20);

        return config;
    }

    /**
     * @return the plant for the sim
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
