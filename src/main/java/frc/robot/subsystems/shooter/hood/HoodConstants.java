package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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
    public static final int HOOD_MOTOR_CANID = 30; // TODO: change to correct CANID

    /** roborio port of the absolute encoder */
    public static final int ABSOLUTE_ENCODER_PORT = 0; // TODO: change to correct roborio port

    /** how much we are willing to tolerate differences between set PID angle the and actual angle */
    public static final Rotation2d DEGREE_TOLERANCE = Rotation2d.fromDegrees(0.75); // TODO: tweak

    /** gear ratio between absolute encoder and the hood */
    public static final double ABSOLUTE_ENCODER_TO_HOOD_RATIO = 3.0;

    /** gear ratio between motor and hood */
    public static final double MOTOR_TO_MECHANISM_RATIO = 22.5;

    /** unit conversion between the absolute encoder's position and the encoder's position (same as motor position) */
    public static final double ABSOLUTE_ENCODER_TO_ENCODER_POS = MOTOR_TO_MECHANISM_RATIO / ABSOLUTE_ENCODER_TO_HOOD_RATIO;

    /** gear box of the 1 NEO2 motor (currently unavailable but close enough to .getNEO()) */
    public static final DCMotor GEAR_BOX = DCMotor.getNEO(1); // TODO: change to NEO 2 when available

    /** moment of inertia of the hood */
    private static final double MOMENT_OF_INERTIA = 0.023; // TODO: get the real value from cad

    /** length of the hood in meters */
    private static final double HOOD_LENGTH_METERS = 0.21; // TODO: make sure it's real value

    /** the angle where is hood is closed (0) */
    public static final Rotation2d HOOD_CLOSE_ANGLE = Rotation2d.fromDegrees(0);

    /** the max angle the hood can reach */
    public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(43);

    /** whether the simulation should apply gravity forces */
    private static final boolean SIMULATE_GRAVITY = false; // TODO: change when necesarry

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
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // unit conversion
        config.absoluteEncoder.positionConversionFactor(1.0 / ABSOLUTE_ENCODER_TO_HOOD_RATIO);
        config.encoder        .positionConversionFactor(1.0 / MOTOR_TO_MECHANISM_RATIO);

        // configure PIDs
        config.closedLoop.pid(8.0, 0.0, 0.0); // TODO: tweak
        config.closedLoop.pid(8.0, 1.0, 0.0, ClosedLoopSlot.kSlot1);

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
            MOTOR_TO_MECHANISM_RATIO
        );
    }

    /**
     * @return the hood's sim
     */
    public static SingleJointedArmSim getSim() {
        return new SingleJointedArmSim(
            getPlant(),
            GEAR_BOX,
            MOTOR_TO_MECHANISM_RATIO, 
            HOOD_LENGTH_METERS, 
            HOOD_CLOSE_ANGLE.getRadians(), 
            HOOD_MAX_ANGLE.getRadians(), 
            SIMULATE_GRAVITY, 
            HOOD_CLOSE_ANGLE.getRadians()
        );
    }
}
