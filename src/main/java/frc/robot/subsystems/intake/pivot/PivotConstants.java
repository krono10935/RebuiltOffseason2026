package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.statemachine.StateMachine.StateName;


public class PivotConstants {
    public final static int MOTOR_CANID = 23; //TODO: get the real value
    /**The port of the duty cycle encoder */
    public final static int DUTY_CYCLE_ENCODER_PORT = 1;

    /**The motors that are spinning the pivot*/
    public final static DCMotor GEAR_BOX = DCMotor.getKrakenX60(1);
    private static final double MOMENT_OF_INERTIA = 0.088848272;
    private static final double PIVOT_LENGTH_METERS = 0.366;
    
    /** The angle where the pivot is closed */
    public static final Rotation2d PIVOT_CLOSE_ANGLE = Rotation2d.fromDegrees(69); // TODO: get real value from encoder measurements
    /** The angle where the pivot is opened */
    public static final Rotation2d PIVOT_OPEN_ANGLE = Rotation2d.fromDegrees(180);// TODO: get value from encoder measurements
    /**whether we are accounting for gravity in the simulation */
    private static final boolean SIMULATE_GRAVITY = false;                              // TODO: change to wanted mode
    /** The gear ratio between the motor and the pivot arm (a single roation of the pivot is equal to GEAR_RATIO roations of the motor).*/
    public static final double MOTOR_TO_ARM_RATIO = 50;
    /**The gear ratio between the absolute encoder and the pivot arm (a single roation of the pivot is equal to GEAR_RATIO roations of the encoder).*/
    public static final double ABOSLUTE_ENCODER_TO_ARM_RATIO = 1; //TODO: find the real value from CAD
    /**The gear ratio between the motor and the absolute encoder (a single roation of the motor is equal to GEAR_RATIO roations of the encoder).*/
    public static final double MOTOR_TO_ABSOLUTE_ENCODER_RATIO = MOTOR_TO_ARM_RATIO / ABOSLUTE_ENCODER_TO_ARM_RATIO;

    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);

    /**The state name of opening the pivot for state machines */
    public static final StateName OPEN_PIVOT_STATE_NAME = new StateName("OpenPivotState");
    /**The state name of closing the pivot for state machine */
    public static final StateName CLOSE_PIVOT_STATE_NAME = new StateName("ClosePivotState");

    /**
     * @return The plant for the sim
     */
    private static LinearSystem<N2, N1, N2> getPlant(){
        return LinearSystemId.createSingleJointedArmSystem(
                GEAR_BOX,
                MOMENT_OF_INERTIA,
                MOTOR_TO_ARM_RATIO
        );
    }

    /**
     * The pivot arm's simulation
     * @return the pivot's sim
     */
    public static SingleJointedArmSim getSim() {
        return new SingleJointedArmSim(
                getPlant(),
                GEAR_BOX,
                MOTOR_TO_ARM_RATIO,
                PIVOT_LENGTH_METERS,
                PIVOT_CLOSE_ANGLE.getRadians(),
                PIVOT_OPEN_ANGLE.getRadians(),
                SIMULATE_GRAVITY,
                PIVOT_CLOSE_ANGLE.getRadians()
        );
    }

    /**
     * The config we will be using for the TalonFX motor
     * @return The config
     */
    public static TalonFXConfiguration getMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        //TODO: Tweak the PID values
        config.Slot0.kP = 5;
        config.Slot0.kD = 0;
        config.Slot0.kI = 4.0;
        config.Slot0.kG = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        //TODO: Tweak the PID values for setRotationSlow()
        config.Slot1.kP = 5;
        config.Slot1.kG = 0;
        config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        //The trapezoid profile for setRotationSlow()
        config.MotionMagic.MotionMagicAcceleration = 100;
        config.MotionMagic.MotionMagicCruiseVelocity = 100;
        
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = MOTOR_TO_ARM_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PIVOT_OPEN_ANGLE.minus(Rotation2d.fromDegrees(3)).getRotations();
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVOT_CLOSE_ANGLE.plus(Rotation2d.fromDegrees(3)).getRotations();

        return config;
    }
}