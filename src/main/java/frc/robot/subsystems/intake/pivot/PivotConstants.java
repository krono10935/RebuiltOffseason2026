package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class PivotConstants {
    public final static int MOTOR_CANID = 4;

    /**The motors that are spinning the pivot*/
    public final static DCMotor GEAR_BOX = DCMotor.getKrakenX60(1);
    private static final double MOMENT_OF_INERTIA = 0.001; // TODO: get value from CAD
    private static final double PIVOT_LENGTH_METERS = 0.3; // TODO: get value from CAD
    
    /** The angle where the pivot is closed */
    public static final Rotation2d PIVOT_CLOSE_ANGLE = Rotation2d.fromDegrees(67); // TODO: get value from CAD
    /** The angle where the pivot is opened */
    public static final Rotation2d PIVOT_OPEN_ANGLE = Rotation2d.fromDegrees(0);// TODO: get value from CAD
    /**whether we are accounting for gravity in the simulation */
    private static final boolean SIMULATE_GRAVITY = true;                              // TODO: change to wanted mode
    /** The gear ratio between the motor and the pivot arm (a single roation of the arm is equal to GEAR_RATIO roations of the motor).*/
    public static final double GEAR_RATIO = 45;  // TODO: get value from CAD
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5);

    /**
     * @return The plant for the sim
     */
    private static LinearSystem<N2, N1, N2> getPlant(){
        return LinearSystemId.createSingleJointedArmSystem(
                GEAR_BOX,
                MOMENT_OF_INERTIA,
                GEAR_RATIO
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
                GEAR_RATIO,
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
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;
        config.Slot0.kI = 0;
        config.Slot0.kG = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        //TODO: Tweak the PID values for setRotationSlow()
        config.Slot1.kP = 0;
        config.Slot1.kG = 0;
        config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        //The trapezoid profile for setRotationSlow()
        config.MotionMagic.MotionMagicAcceleration = 1;
        config.MotionMagic.MotionMagicCruiseVelocity = 1;
        
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;        
    }
}