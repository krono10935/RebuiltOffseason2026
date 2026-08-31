package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;

public class FlywheelConstants {
    public static final int[] MOTOR_IDS = {0, 1, 2, 3}; // TODO:change to correct CANID
    
    private static final double FLYWHEEL_RADIUS = UnitConversions.inchesToMeters(4); // TODO: make sure this is correct

    private static final double FLYWHEEL_CIRCUMFERENCE =  FLYWHEEL_RADIUS * 2 * Math.PI;

    public static final boolean LEAD_INVERTED = false; // TODO: correct it

    public static final int MPS_TOLERANCE = 5; // TODO: tweak
    
    public static final double GEAR_RATIO = 6.9; // TODO: set to correct value
    
    private static final double MOMENT_OF_INERTIA = 0.001; // TODO: set to correct value

    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(MOTOR_IDS.length);
        
    /**
     * @param isInverted should the motor's output be inverted
     * @return the lead motor's configuration
     */
    public static TalonFXConfiguration getLeadConfig(boolean isInverted) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // general

        config.Feedback.SensorToMechanismRatio = FLYWHEEL_CIRCUMFERENCE;

        config.MotorOutput.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // spinup gains

        config.Slot0.kP = 1; // TODO: tweak
        config.Slot0.kV = 1; // TODO: tweak

        // hold gains

        config.Slot1.kP = 10; // TODO: tweak
        config.Slot1.kV = 1; // TODO: tweak
        
        // current limits

        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;
    }

    /**
     * @param isInverted should the motors' output be inverted
     * @return the follower motors' configuration
     */
    public static TalonFXConfiguration getFollowerConfig(boolean isInverted) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // general

        config.Feedback.SensorToMechanismRatio = FLYWHEEL_CIRCUMFERENCE;
        
        config.MotorOutput.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // current limits

        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        return config;
    }

    /**
     * @return get the plant for the sim
     */
    private static LinearSystem<N1,N1,N1> getPlant(){
        return LinearSystemId.createFlywheelSystem(
            GEAR_BOX,
            MOMENT_OF_INERTIA,
            GEAR_RATIO
        );
    }

    /**
     * @return gets the flywheel's simulation
     */
    public static FlywheelSim getSim(){
        return new FlywheelSim(getPlant(), GEAR_BOX);
    }
}
