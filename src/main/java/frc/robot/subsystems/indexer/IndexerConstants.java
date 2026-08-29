package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerConstants {
    public static final int[] CAN_IDS = {0, 1, 2};
    public static final boolean[] MOTORS_INVERTED = {false, false, true};

    public static final double DUTY_CYCLE_FORWARD = 0.67;
    public static final double DUTY_CYCLE_BACKWARD = -0.67;

    private static final double MOMENT_OF_INERTIA = 0.001;
    public static final double GEAR_RATIO = 2;
    private static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(3);

    /**
     * get the config
     * @param isInverted is the motor going to spin clockwise or counter clockwise
     * @return the motor config
     */
    public static TalonFXConfiguration getConfig(boolean isInverted){
        
        TalonFXConfiguration config = new TalonFXConfiguration();

       config.MotorOutput.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; 
       config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

       config.CurrentLimits.StatorCurrentLimit = 120;
       config.CurrentLimits.SupplyCurrentLimit = 80;

       config.CurrentLimits.StatorCurrentLimitEnable = true;
       config.CurrentLimits.SupplyCurrentLimitEnable = true;

       return config;
    }

    private static LinearSystem<N1,N1,N1> getPlant(){
        return LinearSystemId.createFlywheelSystem(
            GEAR_BOX, 
            MOMENT_OF_INERTIA, 
            GEAR_RATIO
        );
    }

    public static FlywheelSim getSim(){
        return new FlywheelSim(getPlant(), GEAR_BOX);
    }
}
