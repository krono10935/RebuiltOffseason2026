package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;

public class RollerConstants {
    public final static int MOTOR_ONE_CANID = 1;
    public final static int MOTOR_TWO_CANID = 2;

    public final static double ROLLER_RADIUS_METER = 0.0275;
    public final static double UNIT_CONVERSION = UnitConversions.RPMtoRPS(2 * ROLLER_RADIUS_METER * Math.PI); // RPM to MPS

    private static final double MOMENT_OF_INERTIA = 0.001;
    public static final double GEAR_RATIO = 1;
    public static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(2);

    public static SparkMaxConfig getLeadConfig(){
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.encoder.positionConversionFactor(UNIT_CONVERSION);  
        motorConfig.smartCurrentLimit(80,30);

        return motorConfig;
    }

    public static SparkMaxConfig getFollowerConfig(){
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        
        motorConfig.encoder.positionConversionFactor(UNIT_CONVERSION);

        motorConfig.follow(MOTOR_ONE_CANID, true);
        motorConfig.smartCurrentLimit(80,30);

        return motorConfig;
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
