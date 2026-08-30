package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.math.UnitConversions;

public class RollerConstants {
    public final static int MOTOR_ONE_CANID = 1;
    public final static int MOTOR_TWO_CANID = 2;

    public final static double ROLLER_RADIUS_METER = 0.0275;
    public final static double UNIT_CONVERSION = UnitConversions.RPMtoRPS(2 * ROLLER_RADIUS_METER * Math.PI); // RPM to MPS



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

}
