package frc.robot.subsystems.intake.roller;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;
import frc.lib.statemachine.StateMachine.StateName;

public class RollerConstants {
    public final static int MOTOR_ONE_CANID = 1; //TODO: find the correct can id
    public final static int MOTOR_TWO_CANID = 2; //TODO: find the correct can id

    /**The roller's radius in meters */
    public final static double ROLLER_RADIUS_METER = 0.0275;    //TODO - Get the real Roller radius in meters

    /**The ratio rotations per second to meters per seoond based on the roller's raidus */
    public final static double RPM_TO_MPS_RATIO = UnitConversions.RPMtoRPS(2 * ROLLER_RADIUS_METER * Math.PI); // RPM to MPS

    /**The moment of inertia of the roller flywheel */
    private static final double MOMENT_OF_INERTIA = 0.001;//TODO: find the right value
    /** The gear ratio between the motor and the roller (a single roation of the roller is equal to GEAR_RATIO roations of the motors).*/
    public static final double GEAR_RATIO = 1; //TODO - Get the real gear ratio
    /**The gear box we are using */
    public static final DCMotor GEAR_BOX = DCMotor.getKrakenX60(2);

    /**The duty cycle value the roller is considerd on */
    public static final double onDutyCycle = 1; //TODO - tweak

    /**The duty cycle value the roller is considerd reversed */
    public static final double reversedDutyCycle = 1; //TODO - tweak
    /**The state name of turing on the roller for state machines */
    public static final StateName turnOnRollerStateName = new StateName("turnOnRollerState");
    /**The state name of turing reverse the roller for state machine */
    public static final StateName ReverseRollerStateName = new StateName("turnReverseRollerState");
    /**The state name of turing off the roller for state machines */
    public static final StateName turnOffRollerStateName = new StateName("turnOffRollerState");

    /**
     * Get the motor config of the lead motor
     * @return The SparkMaxConfig of the lead motor
     */
    public static SparkMaxConfig getLeadConfig(){
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.encoder.positionConversionFactor(RPM_TO_MPS_RATIO);  
        motorConfig.smartCurrentLimit(80,30); 
        return motorConfig;
    }

    /**
     * Get the motor config of the follower motor
     * @return The SparkMaxConfig of the follower motor
     */
    public static SparkMaxConfig getFollowerConfig(){
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        
        motorConfig.encoder.positionConversionFactor(RPM_TO_MPS_RATIO);

        motorConfig.follow(MOTOR_ONE_CANID, true);
        motorConfig.smartCurrentLimit(80,30);

        return motorConfig;
    }

    /**
     * Get the simulation plant
     * @return The simulation plant
     */
    private static LinearSystem<N1,N1,N1> getPlant(){
        return LinearSystemId.createFlywheelSystem(
            GEAR_BOX, 
            MOMENT_OF_INERTIA, 
            GEAR_RATIO
        );
    }

    /**
     * Get the simulation of the roller
     * @return The simulation of the roller
     */
    public static FlywheelSim getSim(){
        return new FlywheelSim(getPlant(), GEAR_BOX);
    }
}

//btw skebob
//also, even more skebob then last skebob