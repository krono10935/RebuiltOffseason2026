package frc.robot.subsystems.drivetrain.module.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CommonModuleConstants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.ModuleConstants;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFXConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains;
import io.github.captainsoccer.basicmotor.gains.FeedForwardsGains;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

public enum SwerveModulesMK4 {

    // FRONT_LEFT(
    //         6,  0.248, 12
    //         ,
    //         new PIDGains(5, 7, 0, 0, 0, 0),
    //         new FeedForwardsGains(2.016),
    //         0.13666,
    //         4,
    //         new PIDGains(10, 40, 0, 0.05, 1, 0.001),
    //         new FeedForwardsGains(0),
    //         1.4128,0.725,
    //         new Translation2d(0.29, 0.29)),


    // FRONT_RIGHT(
    //         9, 0, 13,
    //         new PIDGains(5, 7, 0, 0, 0, 0),
    //         new FeedForwardsGains(1.8745),
    //         0.12113,
    //         5,
    //         new PIDGains(10, 40, 0, 0.05, 1, 0.001),
    //         new FeedForwardsGains(0),
    //         1.3695,0.61619,
    //         new Translation2d(0.29, -0.29)),


    // BACK_LEFT(
    //         7, 0.111, 11,
    //         new PIDGains(5, 7, 0, 0, 0, 0),
    //         new FeedForwardsGains(1.8745),
    //         0.12113,
    //         3,
    //         new PIDGains(10, 40, 0, 0.05, 1, 0.001),
    //         new FeedForwardsGains(0),
    //         1.4873,0.28857,
    //         new Translation2d(-0.29, 0.29)),


    // BACK_RIGHT(
    //         8, 0.257, 10,
    //         new PIDGains(5, 7, 0, 0, 0, 0),
    //         new FeedForwardsGains(2.05),
    //         0.12315,
    //         2,
    //         new PIDGains(10, 40, 0, 0.05, 1, 0.001),
    //         new FeedForwardsGains(0),
    //         1.3854,0.83239,
    //         new Translation2d(-0.29, -0.29));

        FRONT_LEFT(
            8, 0.257 - 0.5, 10,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(2.05),
            0.12315,
            2,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            1.3854,0.83239,
            new Translation2d(-0.29, -0.29)),


    FRONT_RIGHT(
            7, 0.111 - 0.5, 11,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(1.8745),
            0.12113,
            3,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            1.4873,0.28857,
            new Translation2d(-0.29, 0.29)),


    BACK_LEFT(
            9, 0 - 0.5, 13,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(1.8745),
            0.12113,
            5,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            1.3695,0.61619,
            new Translation2d(0.29, -0.29)),


    BACK_RIGHT(
            6,  0.248 - 0.5, 12
            ,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(2.016),
            0.13666,
            4,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            1.4128,0.725,
            new Translation2d(0.29, 0.29));


    SwerveModulesMK4(int canCoderID,
                     double zeroOffset,
                     int driveMotorID,
                     PIDGains drivePIDGains,
                     FeedForwardsGains driveFeedForwards,
                     double driveKA,
                     int steerMotorID,
                     PIDGains steerPIDGains,
                     FeedForwardsGains steerFeedForwards,
                     double steerKV,
                     double steerKA,
                     Translation2d location) {
        BasicTalonFXConfig driveConfig = getGenericConf().DRIVE_CONFIG().copy();
        BasicTalonFXConfig steerConfig = getGenericConf().STEER_CONFIG().copy();

        driveConfig.motorConfig.id = driveMotorID;
        steerConfig.motorConfig.id = steerMotorID;

        driveConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(drivePIDGains);
        steerConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(steerPIDGains);

        driveConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(driveFeedForwards);
        steerConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(steerFeedForwards);

        driveConfig.simulationConfig.kA = driveKA;

        steerConfig.simulationConfig.kV = steerKV;
        steerConfig.simulationConfig.kA = steerKA;

        driveConfig.motorConfig.name = this.name() + " drive motor";
        steerConfig.motorConfig.name = this.name() + " steer motor";

        constants = new ModuleConstants(canCoderID, zeroOffset, driveConfig, steerConfig, location,this.name());

    }

    public final ModuleConstants constants;

    private static CommonModuleConstants genericConf;

    /**
     *
     * @return the generic config
     */
    public static CommonModuleConstants getGenericConf(){
        if(genericConf != null) return genericConf;

        var driveConfig = new BasicTalonFXConfig();

        driveConfig.motorConfig.gearRatio = 6.75;
        driveConfig.motorConfig.unitConversion = 2 * Math.PI * 0.0508;
        driveConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        driveConfig.motorConfig.motorType = DCMotor.getKrakenX60(1);

        driveConfig.currentLimitConfig.statorCurrentLimit = 90;
        driveConfig.currentLimitConfig.supplyCurrentLimit = 0;

        var steerConfig = new BasicTalonFXConfig();

        steerConfig.motorConfig.gearRatio = 12.8;
        steerConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        steerConfig.motorConfig.motorType = DCMotor.getFalcon500(1);

        steerConfig.currentLimitConfig.statorCurrentLimit = 35;

        steerConfig.constraintsConfig.constraintType = ConstraintsGains.ConstraintType.CONTINUOUS;
        steerConfig.constraintsConfig.maxValue = 0.5;
        steerConfig.constraintsConfig.minValue = -0.5;


        genericConf = new CommonModuleConstants(driveConfig,steerConfig,1);
        return genericConf;
    }

    public static ModuleConstants[] getConstants(){
        ModuleConstants[] constants = new ModuleConstants[values().length];
        for(int i=0;i<values().length;i++){
            constants[i] = values()[i].constants;
        }
        return constants;
    }
}
