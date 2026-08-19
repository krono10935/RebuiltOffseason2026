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

public enum SwerveModulesMK5 {

    FRONT_LEFT(
            6,  -0.430, 12
            ,
            new PIDGains(7, 0.4, 0, 0, 0, 0),
            new FeedForwardsGains(2.4805, 0.32),
            0.32661,
            4,
            new PIDGains(30, 5, 0, 0, 0, 0),
            new FeedForwardsGains(2.5776, 0),
            2.5776,0.37782,
            new Translation2d(0.3, 0.3),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),


    FRONT_RIGHT(
            8,  -0.45, 13
            ,
            new PIDGains(7, 0.4, 0, 0, 0, 0),
            new FeedForwardsGains(2.3745, 0.32),
            0.40069,
            5,
            new PIDGains(30, 5, 0, 0, 0, 0),
            new FeedForwardsGains(2.4944, 0),
            2.4944,1.3643,
            new Translation2d(0.3, -0.3),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),

    BACK_LEFT(
            7,  0.2624, 11
            ,
            new PIDGains(7, 0.4, 0, 0, 0, 0),
            new FeedForwardsGains(2.4752, 0.32),
            0.91735,
            3,
            new PIDGains(30, 5, 0, 0, 0, 0),
            new FeedForwardsGains(2.4895, 0),
            2.4895,0.83686,
            new Translation2d(-0.3, 0.3),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),


    BACK_RIGHT(
            9,  0.236, 10
            ,
            new PIDGains(7, 0.4, 0, 0, 0, 0),
            new FeedForwardsGains(2.3786, 0.32),
            0.71613,
            2,
            new PIDGains(30, 5, 0, 0, 0, 0),
            new FeedForwardsGains(2.5978, 0),
            2.5978,0.53702,
            new Translation2d(-0.3, -0.3),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),
    ;


    SwerveModulesMK5(int canCoderID,
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
                     Translation2d location,
                     PIDGains drivePIDGainsWithBalls,
                     FeedForwardsGains driveFeedForwardsWithBalls,
                     PIDGains steerPIDGainsWithBalls,
                     FeedForwardsGains steerFeedForwardsWithBalls) {

        BasicTalonFXConfig driveConfig = getGenericConf().DRIVE_CONFIG().copy();
        BasicTalonFXConfig steerConfig = getGenericConf().STEER_CONFIG().copy();

        driveConfig.motorConfig.id = driveMotorID;
        steerConfig.motorConfig.id = steerMotorID;

        driveConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(drivePIDGains);
        steerConfig.slot0Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(steerPIDGains);

        driveConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(driveFeedForwards);
        steerConfig.slot0Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(steerFeedForwards);

        driveConfig.slot1Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(driveFeedForwardsWithBalls);
        steerConfig.slot1Config.feedForwardConfig = BasicMotorConfig.FeedForwardConfig.fromFeedForwards(steerFeedForwardsWithBalls);

        driveConfig.slot1Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(drivePIDGainsWithBalls);
        driveConfig.slot1Config.pidConfig = BasicMotorConfig.PIDConfig.fromGains(steerPIDGainsWithBalls);

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

        driveConfig.motorConfig.gearRatio = 6.03;
        driveConfig.motorConfig.unitConversion = 2 * Math.PI * 0.0508;
        driveConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        driveConfig.motorConfig.motorType = DCMotor.getKrakenX60(1);

        driveConfig.currentLimitConfig.statorCurrentLimit = 120;
        driveConfig.currentLimitConfig.supplyCurrentLimit = 65;
        driveConfig.currentLimitConfig.lowerCurrentLimit = 40;
        driveConfig.currentLimitConfig.lowerLimitTime = 0.2;

        driveConfig.enableFOC = true;

        var steerConfig = new BasicTalonFXConfig();

        steerConfig.enableFOC = true;

        steerConfig.motorConfig.gearRatio = 26.1;
        steerConfig.motorConfig.idleMode = BasicMotor.IdleMode.COAST;
        steerConfig.motorConfig.motorType = DCMotor.getKrakenX44(1);

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