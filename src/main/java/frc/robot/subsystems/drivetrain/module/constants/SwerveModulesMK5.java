package frc.robot.subsystems.drivetrain.module.constants;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.math.PIDGains;
import frc.lib.simulation.SimulationConfig;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;
import frc.lib.math.FeedForwardsGains;

public enum SwerveModulesMK5 {
    FRONT_LEFT(
            6,  0, 12
            ,
            new PIDGains(0, 0, 0, 0, 0, 0),
            new FeedForwardsGains(0, 0),
            0.32661,
            4,
            new PIDGains(SwerveModulesMK4.p, SwerveModulesMK4.i, SwerveModulesMK4.d, 0, SwerveModulesMK4.iMax , 0),
            new FeedForwardsGains(0.099965, 0.52098),
            0.011139,
            new Translation2d(0.276375, 0.289025),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),


    FRONT_RIGHT(
            8,  -0.188477, 13
            ,
            new PIDGains(0, 0, 0, 0, 0, 0),
            new FeedForwardsGains(0, 0),
            0.40069,
            5,
            new PIDGains(SwerveModulesMK4.p, SwerveModulesMK4.i, SwerveModulesMK4.d, 0, SwerveModulesMK4.iMax , 0),
            new FeedForwardsGains(0.10363, 0.29686),
            0.0060669,
            new Translation2d(0.276375, -0.289025),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),

    BACK_LEFT(
            7, -0.015625, 11
            ,
            new PIDGains(0, 0, 0, 0, 0, 0),
            new FeedForwardsGains(0, 0),
            0.91735,
            3,
            new PIDGains(SwerveModulesMK4.p, SwerveModulesMK4.i, SwerveModulesMK4.d, 0, SwerveModulesMK4.iMax , 0),
            new FeedForwardsGains(0.10471, 0.40178),
            0.0052174,
            new Translation2d(-0.276375, 0.289025),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains()),


    BACK_RIGHT(
            9,  0.79, 10
            ,
            new PIDGains(0, 0, 0, 0, 0, 0),
            new FeedForwardsGains(0, 0),
            0.71613,
            2,
            new PIDGains(SwerveModulesMK4.p, SwerveModulesMK4.i, SwerveModulesMK4.d, 0, SwerveModulesMK4.iMax , 0),
            new FeedForwardsGains(0.10256, 0.52537),
            0.010426,
            new Translation2d(-0.276375, -0.289025),
            new PIDGains(),
            new FeedForwardsGains(),
            new PIDGains(),
            new FeedForwardsGains());
            
    private static final double RADIUS = 0.0508;
    private static final double UNIT_CONVERSION = 2 * Math.PI * RADIUS;

    private static final double STEER_GEAR_RATIO = 26.1;
    private static final double DRIVE_GEAR_RATIO = 6.03;



    SwerveModulesMK5(int canCoderID,
                     double zeroOffset,
                     int driveMotorID,
                     PIDGains drivePIDGains,
                     FeedForwardsGains driveFeedForwards,
                     double driveKA,
                     int steerMotorID,
                     PIDGains steerPIDGains,
                     FeedForwardsGains steerFeedForwards,
                     double steerKA,
                     Translation2d location,
                     PIDGains drivePIDGainsWithBalls,
                     FeedForwardsGains driveFeedForwardsWithBalls,
                     PIDGains steerPIDGainsWithBalls,
                     FeedForwardsGains steerFeedForwardsWithBalls) {

        TalonFXConfiguration driveConfig = getGenericConf().driveMotorControllerConfig().clone();
        TalonFXConfiguration steerConfig = getGenericConf().steerMotorControllerConfig().clone();

        SlotConfigs driveConfigSlot0 = new SlotConfigs();
        SlotConfigs steerConfigSlot0 = new SlotConfigs();


        driveConfig.withSlot0(Slot0Configs.from(
            driveFeedForwards.applyConfigCTRE(
                drivePIDGains.applyConfigCTRE(driveConfigSlot0)
            )
        ));

        
        steerConfig.withSlot0(Slot0Configs.from(
            steerFeedForwards.applyConfigCTRE(
                steerPIDGains.applyConfigCTRE(steerConfigSlot0)
            )
        ));



        SlotConfigs driveConfigSlot1 = new SlotConfigs();
        SlotConfigs steerConfigSlot1 = new SlotConfigs();

        driveConfig.withSlot1(Slot1Configs.from(
            driveFeedForwards.applyConfigCTRE(
                drivePIDGains.applyConfigCTRE(driveConfigSlot1)
            )
        ));

        
        steerConfig.withSlot1(Slot1Configs.from(
            steerFeedForwards.applyConfigCTRE(
                steerPIDGains.applyConfigCTRE(steerConfigSlot1)
            )
        ));


        SimulationConfig driveSimConfig = new SimulationConfig(
            Optional.of(driveFeedForwards.withkA(driveKA)), 
            0,
            0,
            getGenericConf().driveMotor()
        );

        SimulationConfig steerSimConfig = new SimulationConfig(
            Optional.of(steerFeedForwards.withkA(steerKA)),
            0,
            0,
            getGenericConf().steerMotor()
        );


        constants = new CTREModuleConstants(
            canCoderID, steerMotorID, driveMotorID,
            driveSimConfig, steerSimConfig, UNIT_CONVERSION, zeroOffset,
             STEER_GEAR_RATIO, DRIVE_GEAR_RATIO,
            driveConfig, steerConfig, location, this.name());

    }

    public final CTREModuleConstants constants;

    private static GenericModuleConfigCTRE genericConf;

    /**
     * 
     * @return the generic config
     */
    public static GenericModuleConfigCTRE getGenericConf(){
        if(genericConf != null) return genericConf;

        var driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        driveConfig.CurrentLimits.StatorCurrentLimit = 120;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 65;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;

        // TODO check how the motors react to this being enabled
        driveConfig.Feedback.RotorToSensorRatio = DRIVE_GEAR_RATIO;

        var steerConfig = new TalonFXConfiguration();

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        steerConfig.CurrentLimits.StatorCurrentLimit = 35;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        steerConfig.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        genericConf = new GenericModuleConfigCTRE(
            steerConfig,
            DCMotor.getKrakenX44Foc(1),
            driveConfig,
            DCMotor.getKrakenX60Foc(1));
        return genericConf;
    }

    public static CTREModuleConstants[] getConstants(){
        CTREModuleConstants[] constants = new CTREModuleConstants[values().length];
        for(int i=0;i<values().length;i++){
            constants[i] = values()[i].constants;
        }
        return constants;
    }
}