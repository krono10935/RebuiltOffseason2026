package frc.robot.subsystems.drivetrain.module.constants;

import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.math.PIDGains;
import frc.lib.simulation.SimulationConfig;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;
import frc.lib.math.FeedForwardsGains;

public enum SwerveModulesMK4 {
        FRONT_LEFT(
            8, 0.257 - 0.5, 10,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(2.05),
            0.12315,
            2,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            0.83239,
            new Translation2d(-0.29, -0.29)),


    FRONT_RIGHT(
            7, 0.111 - 0.5, 11,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(1.8745),
            0.12113,
            3,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            0.28857,
            new Translation2d(-0.29, 0.29)),


    BACK_LEFT(
            9, 0 - 0.5, 13,
            new PIDGains(5, 7, 0, 0, 0, 0),
            new FeedForwardsGains(1.8745),
            0.12113,
            5,
            new PIDGains(15, 40, 0, 0.05, 1, 0.001),
            new FeedForwardsGains(0),
            0.61619,
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
            0.725,
            new Translation2d(0.29, 0.29));


    private static final double RADIUS = 0.0508;
    private static final double UNIT_CONVERSION = 2 * Math.PI * RADIUS;

    SwerveModulesMK4(int canCoderID,
                     double zeroOffset,
                     int driveMotorID,
                     PIDGains drivePIDGains,
                     FeedForwardsGains driveFeedForwards,
                     double driveKA,
                     int steerMotorID,
                     PIDGains steerPIDGains,
                     FeedForwardsGains steerFeedForwards,
                     double steerKA,
                     Translation2d location) {
        
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
            driveConfig, steerConfig, location,this.name());

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

        driveConfig.Feedback.SensorToMechanismRatio = 6.75;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        driveConfig.CurrentLimits.StatorCurrentLimit = 90;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        var steerConfig = new TalonFXConfiguration();


        steerConfig.Feedback.SensorToMechanismRatio = 12.8;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        steerConfig.CurrentLimits.StatorCurrentLimit = 35;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;



        genericConf = new GenericModuleConfigCTRE(
            steerConfig, 
            DCMotor.getFalcon500Foc(1), 
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
