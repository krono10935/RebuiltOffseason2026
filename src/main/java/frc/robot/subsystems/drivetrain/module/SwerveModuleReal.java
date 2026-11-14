package frc.robot.subsystems.drivetrain.module;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.CTREModuleConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveModuleReal extends SwerveModuleCTRE {

    private final CANcoder canCoder;

    public SwerveModuleReal(CTREModuleConstants constants){
        super(constants);

        canCoder = createCANcoder(constants);

        steeringMotor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(constants.CAN_CODER_ID())
                // .withSensorToMechanismRatio(constants.STEERING_CONFIG().Feedback.SensorToMechanismRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));         

        canCoder.getMagnetHealth().setUpdateFrequency(4);
        canCoder.optimizeBusUtilization();
    }

    

    @Override
    public void update(){
        super.update();
        Logger.recordOutput("drivetrain/ctre module/" + constants.NAME() + "/magnet health",
                    canCoder.getMagnetHealth().toString());
    }

    /**
     * Auto configs a CANCoder for the module
     * @param constants The constants of the module
     * @return A configured CANCoder for the module
     */
    private static CANcoder createCANcoder(CTREModuleConstants constants){
        CANcoder encoder = new CANcoder(constants.CAN_CODER_ID(), Constants.canivore);
        var config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -constants.ZERO_OFFSET();

        encoder.getConfigurator().apply(config);

        encoder.getPosition().setUpdateFrequency(250);
        encoder.optimizeBusUtilization();

        return encoder;
    }


}
