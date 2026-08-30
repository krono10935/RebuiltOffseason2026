package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;


public class PivotConstants {
    public final static int MOTOR_CANID = 4;

    public static TalonFXConfiguration getMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kP = 0;

        config.Slot0.kG = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 90;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;        
    }
}
