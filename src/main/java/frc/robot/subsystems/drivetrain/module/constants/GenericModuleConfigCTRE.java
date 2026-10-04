package frc.robot.subsystems.drivetrain.module.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;

public record GenericModuleConfigCTRE(
    TalonFXConfiguration steerMotorControllerConfig,
    DCMotor steerMotor,
    TalonFXConfiguration driveMotorControllerConfig,
    DCMotor driveMotor,
    double steerSpeedReduction) implements GenericModuleConfig {
    
    @Override
    public double getMaxSteerSpeed(){
        return steerMotor.freeSpeedRadPerSec * 
            steerSpeedReduction;
    }
}
