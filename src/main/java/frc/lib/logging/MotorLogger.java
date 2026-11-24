package frc.lib.logging;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class MotorLogger {
    private record MotorLogData(
        int canID,        
        double angle,
        double speedMPS,
        double temperatureCelsius,
        double voltage,
        double current
        ) {
    }

    private static void logMotorData(String key, MotorLogData data) {
        Logger.recordOutput(key + "CanID", data.canID());
        Logger.recordOutput(key + "Angle (after conversion)", Rotation2d.fromRotations(data.angle()));
        Logger.recordOutput(key + "Speed (MPS, after conversion)", data.speedMPS());
        Logger.recordOutput(key + "Temperature (Celsius)", data.temperatureCelsius());
        Logger.recordOutput(key + "Voltage", data.voltage());
        Logger.recordOutput(key + "Current", data.current());
    }

    public static void log(String key, SparkMax motor) {
        MotorLogData logData = new MotorLogData(
            motor.getDeviceId(),
            motor.getEncoder().getPosition(),
            motor.getEncoder().getVelocity(),
            motor.getMotorTemperature(),
            motor.getBusVoltage(),
            motor.getOutputCurrent()
        );
        
        logMotorData(key, logData);
    }

    public static void log(String key, TalonFX motor) {
        MotorLogData logData = new MotorLogData(
            motor.getDeviceID(),
            motor.getPosition().getValueAsDouble(),
            motor.getVelocity().getValueAsDouble(),
            motor.getDeviceTemp().getValueAsDouble(),
            motor.getSupplyVoltage().getValueAsDouble(),
            motor.getSupplyCurrent().getValueAsDouble()
        );
        
        logMotorData(key, logData);
    }
}
