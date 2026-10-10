package frc.robot.subsystems.drivetrain.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

public final class GyroIOPigeon implements GyroIO{
    private final Pigeon2 gyro;

    public GyroIOPigeon(int id){
        this.gyro = new Pigeon2(id);
        gyro.getYaw().setUpdateFrequency(1.0 / ChassisConstants.LOOP_TIME_SECONDS);
        gyro.optimizeBusUtilization();
    }
    

    @Override
    public void reset(Rotation2d rotation) {
        gyro.setYaw(rotation.getDegrees());
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.rotation = gyro.getRotation2d();
    }
}
