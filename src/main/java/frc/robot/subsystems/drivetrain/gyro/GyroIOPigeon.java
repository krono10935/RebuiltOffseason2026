package frc.robot.subsystems.drivetrain.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;


import java.util.Optional;


public final class GyroIOPigeon implements GyroIO{
    private final Pigeon2 gyro;

    private Rotation2d angleOffset = Rotation2d.kZero;

    public GyroIOPigeon(int id){
        this.gyro = new Pigeon2(id);
        gyro.getYaw().setUpdateFrequency(1.0 / ChassisConstants.LOOP_TIME_SECONDS);
        gyro.optimizeBusUtilization();
    }
    
    @Override
    public Optional<GyroPoseOutput> getEstimatedPosition() {
        return Optional.empty();
    }

    @Override
    public void reset(Pose2d pose) {
        gyro.reset();
        angleOffset = pose.getRotation();
    }

    @Override
    public Rotation2d update() {
        return gyro.getRotation2d().rotateBy(angleOffset);
    }


}
