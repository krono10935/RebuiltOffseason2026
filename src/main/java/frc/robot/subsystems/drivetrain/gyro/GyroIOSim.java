package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;
import java.util.function.Supplier;

public final class GyroIOSim implements GyroIO{

    private final Supplier<ChassisSpeeds> speedsSupplier;
    
    private Rotation2d angle = Rotation2d.kZero;

    public GyroIOSim(Supplier<ChassisSpeeds> speedsSupplier){
        this.speedsSupplier = speedsSupplier;
    }


    @Override
    public Optional<GyroPoseOutput> getEstimatedPosition() {
        return Optional.empty();
    }

    @Override
    public void reset(Pose2d pose) {
        this.angle = pose.getRotation();
    }

    @Override
    public Rotation2d update() {
        double omega =speedsSupplier.get().omegaRadiansPerSecond;
        angle = Rotation2d.fromRadians(angle.getRadians() + omega * 0.02);
        return angle;
    }



}
