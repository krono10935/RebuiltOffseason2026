package frc.robot.subsystems.drivetrain.gyro;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

import java.util.function.Supplier;

public final class GyroIOSim implements GyroIO{

    private final Supplier<ChassisSpeeds> speedsSupplier;
    
    private Rotation2d angle = Rotation2d.kZero;

    public GyroIOSim(Supplier<ChassisSpeeds> speedsSupplier){
        this.speedsSupplier = speedsSupplier;
    }


    @Override
    public void reset(Pose2d pose) {
        this.angle = pose.getRotation();
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        angle = angle.plus(
            Rotation2d.fromRadians(speedsSupplier.get().omegaRadiansPerSecond)
            .times(Constants.LOOP_PERIOD_SECONDS)
        );

        inputs.pose = new Pose2d(new Translation2d(), angle);
        inputs.stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 1,1,1);
    }
}
