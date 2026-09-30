package frc.robot.subsystems.drivetrain.gyro;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

import java.util.function.Supplier;

public final class GyroIOSim implements GyroIO{

    private Supplier<ChassisSpeeds> speedsSupplier;
    
    private Rotation2d angle = Rotation2d.kZero;

    public GyroIOSim(Supplier<ChassisSpeeds> speedsSupplier){
        this.speedsSupplier = speedsSupplier;
    }


    @Override
    public void reset(Rotation2d rotation) {
        this.angle = rotation;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        angle = angle.plus(
            Rotation2d.fromRadians(speedsSupplier.get().omegaRadiansPerSecond)
            .times(Constants.LOOP_PERIOD_SECONDS)
        );

        inputs.rotation =  angle;
    }
}
