package frc.robot.subsystems.drivetrain.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

public final class GyroIOPigeon implements GyroIO{
    private final Pigeon2 gyro;

    public GyroIOPigeon(int id){
        this.gyro = new Pigeon2(id);
        gyro.getYaw().setUpdateFrequency(1.0 / ChassisConstants.LOOP_TIME_SECONDS);
        gyro.optimizeBusUtilization();
    }
    

    @Override
    public void reset(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.pose = new Pose2d(new Translation2d(), gyro.getRotation2d());
        inputs.stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 1,1,1);
    }
}
