package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.IsNear;

public class HoodIOReal implements HoodIO{

    private final SparkMax hoodMotor;

    public HoodIOReal()
    {
        hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_CANID, MotorType.kBrushless);
        hoodMotor.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        hoodMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void holdAngle(Rotation2d angle) {
        hoodMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.currentAngle = Rotation2d.fromRotations(hoodMotor.getEncoder().getPosition());

        inputs.isAtGoal = IsNear.isNear(inputs.currentAngle, Rotation2d.fromRotations(hoodMotor.getClosedLoopController().getSetpoint()), HoodConstants.DEGREE_TOLERANCE);
    }

}
