package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.IsNear;

public class HoodIOReal implements HoodIO {

    public enum HoodState {
        SETTING_ANGLE,
        HOLDING_ANGLE,
        DISABLED,
    }

    private HoodState state = HoodState.DISABLED;;
    
    private final SparkMax hoodMotor;

    public HoodIOReal()
    {
        hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_CANID, MotorType.kBrushless);
        hoodMotor.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        if (state != HoodState.SETTING_ANGLE)
            resetPIDController();

        state = HoodState.SETTING_ANGLE;
        hoodMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void holdAngle(Rotation2d angle) {
        if (state != HoodState.HOLDING_ANGLE)
            resetPIDController();
        state = HoodState.HOLDING_ANGLE;
        hoodMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void resetPIDController() {
        hoodMotor.getClosedLoopController().setIAccum(0.0);
    }

    @Override
    public void stop() {
        state = HoodState.DISABLED;

        hoodMotor.stopMotor();
    }
    
    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.currentAngle = Rotation2d.fromRotations(hoodMotor.getAbsoluteEncoder().getPosition()); // TODO: make sure it's an absolute encoder
        
        inputs.isAtGoal = IsNear.isNear(inputs.currentAngle, Rotation2d.fromRotations(hoodMotor.getClosedLoopController().getSetpoint()), HoodConstants.DEGREE_TOLERANCE);

        Logger.recordOutput("Hood/current state", state);
    }
}
