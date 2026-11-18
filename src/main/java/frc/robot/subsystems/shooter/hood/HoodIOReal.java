package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.IsNear;

public class HoodIOReal implements HoodIO {

    public enum HoodState {
        SETTING_ANGLE,
        HOLDING_ANGLE,
        DISABLED,
    }

    private HoodState state = HoodState.DISABLED;;
    
    private final SparkMax hoodMotor;

    private final DutyCycleEncoder absoluteEncoder;
    private final double realPositionOffsetRot = 0.104;

    double setpoint = 0 ;

    PIDController controller = new PIDController(360, 0,  0);
    public HoodIOReal()
    {
        absoluteEncoder = new DutyCycleEncoder(HoodConstants.ABSOLUTE_ENCODER_PORT);
        absoluteEncoder.setInverted(true);

        hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_CANID, MotorType.kBrushless);
        hoodMotor.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoder();
    }

    /**
     * transform the absoluteEncoder's position to the encoder's position
     * @return the transformed position
     */
    private double absoluteEncoderToEncoderPosition() {
        return absoluteEncoder.get() / 8 ;
    }
    // BLACK MAGIC!!!! NO ONE BUT CLAUDE CAN TOUCH
    private double getHoodPosition(){
        return MathUtil.inputModulus(absoluteEncoder.get() - realPositionOffsetRot, -0.022 , -0.022+1); 
    }

    /**
     * reset the motor's built in encoder to the absolute encoder's position
     */
    private void resetEncoder() {
        hoodMotor.getEncoder().setPosition(
          getHoodPosition()/8);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        // if (state != HoodState.SETTING_ANGLE)
        //     resetPIDController();
        setpoint = angle.getRotations();
        // state = HoodState.SETTING_ANGLE;
        // hoodMotor.getClosedLoopController().setSetpoint(angle.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
        //inputs.currentAngle = Rotation2d.fromRotations(absoluteEncoderToEncoderPosition() - realPositionOffsetRot); // TODO: make sure it's an absolute encoder
        inputs.currentAngle = Rotation2d.fromRotations(getHoodPosition()/8.0);
        Logger.recordOutput("good", absoluteEncoder.get());
        inputs.isAtGoal = IsNear.isNear(inputs.currentAngle, Rotation2d.fromRotations(hoodMotor.getClosedLoopController().getSetpoint()), HoodConstants.DEGREE_TOLERANCE);

        resetEncoder();
        if (state != HoodState.DISABLED){
            hoodMotor.setVoltage(controller.calculate(getHoodPosition()/8.0, setpoint));
        }
        
        Logger.recordOutput("Hood/Error" , controller.getError());

        Logger.recordOutput("Hood/current state", state);
    }
}
