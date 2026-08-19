package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drivetrain.configsStructure.moduleConfig.ModuleConstants;
import io.github.captainsoccer.basicmotor.ctre.talonfx.BasicTalonFX;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;

public class SwerveModuleBasic extends SwerveModuleIO {

    private final BasicMotor drivingMotor;
    private final BasicMotor steeringMotor;

    private final CANcoder canCoder;

    public SwerveModuleBasic(ModuleConstants constants){
        super(constants);

        if (RobotBase.isReal()) {
            drivingMotor = new BasicTalonFX(constants.DRIVING_CONFIG());
            steeringMotor = new BasicTalonFX(constants.STEERING_CONFIG());

            canCoder = createCANcoder(constants);

            // steeringMotor.resetEncoder(canCoder.getAbsolutePosition().getValueAsDouble());

            ((BasicTalonFX)steeringMotor).useFusedCanCoder(canCoder, constants.STEERING_CONFIG().motorConfig.gearRatio);
//            ((BasicTalonFX)steeringMotor).useFusedCanCoder(canCoder, 1);

            canCoder.getMagnetHealth().setUpdateFrequency(4);
            canCoder.optimizeBusUtilization();
        }
        else{
            drivingMotor = new BasicMotorSim(constants.DRIVING_CONFIG());
            steeringMotor = new BasicMotorSim(constants.STEERING_CONFIG());

            canCoder = null;
        }
    }

    @Override
    protected double getDriveVelocity() {
        if (drivingMotor.isConnected())
            return drivingMotor.getVelocity();
        else
            return 0;
    }

    @Override
    protected double getDrivePos() {
        return drivingMotor.getPosition();
    }

    @Override
    protected double getSteerAngle() {
        return steeringMotor.getPosition();
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        drivingMotor.setControl(targetState.speedMetersPerSecond,ControlMode.VELOCITY);
        steeringMotor.setControl(targetState.angle.getRotations(),ControlMode.POSITION);
    }

    //TODO: tune pid with balls
    @Override
    public void setTargetStateWithBalls(SwerveModuleState targetState){
        drivingMotor.setControl(targetState.speedMetersPerSecond, ControlMode.VELOCITY);
        steeringMotor.setControl(targetState.angle.getRotations(), ControlMode.POSITION);
    }

    @Override
    public void setSteerVoltage(double voltage){
        steeringMotor.setVoltage(voltage);
    }

    @Override
    public void setDriveVoltageAndSteerAngle(double voltage, Rotation2d angle) {
        steeringMotor.setControl(angle.getRotations(), ControlMode.POSITION);
        drivingMotor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        IdleMode idleMode = isBrake ? IdleMode.BRAKE : IdleMode.COAST;

        drivingMotor.setIdleMode(idleMode);
        steeringMotor.setIdleMode(idleMode);
    }

    @Override
    public void update(){
        super.update();
        if (canCoder != null){
            Logger.recordOutput("basic module/" + constants.NAME() + "/magnet health",
                    canCoder.getMagnetHealth().toString());
        }
    }

    /**
     * Auto configs a CANCoder for the module
     * @param constants The constants of the module
     * @return A configured CANCoder for the module
     */
    private static CANcoder createCANcoder(ModuleConstants constants){
        CANcoder encoder = new CANcoder(constants.CAN_CODER_ID());
        var config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -constants.ZERO_OFFSET();

        encoder.getConfigurator().apply(config);
        return encoder;
    }


}
