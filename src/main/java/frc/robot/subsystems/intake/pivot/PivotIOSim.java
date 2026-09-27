package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {

    private final TalonFX motor;
    private final SingleJointedArmSim pivotSim;

    public PivotIOSim(){
        motor = new TalonFX(PivotConstants.MOTOR_CANID);
        
        motor.getConfigurator().apply(PivotConstants.getMotorConfig());

        pivotSim = PivotConstants.getSim();

    }
    @Override
    public void setRotation(Rotation2d rotation) {
        motor.setControl(new PositionVoltage(rotation.getRotations()).withSlot(0));
    }

    @Override
    public void setRotationSlow(Rotation2d rotation) {
        motor.setControl(new MotionMagicVoltage(rotation.getRotations()).withSlot(1));
     }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Steps the simulation by Constants.LOOP_PERIOD_SECONDS(20ms).
     */
    private void simulateStep(){
        
        TalonFXSimState motorSim = motor.getSimState();

        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        pivotSim.update(Constants.LOOP_PERIOD_SECONDS);

        motorSim.setRawRotorPosition(
            UnitConversions.radiansToRotations(pivotSim.getAngleRads() * PivotConstants.GEAR_RATIO)
        );
        
        motorSim.setRotorVelocity(
            UnitConversions.radiansPerSecondToRotationsPerSecond(pivotSim.getVelocityRadPerSec() * PivotConstants.GEAR_RATIO)
        );


    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        simulateStep();

        inputs.angle = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.motorTemperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();

        Logger.recordOutput("p", motor.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("i", motor.getClosedLoopIntegratedOutput().getValueAsDouble());
        Logger.recordOutput("d", motor.getClosedLoopDerivativeOutput().getValueAsDouble());

        Logger.recordOutput("error", motor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("output", motor.getClosedLoopOutput().getValueAsDouble());
    }
}