package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
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
        motor.setControl(new PositionVoltage(rotation.getRotations()));
    }

    @Override
    public void setRotationSlow(Rotation2d rotation) {
        motor.setControl(new MotionMagicVoltage(rotation.getRotations()).withSlot(1));
     }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetMotorEncoder() { return; }

    /**
     * Steps the simulation by Constants.LOOP_PERIOD_SECONDS(20ms).
     */
    private void simulationStep(){
        TalonFXSimState motorSim = motor.getSimState();

        pivotSim.setInputVoltage(motor.getClosedLoopOutput().getValueAsDouble());

        // Next, we update it. The standard loop time is 20ms.
        pivotSim.update(Constants.LOOP_PERIOD_SECONDS);

        motorSim.setRawRotorPosition(
            UnitConversions.radiansToRotations(pivotSim.getAngleRads() * PivotConstants.MOTOR_TO_ARM_RATIO)
        );

        motorSim.setRotorVelocity(
            UnitConversions.radiansPerSecondToRotationsPerSecond(pivotSim.getVelocityRadPerSec() * PivotConstants.MOTOR_TO_ARM_RATIO)
        );
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        Logger.recordOutput("pivotSim/arm and motor/Pivot arm raw", UnitConversions.radiansToDegrees(pivotSim.getAngleRads()));
        simulationStep();

        inputs.angle = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.motorTemperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();

        Logger.recordOutput("pivotSim/arm and motor/Encoder mesurments", UnitConversions.rotationsToDegrees(motor.getRotorPosition().getValueAsDouble()));
        Logger.recordOutput("pivotSim/arm and motor/arm speed: RPS", UnitConversions.radiansPerSecondToRotationsPerSecond(pivotSim.getVelocityRadPerSec()));
        Logger.recordOutput("pivotSim/arm and motor/Motor speed RPS", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("pivotSim/arm and motor/Motor pos", UnitConversions.radiansToRotations(pivotSim.getAngleRads() * PivotConstants.MOTOR_TO_ARM_RATIO));
        
        double error = UnitConversions.rotationsToDegrees(motor.getClosedLoopReference().getValueAsDouble()) - UnitConversions.radiansToDegrees(pivotSim.getAngleRads());
        Logger.recordOutput("pivotSim/PID/setPoint", UnitConversions.rotationsToDegrees(motor.getClosedLoopReference().getValueAsDouble()));
        Logger.recordOutput("pivotSim/PID/P", motor.getClosedLoopProportionalOutput().getValueAsDouble());
        Logger.recordOutput("pivotSim/PID/I", motor.getClosedLoopIntegratedOutput().getValueAsDouble());
        Logger.recordOutput("pivotSim/PID/D", motor.getClosedLoopDerivativeOutput().getValueAsDouble());
        Logger.recordOutput("pivotSim/PID/error", UnitConversions.rotationsToDegrees(motor.getClosedLoopError().getValueAsDouble()));
        Logger.recordOutput("pivotSim/PID/real error", error);
        Logger.recordOutput("pivotSim/PID/output", motor.getClosedLoopOutput().getValueAsDouble());
    }
}