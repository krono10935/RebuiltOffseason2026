package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.IsNear;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {

    public enum HoodState {
        SETTING_ANGLE,
        HOLDING_ANGLE,
        DISABLED,
    }

    private HoodState state = HoodState.DISABLED;

    private final SparkMax hoodMotor;
    private final SingleJointedArmSim hoodSim;
    private final SparkMaxSim sparkMaxSim;

    public HoodIOSim(){
        hoodSim = HoodConstants.getSim();

        hoodMotor = new SparkMax(HoodConstants.HOOD_MOTOR_CANID, MotorType.kBrushless);
        hoodMotor.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sparkMaxSim = new SparkMaxSim(hoodMotor, HoodConstants.GEAR_BOX);
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
        hoodMotor.stopMotor();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        double vbus = RoboRioSim.getVInVoltage(); // get roborio voltage

        // update voltage and loop period
        hoodSim.setInput(sparkMaxSim.getAppliedOutput() * vbus);
        hoodSim.update(Constants.LOOP_PERIOD_SECONDS);

        // calculate rotations in hood rotations (auto converted by the config)
        double hoodRotations = hoodSim.getAngleRads() / (2.0 * Math.PI);
        double hoodRPM = Units.radiansPerSecondToRotationsPerMinute(hoodSim.getVelocityRadPerSec());

        // update the spark max sim
        sparkMaxSim.iterate(hoodRPM, vbus, Constants.LOOP_PERIOD_SECONDS);

        // update encoders (no need for conversion because of the config handling it for us)
        sparkMaxSim.setPosition(hoodRotations);
        sparkMaxSim.getAbsoluteEncoderSim().setPosition(hoodRotations);

        // set the voltage input for the roborio sim
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.getCurrentDrawAmps()));

        // update inputs
        inputs.currentAngle = Rotation2d.fromRotations(hoodRotations);
        inputs.isAtGoal = IsNear.isNear(
            inputs.currentAngle,
            Rotation2d.fromRotations(hoodMotor.getClosedLoopController().getSetpoint()),
            HoodConstants.DEGREE_TOLERANCE
        );

        // logging
        Logger.recordOutput("Hood/current state", state);
    }
}
