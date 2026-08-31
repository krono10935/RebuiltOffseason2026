package frc.robot.subsystems.shooter.hood;

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

    private final SparkMax hoodMotor;
    private final SingleJointedArmSim hoodSim;

    public HoodIOSim(){
        hoodSim = HoodConstants.getSim();

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
        SparkMaxSim sparkMaxSim = new SparkMaxSim(hoodMotor, HoodConstants.GEAR_BOX); // get the sim state

        hoodSim.setInput(sparkMaxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        hoodSim.update(Constants.LOOP_PERIOD_SECONDS);

        // Now, we update the Spark Flex
        sparkMaxSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                hoodSim.getVelocityRadPerSec()),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            Constants.LOOP_PERIOD_SECONDS); // Time interval, in Seconds

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.getCurrentDrawAmps()));

        inputs.currentAngle = Rotation2d.fromRotations(hoodMotor.getEncoder().getPosition());

        inputs.isAtGoal = IsNear.isNear(inputs.currentAngle, Rotation2d.fromRotations(hoodMotor.getClosedLoopController().getSetpoint()), HoodConstants.DEGREE_TOLERANCE);
    }

}
