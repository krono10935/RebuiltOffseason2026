package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {

    private final TalonFX motor;
    private final SingleJointedArmSim pivotSim;

    public PivotIOSim(){
        motor = new TalonFX(PivotConstants.MOTOR_CANID);
        pivotSim = PivotConstants.getSim();
    }
    @Override
    public void setRotation(Rotation2d rotation) {
        motor.setControl(new PositionVoltage(rotation.getRotations()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public void step(){
        TalonFXSimState motorSim = motor.getSimState(); // get the sim state

        motorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Next, we update it. The standard loop time is 20ms.
        motorSimSim.update(Constants.LOOP_PERIOD_SECONDS);

        // Now, we update the Spark Flex
        sparkMaxSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                        motorSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                Constants.LOOP_PERIOD_SECONDS); // Time interval, in Seconds
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
        inputs.motorTempC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();
    }
}