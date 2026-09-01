package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {

    private final TalonFX motor;
    private final SingleJointedArmSim pivotSim;
    TalonFXSimState motorSim;

    public PivotIOSim(){
        motor = new TalonFX(PivotConstants.MOTOR_CANID);
        pivotSim = PivotConstants.getSim();
        motorSim = motor.getSimState();
    }
    @Override
    public void setRotation(Rotation2d rotation) {
        motor.setControl(new PositionVoltage(rotation.getRotations()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Steps the simulation by Constants.LOOP_PERIOD_SECONDS(20ms).
     */
    private void simulateStep(){

        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        pivotSim.update(Constants.LOOP_PERIOD_SECONDS);

        motorSim.addRotorPosition(
            pivotSim.getAngleRads() / PivotConstants.GEAR_RATIO - motor.getPosition().getValueAsDouble()
        );
        
        motorSim.setRotorVelocity(
            motor.getVelocity().getValueAsDouble()
        ); 
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        simulateStep();        
        
        inputs.angle = Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
        inputs.motorTempC = motor.getDeviceTemp().getValueAsDouble();
        inputs.angularVelocityRPS = motor.getVelocity().getValueAsDouble();

    }
}