package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.IsNear;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO{

    private final TalonFX[] flywheelMotors; 
    private final FlywheelSim flywheelSim;

    public FlywheelIOSim(){

        flywheelSim = FlywheelConstants.getSim();
        flywheelMotors = new TalonFX[4];

        flywheelMotors[0] = new TalonFX(FlywheelConstants.MOTOR_IDS[0]); // flywheelMotors[0] is the leading motor
        flywheelMotors[0].getConfigurator().apply(FlywheelConstants.getLeadConfig(FlywheelConstants.LEAD_INVERTED));

        for (int i = 1; i < flywheelMotors.length; i++){
            flywheelMotors[i] = new TalonFX(FlywheelConstants.MOTOR_IDS[i]);

            flywheelMotors[i].getConfigurator().apply(FlywheelConstants.getFollowerConfig(i >= flywheelMotors.length / 2)); // set half the motors to inverted

            flywheelMotors[i].setControl(new Follower(FlywheelConstants.MOTOR_IDS[0], MotorAlignmentValue.Aligned)); // TODO: understand how aligned works, we set it to inverted above this
        }
    }

    /**
     * @return returns the lead motor
     */
    private TalonFX getLeadMotor()
    {
        return flywheelMotors[0];
    }

    @Override
    public void setSpeed(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps).withSlot(0));
    }

    @Override
    public void holdSpeed(double mps) {
        getLeadMotor().setControl(new VelocityVoltage(mps).withSlot(1));
    }

    @Override
    public void stop() {
        getLeadMotor().stopMotor();
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {

        TalonFXSimState simState = getLeadMotor().getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simState.getMotorVoltage();

        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.addRotorPosition(
            UnitConversions.RPMtoRotationsPerCycle(
                flywheelSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS)
                * FlywheelConstants.GEAR_RATIO);
        
        simState.setRotorVelocity(
            flywheelSim.getAngularVelocity().times(
                FlywheelConstants.GEAR_RATIO
            )
        ); 

        inputs.speedMPS = getLeadMotor().getVelocity().getValueAsDouble();

        inputs.isAtGoal = IsNear.isNear(inputs.speedMPS, getLeadMotor().getClosedLoopReference().getValueAsDouble(), FlywheelConstants.MPS_TOLERANCE);
    }

}
