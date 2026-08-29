package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO{
    private final TalonFX[] motors; 
    private final FlywheelSim indexerSim;

    public IndexerIOSim(){
        motors = new TalonFX[3];

        indexerSim = IndexerConstants.getSim();

        for (int i = 0; i < motors.length; i++) {
            motors[i] = new TalonFX(IndexerConstants.CAN_IDS[i]);
            motors[i].getConfigurator().apply(IndexerConstants.getConfig(IndexerConstants.MOTORS_INVERTED[i]));
            motors[i].getSimState().setMotorType(MotorType.KrakenX60);

            if(i != 0){
                motors[i].setControl(new Follower(IndexerConstants.CAN_IDS[0], MotorAlignmentValue.Aligned));
            }
        }
    }

    /**
     * get the lead motor
     * @return the lead motor
     */
    private TalonFX getLeadMotor(){
        return motors[0];
    }

    @Override
    public void stop() {
        getLeadMotor().stopMotor();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        getLeadMotor().set(dutyCycle);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        
        TalonFXSimState simState = getLeadMotor().getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simState.getMotorVoltage();

        indexerSim.setInputVoltage(motorVoltage);
        indexerSim.update(Constants.LOOP_PERIOD_SECONDS);

        simState.addRotorPosition(
            UnitConversions.RPMtoRotationsPerCycle(
                indexerSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS)
                * IndexerConstants.GEAR_RATIO);
        
        simState.setRotorVelocity(
            indexerSim.getAngularVelocity().times(
                IndexerConstants.GEAR_RATIO
            )
        ); 

        double[] temps = new double[3];

        for (int i = 0; i < temps.length; i++) {
            temps[i] = getLeadMotor().getDeviceTemp().getValueAsDouble();
        }

        inputs.tempCel = temps;

        inputs.speedRPS = getLeadMotor().getVelocity().getValueAsDouble();
    }
}
