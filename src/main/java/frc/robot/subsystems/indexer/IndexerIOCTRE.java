package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IndexerIOCTRE implements IndexerIO {

    private final TalonFX[] motors; 

    public IndexerIOCTRE(){
        motors = new TalonFX[3];

        for (int i = 0; i < motors.length; i++) {
            motors[i] = new TalonFX(IndexerConstants.CAN_IDS[i]);
            motors[i].getConfigurator().apply(IndexerConstants.getConfig(IndexerConstants.MOTORS_INVERTED[i]));

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
        inputs.speedRPS = getLeadMotor().getVelocity().getValueAsDouble();

        double[] temps = new double[3];

        for (int i = 0; i < temps.length; i++) {
            temps[i] = getLeadMotor().getDeviceTemp().getValueAsDouble();
        }

        inputs.tempCel = temps;

        
    }

}
