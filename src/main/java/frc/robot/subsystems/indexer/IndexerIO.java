package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {


    @AutoLog
    public class IndexerInputs{
        double speedRPS;
        /**
         * an array of variables of each motor's temperature
         */
        double[] tempCel;
    }

    /**
     * stops the indexer
     */
    void stop();

    /**
     * makes the indexer spin forward
     * @param dutyCycle percent output of the motor
     */
    void setDutyCycle(double dutyCycle);


    /**
     * update the inputs 
     * @param inputs the inputs object
     */
    void updateInputs(IndexerInputs inputs);
}

