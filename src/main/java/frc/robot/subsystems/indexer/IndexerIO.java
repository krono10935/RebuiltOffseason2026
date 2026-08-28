package frc.robot.subsystems.indexer;

public interface IndexerIO {

    public class IndexerInputs{
        double speedRPS;
        double tempCel;
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

