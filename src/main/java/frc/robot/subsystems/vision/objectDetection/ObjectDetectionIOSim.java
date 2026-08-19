package frc.robot.subsystems.vision.objectDetection;

public class ObjectDetectionIOSim implements ObjectDetectionIO{
    public ObjectDetectionIOSim(){

    }


    @Override
    public void updateInputs(ObjectDetectionInputsAutoLogged inputs){ 
        inputs.hasBalls = true;
        inputs.isConnected = true;
    }


}