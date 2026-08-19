package frc.robot.subsystems.vision.objectDetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
    
    @AutoLog
    public class ObjectDetectionInputs{
        boolean isConnected;
        boolean hasBalls;
    }

    public void updateInputs(ObjectDetectionInputsAutoLogged inputs);

    public class ObjectDetectionIOReplay implements ObjectDetectionIO{

        public ObjectDetectionIOReplay(){
            
        }

        @Override
        public void updateInputs(ObjectDetectionInputsAutoLogged inputs){ 
        
        }
    }
}
