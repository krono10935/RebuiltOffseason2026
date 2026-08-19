package frc.robot.subsystems.vision.objectDetection;

import org.photonvision.PhotonCamera;

public class ObjectDetectionIOPhoton implements ObjectDetectionIO{

    private PhotonCamera camera;

    public ObjectDetectionIOPhoton(){
        camera = new PhotonCamera(ObjectDetectionContstants.CAMERA_NAME);
        camera.setPipelineIndex(0);
    }


    @Override
    public void updateInputs(ObjectDetectionInputsAutoLogged inputs){ 
        inputs.isConnected = camera.isConnected();

        if(!inputs.isConnected){
            inputs.hasBalls = true;
            return;
        }

        var results = camera.getAllUnreadResults();
        if(results.isEmpty()){
            return;
        }

        var result = results.get(results.size() -1);


        inputs.hasBalls = false;
        for (var target : result.targets){
            if (target.area > ObjectDetectionContstants.MIN_AREA){
                inputs.hasBalls = true;
                return;
            }
        }
    }
}