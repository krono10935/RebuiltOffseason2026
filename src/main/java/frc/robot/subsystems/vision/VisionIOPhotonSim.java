package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CamerasConstants;

/**
 * A simulated version of the VisionIOPhoton class
 * Uses PhotonVision's simulation tools to replicate camera behavior
 * for use in the FRC simulator.
 */
public class VisionIOPhotonSim extends VisionIOPhoton {

    // Simulation of the whole vision system (AprilTags, cameras, etc.)
    VisionSystemSim visionSim;

    // Simulated camera
    PhotonCameraSim cameraSim;

    /**
     * Constructor for the simulated Vision I/O
     * @param camConst - constants/config for this camera
     * @param lastPoseSupplier - function to provide the robot's latest pose
     */
    public VisionIOPhotonSim(CamerasConstants camConst, Supplier<Pose2d> lastPoseSupplier) {
        // Call the parent constructor
        super(camConst, lastPoseSupplier);  

        // Create a new VisionSystemSim if it doesn't exist yet
        if(visionSim == null){
            visionSim = new VisionSystemSim("main");
            // Add AprilTags to the simulation using the field layout
            visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        }

        // Define simulated camera properties (resolution, FOV, latency, etc.)
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(89.5)); // resolution + diagonal FOV
        cameraProp.setAvgLatencyMs(35); // simulate network/processing delay (tunable)
        
        // Create a simulated PhotonCamera based on our properties
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addCamera(cameraSim,camConst.ROBOT_TO_CAMERA);

        // Enable the raw and processed streams. These are enabled by default.
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        cameraSim.enableDrawWireframe(true);

    }
    /**
     * Updates the simulated vision system
     * @param inputs - the data structure where vision results will be stored
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        // Update the vision simulation with the robot's latest pose
        visionSim.update(lastPoseSupplier.get());

        // Call the parent class to handle the rest of the data update
        super.updateInputs(inputs);
    }

    /**
     * @return the camera's name (from its configuration)
     */
    public String getName(){
        return camera.getName();
    }
}