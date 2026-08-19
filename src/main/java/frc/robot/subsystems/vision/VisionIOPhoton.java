package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.VisionConstants.CamerasConstants;

/**
 * Implementation of VisionIO using PhotonVision cameras.
 * Handles capturing camera frames, estimating robot pose, and updating inputs.
 */
public class VisionIOPhoton implements VisionIO {

    /** Underlying PhotonVision camera interface */
    protected final PhotonCamera camera;
    /** Pose estimator using fiducial targets and field layout */
    private final PhotonPoseEstimator poseEstimator;
    /** Supplier for the last known robot pose, used as reference for pose estimation */
    protected final Supplier<Pose2d> lastPoseSupplier;

    /**
     * Constructs a VisionIOPhoton instance.
     *
     * @param camerasConstants The camera-specific constants (name, strategies, transforms)
     * @param lastPoseSupplier Supplies the robot's last estimated pose for reference
     */
    public VisionIOPhoton(CamerasConstants camerasConstants, Supplier<Pose2d> lastPoseSupplier) {
        this.camera = new PhotonCamera(camerasConstants.CAMERA_NAME);

        // Initialize the pose estimator with main strategy and camera-to-robot transform
        this.poseEstimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT,
                camerasConstants.ROBOT_TO_CAMERA);

        this.lastPoseSupplier = lastPoseSupplier;
    }

    /**
     * Converts a PhotonVision pipeline result into a VisionFrame.
     * Handles computing ambiguity and average target distance safely.
     *
     * @param result The raw pipeline result from PhotonVision
     * @return A VisionFrame encapsulating the estimated pose and additional metadata
     */
    public VisionFrame getNewFrame(PhotonPipelineResult result){
        if(!result.hasTargets()){
            // No targets detected → return empty frame
            return VisionFrame.EMPTY;
        }

        // Estimate robot pose based on detected targets
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
        if(estimatedPose.isEmpty()){
            estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result);
        }
    
        if(estimatedPose.isEmpty()){
            // Pose estimation failed → return empty frame
            return VisionFrame.EMPTY;
        }

        EstimatedRobotPose pose = estimatedPose.get();

        // Initialize default values for ambiguity and average distance
        double ambiguity= 0;
        double avgDistance = 0.0;

        if (!result.getTargets().isEmpty()) {
            // Compute average distance to targets safely
            avgDistance = result.getTargets().stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(0.0);

            // Determine pose ambiguity based on single vs multiple targets
            if (result.getTargets().size() > 1 && result.multitagResult.isPresent()) {
                ambiguity = result.multitagResult.get().estimatedPose.ambiguity;
            } else {
                ambiguity = result.getTargets().get(0).getPoseAmbiguity();
            }
        }

        // Construct the VisionFrame containing all relevant information
        return new VisionFrame(
            true,
            result.getTimestampSeconds(),
            RobotController.getMeasureTime().in(Seconds) - result.getTimestampSeconds(),
            pose.estimatedPose,
            ambiguity,
            avgDistance,
            result.getTargets().size()
        );
    }

    /**
     * Updates the provided VisionInputs object with the latest camera data.
     * Populates visionFrames and targetIDs arrays for further processing.
     *
     * @param inputs The VisionInputs object to populate
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        // Check if the camera is currently connected
        inputs.isConnected = camera.isConnected();
        
        if(!inputs.isConnected){
            // Camera disconnected → populate with empty frame
            inputs.visionFrames = new VisionFrame[]{VisionFrame.EMPTY};
            inputs.targetIDs = new int[0];
            return;
        }

        inputs.pipelineIndex = camera.getPipelineIndex();

        // Retrieve all unread results from the camera
        var result = camera.getAllUnreadResults();
        
        if (result.isEmpty() || !inputs.isEnabled) {
            // No new results → populate with empty frame
            inputs.visionFrames = new VisionFrame[]{VisionFrame.EMPTY};
            inputs.targetIDs = new int[0];
            return;
        }

        // Convert each pipeline result into a VisionFrame
        inputs.visionFrames = result.stream()
            .map(this::getNewFrame)
            .toArray(VisionFrame[]::new);

        // Collect all fiducial IDs from all detected targets
        inputs.targetIDs = result.stream()
            .flatMap(r -> r.getTargets().stream())
            .mapToInt(target -> target.getFiducialId())
            .toArray();


        Logger.recordOutput("VisionIO/Last pose", lastPoseSupplier.get());
    }

    @Override
    public void setPiplineIndex(int index) {
        camera.setPipelineIndex(index);
    }

}