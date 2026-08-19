
package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Interface representing a vision camera system.
 * Implementations provide methods to update inputs and convert raw camera data
 * into structured vision frames for the robot code.
 */
public interface VisionIO {

    /**
 * Represents a single "frame" or snapshot of vision data from a camera.
 * Contains the estimated robot pose, target ambiguity, and metadata.
 */
public record VisionFrame(
    /** True if at least one target was detected in this frame */
    boolean hasTarget,

    /** Timestamp when the frame was captured (seconds) */
    double timeStampSeconds,

    /** Latency between capture and processing (seconds) */
    double latency,

    /** Estimated 3D pose of the robot */
    Pose3d targetPose,

    /** Ambiguity/confidence of the pose estimation */
    double targetPoseAmbiguity,

    /** Average distance to all detected targets (meters) */
    double avrageDistanceToTargetsMeters,

    /** Number of fiducial targets detected in this frame */
    int numTargets
) {
    /** Shared "empty" frame to represent no targets or invalid data */
    public static final VisionFrame EMPTY = new VisionFrame(
        false, 0, 0, new Pose3d(), 1, 0, 0
    );
}
    /**
     * Container for auto-logged vision data.
     * @see org.littletonrobotics.junction.AutoLog
     */
    @AutoLog
    public static class VisionInputs {
        /** True if the camera is currently connected */
        public boolean isConnected = false;

        public int pipelineIndex = 0;
        /** Array of recent vision frames captured by this camera */
        public VisionFrame[] visionFrames = new VisionFrame[0];
        /** Array of all detected fiducial target IDs in the frames */
        public int[] targetIDs = new int[0];

        public boolean isEnabled = true;
    }

    /**
     * Updates the provided VisionInputs object with the latest camera data.
     *
     * @param inputs The VisionInputs instance to populate
     */
    public void updateInputs(VisionInputs inputs);
    
    /**
     * @param index the index of what pipeline to use
     */
    public void setPiplineIndex(int index);

}
