
package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
      /**
     * Max height deviation for vision targets (in meters)
     */
    public static final double MAX_HEIGHT_DEV = 0.1;
    /**
     * Max ambiguity for multi-tag targets (0,one hunderd precent sure to 1, random.next)
     */
    public static final double MAX_MULTI_AMBIGUTY = 0.3;
    /**
     * Max ambiguity for single-tag targets (0 to 1)
     */
    public static final double MAX_SINGLE_AMBIGUTY = 0.1;

    public record StdDevsFactors(double xyStdFactor,double thetaStdFactor,double minXyStd,double minThetaStd) {
    }

    public enum PipelineModes {
        DISABLE,
        HIGH,
        LOW
    }


    // enum with all the camera constants
    public enum CamerasConstants {
        // Define the camera constants for the front camera
        SHOOTER_CAMERA(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
            "front_camera",
            new Transform3d(
                new Translation3d(0.315, 0, 0.423),
                new Rotation3d(0, Units.degreesToRadians(-25), 0)
    
            ),
            new StdDevsFactors(0.1,0.3,0.1,0.3),
            new StdDevsFactors(0.15,0.35,0.1,0.3)
            
        );
        
//         SIDE_CAMERA(
//            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
//             "roni",
//             new Transform3d(
//                 new Translation3d(0.317, -0.303, 0.2805),
//                 new Rotation3d(0, 0,Units.degreesToRadians(-90))
//
//             ),
//             new StdDevsFactors(0.25,0.25,0.1,0.3),
//             new StdDevsFactors(0.3,0.3,0.1,0.3)
//
//         );
        
       
        /**
         * The main strategy for the camera
         */
        public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        
       /**
        * The alternate strategy for the camera
        */
        public final PhotonPoseEstimator.PoseStrategy ALTERNATE_STRATEGY;

        /**
         * The name of the camera
         */
        public final String CAMERA_NAME;
        
                /**
                 * The transform from the robot to the camera
                 */ 
                public final Transform3d ROBOT_TO_CAMERA;
        
                public final StdDevsFactors[] stdDevsFactors;
            
        
                // Constructor for the camera constants
                CamerasConstants(
                PhotonPoseEstimator.PoseStrategy alternateStrategy, 
                String cameraName, 
                Transform3d robotToCamera,
                StdDevsFactors... factors) {
                    
        
                    this.ALTERNATE_STRATEGY = alternateStrategy;
        
                    this.CAMERA_NAME = cameraName;

            this.ROBOT_TO_CAMERA = robotToCamera;

            stdDevsFactors = factors;

            if(factors.length < 1){
                throw new IllegalArgumentException("must provide at least 1 std Devs factor");
            }
        }


        
    }
    /**
     * The field layout for the 2026 FRC game "Rebuilt"
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
}
