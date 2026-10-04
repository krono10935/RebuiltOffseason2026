package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.math.UnitConversions;

public class FieldConstants {
    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = field.getFieldLength();
    public static final double fieldWidth = field.getFieldWidth();


    public static class Hub {

        // Dimensions
        public static final double width = UnitConversions.inchesToMeters(47.0);
        public static final double height =
            UnitConversions.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = UnitConversions.inchesToMeters(41.7);
        public static final double innerHeight = UnitConversions.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint =
            new Translation3d(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation3d innerCenterPoint =
            new Translation3d(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                innerHeight);

        public static final Translation2d nearLeftCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
            new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
            new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint =
            new Translation3d(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(4).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation2d oppNearLeftCorner =
            new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner =
            new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner =
            new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner =
            new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(26).get().toPose2d();
        public static final Pose2d farFace =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(20).get().toPose2d();
        public static final Pose2d rightFace =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(18).get().toPose2d();
        public static final Pose2d leftFace =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(21).get().toPose2d();
  }
}
