package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class FieldConstants {
    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = field.getFieldLength();
    public static final double fieldWidth = field.getFieldWidth();
}
