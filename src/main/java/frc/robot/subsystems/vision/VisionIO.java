package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public long aprilTagID;
    }

    public default void updateInputs(AprilTagFieldLayout layout, VisionIOInputs inputs) {
    }

    public default VisionData getVisionData() {
        return null;
    }

}
