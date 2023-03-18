package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose2d estBotPose;
        public int aprilTagID;
        public double estBotPoseTimestampSecs;
        public double estBotPoseLatencySecs;
    }

    public default void updateInputs(AprilTagFieldLayout layout, VisionIOInputs inputs) {
    }

}
