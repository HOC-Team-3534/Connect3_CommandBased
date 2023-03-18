package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionData {
    final public Pose2d pose;
    final public double estBotPoseTimestampSecs, estBotPoseLatencySecs;

    public VisionData(Pose2d pose, double timestamp, double latency) {
        this.pose = pose;
        this.estBotPoseTimestampSecs = timestamp;
        this.estBotPoseLatencySecs = latency;
    }

}