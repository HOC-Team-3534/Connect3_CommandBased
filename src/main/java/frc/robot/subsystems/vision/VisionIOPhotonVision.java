package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionIOPhotonVision implements VisionIO {
    PhotonCamera camera = new PhotonCamera("3534camera");
    VisionData visionData;
    final Transform3d cameraToRobot = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.5), 0, Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(3), 0));

    @Override
    public void updateInputs(AprilTagFieldLayout layout, VisionIOInputs inputs) {

        // TODO complete work to feed bot pose and april tag and such

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            // Calculate robot's field relative pose
            var robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    layout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            var latency = result.getLatencyMillis() / 1000;
            var timestamp = result.getTimestampSeconds();
            visionData = new VisionData(robotPose.toPose2d(), timestamp, latency);

            inputs.aprilTagID = target.getFiducialId();
        } else {
            inputs.aprilTagID = -1;
            visionData = null;
        }
    }

    @Override
    public VisionData getVisionData() {
        return visionData;
    }
}
