package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class VisionIOPhotonVision implements VisionIO {
        PhotonCamera camera = new PhotonCamera("3534camera");
        VisionData visionData;
        Field2d field = new Field2d();

        private final LoggedTunableNumber x_offset = new LoggedTunableNumber(
                        "Vision/x_offset", Units.inchesToMeters(-12.5));
        private final LoggedTunableNumber y_offset = new LoggedTunableNumber(
                        "Vision/y_offset", 0);
        private final LoggedTunableNumber z_offset = new LoggedTunableNumber(
                        "Vision/x_offset", Units.inchesToMeters(8));

        private final LoggedTunableNumber roll_offset = new LoggedTunableNumber(
                        "Vision/roll_offset", 0);
        private final LoggedTunableNumber pitch_offset = new LoggedTunableNumber(
                        "Vision/pitch_offset", Units.degreesToRadians(3));
        private final LoggedTunableNumber yaw_offset = new LoggedTunableNumber(
                        "Vision/yaw_offset", 0);

        Transform3d cameraToRobot = new Transform3d(
                        new Translation3d(x_offset.get(), y_offset.get(), z_offset.get()),
                        new Rotation3d(roll_offset.get(), pitch_offset.get(), yaw_offset.get()));

        private final LoggedTunableNumber distanceFromTag = new LoggedTunableNumber(
                        "Vision/DistanceFromTagToCenterRobotMeters", Units.inchesToMeters(43.0));

        final double lengthOfField = 16.542;

        public VisionIOPhotonVision() {
                SmartDashboard.putData(field);
        }

        @Override
        public void updateInputs(AprilTagFieldLayout layout, VisionIOInputs inputs) {

                checkForChangesToCameraTuning();

                // TODO complete work to feed bot pose and april tag and such

                var result = camera.getLatestResult();
                if (result.hasTargets()) {
                        var target = result.getBestTarget();
                        // Calculate robot's field relative pose

                        var tagPose = layout.getTagPose(target.getFiducialId()).get();

                        var robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                                        tagPose, cameraToRobot);
                        var latency = result.getLatencyMillis() / 1000;
                        var timestamp = result.getTimestampSeconds();
                        visionData = new VisionData(robotPose.toPose2d(), timestamp, latency);

                        inputs.aprilTagID = target.getFiducialId();

                        Logger.getInstance().recordOutput("Vision/RawRobotPose3d", robotPose);

                        System.arraycopy(inputs.pose, 0, getPoseArray(robotPose), 0, 6);

                        var distanceTransform = (tagPose.getX() < lengthOfField / 2) ? distanceFromTag.get()
                                        : -distanceFromTag.get();
                        var transformTagToCalibrationPosition = new Transform3d(
                                        new Translation3d(distanceTransform, 0, -tagPose.getZ()),
                                        new Rotation3d());
                        var calibrationDesiredPose = tagPose
                                        .transformBy(transformTagToCalibrationPosition);
                        var poseError = robotPose.relativeTo(calibrationDesiredPose);
                        Logger.getInstance().recordOutput("Vision/CalibrationPoseError", poseError);

                        System.arraycopy(inputs.callibrationPoseError, 0, getPoseArray(poseError), 0, 6);

                        /**
                         * This Calibration Error value assumes you are calibrating from straight in
                         * front of the april tag you are looking at
                         * where the distance between the center of the robot and the april tag is set
                         * by the tunable number "DistanceFromTagToCenterRobotMeters"
                         */

                } else {
                        inputs.aprilTagID = -1;
                        visionData = null;
                }
        }

        @Override
        public VisionData getVisionData() {
                return visionData;
        }

        private void checkForChangesToCameraTuning() {
                if (!Constants.tuningMode)
                        return;
                if (x_offset.hasChanged() || y_offset.hasChanged() || z_offset.hasChanged() || roll_offset.hasChanged()
                                || pitch_offset.hasChanged() || yaw_offset.hasChanged()) {
                        cameraToRobot = new Transform3d(
                                        new Translation3d(x_offset.get(), y_offset.get(), z_offset.get()),
                                        new Rotation3d(roll_offset.get(), pitch_offset.get(), yaw_offset.get()));
                }
        }

        public static double[] getPoseArray(Pose3d pose) {
                double[] poseArray = new double[6];
                poseArray[0] = pose.getX();
                poseArray[1] = pose.getY();
                poseArray[2] = pose.getZ();
                poseArray[3] = pose.getRotation().getX();
                poseArray[4] = pose.getRotation().getY();
                poseArray[5] = pose.getRotation().getZ();
                return poseArray;
        }
}
