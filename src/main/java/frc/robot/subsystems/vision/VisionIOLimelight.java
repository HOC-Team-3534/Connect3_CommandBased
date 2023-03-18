package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(AprilTagFieldLayout layout, VisionIOInputs inputs) {
        var table = NetworkTableInstance.getDefault().getTable("limelight");

        var valid = table.getEntry("tv").getDouble(0.0) > 0;

        if (!valid && RobotBase.isReal()) {
            inputs.aprilTagID = -1;
            inputs.estBotPose = null;
            return;
        }

        inputs.aprilTagID = RobotBase.isSimulation() ? (int) SmartDashboard.getNumber("April Tag Number", 0)
                : (int) table.getEntry("tid").getInteger(0);

        if (inputs.aprilTagID <= 0) {
            inputs.estBotPose = null;
            return;
        }
        double[] botPoseArray;
        switch (DriverStation.getAlliance()) {
            case Blue:
                botPoseArray = (table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]));
                break;

            case Red:
                botPoseArray = (table.getEntry("botpose_wpired").getDoubleArray(new double[7]));
                break;

            default:
                inputs.estBotPose = null;
                return;
        }
        var cameraPoseArray = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        var distanceAway = new Translation2d(cameraPoseArray[0], cameraPoseArray[2]).getNorm();
        SmartDashboard.putNumber("Camera Distance Away from AprilTag", distanceAway);
        if (distanceAway > 2.0)
            inputs.estBotPose = null;
        if (botPoseArray == null || botPoseArray.length < 7)
            inputs.estBotPose = null;
        inputs.estBotPose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));
        var latency = botPoseArray[6] / 1000.0;
        inputs.estBotPoseLatencySecs = latency;
        inputs.estBotPoseTimestampSecs = Timer.getFPGATimestamp() - latency;
    }

}
