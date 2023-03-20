package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerveDrive.SwerveDrive.GridPosition;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.Callable;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    final Translation2d shiftAway = new Translation2d(
            Units.inchesToMeters(14.0 + 12.0) + Constants.Drive.Known.WHEELBASE_METERS / 2.0,
            0);
    final Translation2d shiftSideways = new Translation2d(0, Units.inchesToMeters(22.0));
    final Callable<Pose2d> robotPose;
    final BiConsumer<Pose2d, Double> visionPoseUpdate;
    AprilTagFieldLayout aprilTagFieldLayout;

    final VisionIO io;
    final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(Callable<Pose2d> robotPose, BiConsumer<Pose2d, Double> visionPoseUpdate, VisionIO io) {
        this.io = io;
        this.robotPose = robotPose;
        this.visionPoseUpdate = visionPoseUpdate;
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            switch (DriverStation.getAlliance()) {
                case Blue:
                    aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
                    break;
                case Red:
                    aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                    break;
                case Invalid:

                    break;
            }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        SmartDashboard.putNumber("April Tag Number", 0);
    }

    @Override
    public void periodic() {
        io.updateInputs(aprilTagFieldLayout, inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        var visionData = io.getVisionData();

        if (Constants.EnabledDebugModes.updatePoseWithVisionEnabled && visionData != null) {
            visionPoseUpdate.accept(visionData.pose, visionData.estBotPoseLatencySecs); // TODO switch to timestamp,
                                                                                        // not
                                                                                        // latency
            Logger.getInstance().recordOutput("Vision/Pose", visionData.pose);
            Logger.getInstance().recordOutput("Vision/Timestamp", visionData.estBotPoseTimestampSecs);
            Logger.getInstance().recordOutput("Vision/Latency", visionData.estBotPoseLatencySecs);

            Logger.getInstance().recordOutput("Vision/Timestamp Based Latency Calculated",
                    Timer.getFPGATimestamp() - visionData.estBotPoseTimestampSecs);
        }
    }

    public Pose2d getBotPose() {
        return io.getVisionData().pose;
    }

    public Pose2d getGridPose(GridPosition position) {
        var id = (int) inputs.aprilTagID;
        if (id <= 0)
            return null;
        switch (DriverStation.getAlliance()) {
            case Blue:
                if (!Arrays.asList(6, 7, 8).contains(id))
                    return null;
                break;
            case Red:
                if (!Arrays.asList(1, 2, 3).contains(id))
                    return null;
                break;
            default:
                return null;
        }
        var aprilTag = aprilTagFieldLayout.getTagPose(id).get().getTranslation().toTranslation2d();
        var robotCenter = aprilTag.plus(shiftAway);
        try {
            if (robotPose.call().getTranslation().getDistance(robotCenter) > 2.5)
                return null;
        } catch (Exception e) {
            e.printStackTrace();
        }
        switch (position) {
            case Center:
                break;

            case Left:
                robotCenter = robotCenter.plus(shiftSideways);
                break;

            case Right:
                robotCenter = robotCenter.minus(shiftSideways);
                break;

            default:
                break;
        }
        return new Pose2d(robotCenter, new Rotation2d());
    }
}
