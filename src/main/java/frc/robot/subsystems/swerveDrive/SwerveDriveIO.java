package frc.robot.subsystems.swerveDrive;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import swerve.SwerveDrivetrainModel;
import swerve.SwerveInput;

public interface SwerveDriveIO {
    @AutoLog
    public static class SwerveDriveIOInputs {
        public double pitchDegs, headingDegs;
        public Pose2d pose;
    }

    public default void updateInputs(SwerveDriveIOInputs inputs) {

    }

    public default SwerveDrivetrainModel getDriveTrainModel() {
        return null;
    }

    public default void DriveInBrake() {
    }

    public default void DriveInCoast() {
    }

    public default void updatePoseEstimationWithVision(Pose2d pose, double latency) {
    }

    public default void setChassisSpeeds(ChassisSpeeds speeds, boolean openLoop) {
    }

    public default void setKnownPose(Pose2d pose) {
    }

    public default void driveStraight(double percent) {
    }

    public default void resetThetaController() {
    }

    public default Command driveOnPath(PathPlannerTrajectory trajectory, SwerveDrive swerveDrive,
            boolean resetToInitial) {
        return null;
    }

    public default void drive(SwerveInput swerveInput, boolean creep, boolean openLoop) {
    }

    public default void driveWithFixedAngle(SwerveInput swerveInput, Rotation2d rot, boolean creep) {
    }

    public default Command followOTFCommand(Pose2d gridPose, Pose2d currPose, SwerveDrive swerveDrive) {
        return null;
    }

}
