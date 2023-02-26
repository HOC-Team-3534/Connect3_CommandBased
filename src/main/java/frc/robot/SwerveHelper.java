package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drive;
import swerve.SDSModuleConfiguration;
import swerve.SwerveConstants;

public class SwerveHelper {
    public static boolean loadSwerveConstants() {
        var config = Constants.Drive.Known.SDS_MODULE_CONFIGURATION;
        var newConfig = new SDSModuleConfiguration(Units.inchesToMeters(3.81),
                config.angleGearRatio,
                config.driveGearRatio,
                config.angleKP,
                config.angleKI,
                config.angleKD,
                config.angleKF,
                config.driveMotorInvert,
                config.angleMotorInvert,
                config.canCoderInvert);
        SwerveConstants.fillNecessaryConstantsForFalcon(Drive.Calculated.MAX_FWD_REV_SPEED_MPS_EST,
                Drive.Calculated.MAX_ROTATE_SPEED_RAD_PER_SEC_EST, 2 * Math.PI, Drive.Calculated.KINEMATICS, newConfig,
                0.1, 0.293 / 12.0, 2.367 / 12.0, 0.0379 / 12.0, 1.0, 1.0, 1, 1, 0.25, 0.25);
        SwerveConstants.createSwerveConstants();
        SwerveConstants.modulePoseEstXStdDev = 0.1;
        SwerveConstants.modulePoseEstYStdDev = 0.1;
        SwerveConstants.modulePoseEstAngleStdDev = Rotation2d.fromDegrees(0.01);
        SwerveConstants.visionPoseEstXStdDev = 0.15;
        SwerveConstants.visionPoseEstYStdDev = 0.15;
        SwerveConstants.visionPoseEstAngleStdDev = Rotation2d.fromDegrees(0.25);
        return true;
    }
}
