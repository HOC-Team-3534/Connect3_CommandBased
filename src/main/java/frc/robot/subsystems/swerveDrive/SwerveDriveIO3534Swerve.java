package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.Config;
import frc.robot.Constants.RobotType;
import swerve.SDSModuleConfiguration;
import swerve.SwerveConstants;
import swerve.SwerveDrivetrainModel;
import swerve.SwerveInput;
import swerve.SwerveModule;

public class SwerveDriveIO3534Swerve implements SwerveDriveIO {
    final static double fl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 86.13
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 273.07 : 134.56;
    final static double fr_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 3.86
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 304.62 : 36.035;
    final static double bl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 274.30
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 130.86 : 174.19;
    final static double br_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 23.90
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 20.39 : 153.37;
    final static boolean loadedConstants = loadSwerveConstants();
    final static WPI_TalonFX FL_drive = new WPI_TalonFX(1);
    final static WPI_TalonFX FL_steer = new WPI_TalonFX(3);
    final static CANCoder FL_cancoder = new CANCoder(2);
    final static SwerveModule fl = new SwerveModule(FL_drive, FL_steer, FL_cancoder,
            Rotation2d.fromDegrees(fl_degrees));
    final static WPI_TalonFX FR_drive = new WPI_TalonFX(4);
    final static WPI_TalonFX FR_steer = new WPI_TalonFX(6);
    final static CANCoder FR_cancoder = new CANCoder(5);
    final static SwerveModule fr = new SwerveModule(FR_drive, FR_steer, FR_cancoder,
            Rotation2d.fromDegrees(fr_degrees));
    final static WPI_TalonFX BL_drive = new WPI_TalonFX(7);
    final static WPI_TalonFX BL_steer = new WPI_TalonFX(9);
    final static CANCoder BL_cancoder = new CANCoder(8);
    final static SwerveModule bl = new SwerveModule(BL_drive, BL_steer, BL_cancoder,
            Rotation2d.fromDegrees(bl_degrees));
    final static WPI_TalonFX BR_drive = new WPI_TalonFX(10);
    final static WPI_TalonFX BR_steer = new WPI_TalonFX(12);
    final static CANCoder BR_cancoder = new CANCoder(11);
    final static SwerveModule br = new SwerveModule(BR_drive, BR_steer, BR_cancoder,
            Rotation2d.fromDegrees(br_degrees));
    final static WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(Config.PIGEON2_ID);
    final static SwerveDrivetrainModel dt = new SwerveDrivetrainModel(fl, fr, bl, br, pigeon2);

    boolean resetThetaController;

    @Override
    public void updateInputs(SwerveDriveIOInputs inputs) {
        inputs.pitchDegs = pigeon2.getPitch();
        inputs.headingDegs = dt.getGyroHeading().getDegrees() % 360;
    }

    @Override
    public void DriveInBrake() {
        FL_drive.setNeutralMode(NeutralMode.Brake);
        FR_drive.setNeutralMode(NeutralMode.Brake);
        BL_drive.setNeutralMode(NeutralMode.Brake);
        BR_drive.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void DriveInCoast() {
        FL_drive.setNeutralMode(NeutralMode.Coast);
        FR_drive.setNeutralMode(NeutralMode.Coast);
        BL_drive.setNeutralMode(NeutralMode.Coast);
        BR_drive.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void updatePoseEstimationWithVision(Pose2d pose, double timestamp) {
        dt.updateOdometryWithVision(pose, timestamp, true);
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean openLoop) {
        dt.setModuleStates(speeds, openLoop);
    }

    @Override
    public void setKnownPose(Pose2d pose) {
        dt.setKnownPose(pose);
    }

    @Override
    public void driveStraight(double percent) {
        dt.setModuleStates(new SwerveInput(percent, 0, 0), new Rotation2d(), false, resetThetaController);
        resetThetaController = false;
    }

    @Override
    public void resetThetaController() {
        resetThetaController = true;
    }

    @Override
    public Command driveOnPath(PathPlannerTrajectory trajectory, SwerveDrive swerveDrive,
            boolean resetToInitial) {
        return dt.createCommandForTrajectory(trajectory, swerveDrive, resetToInitial, true);
    }

    @Override
    public void drive(SwerveInput input, boolean creep, boolean openLoop) {
        dt.setModuleStates(input, creep, openLoop);
    }

    @Override
    public void driveWithFixedAngle(SwerveInput input, Rotation2d rot, boolean creep) {
        dt.setModuleStates(input, rot, creep, resetThetaController);
        resetThetaController = false;
    }

    @Override
    public Command followOTFCommand(Pose2d gridPose, Pose2d currPose, SwerveDrive swerveDrive) {
        return dt.createOnTheFlyPathCommand(gridPose,
                gridPose.getTranslation().minus(currPose.getTranslation()).getAngle(), 0.0,
                AUTO.kMaxSpeedMetersPerSecond, AUTO.kMaxAccelerationMetersPerSecondSquared, swerveDrive);
    }

    @Override
    public Pose2d getPose() {
        return dt.getPose();
    }

    @Override
    public SwerveDrivetrainModel getDriveTrainModel() {
        return dt;
    }

    public static boolean loadSwerveConstants() {
        // TODO determine CBOT characterization values
        double ks = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.293
                : (Constants.ROBOTTYPE == RobotType.PBOT) ? .31 : 0.0;
        double kv = (Constants.ROBOTTYPE == RobotType.TBOT) ? 2.367
                : (Constants.ROBOTTYPE == RobotType.PBOT) ? 2.382 : 0.0;
        double ka = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.0379
                : (Constants.ROBOTTYPE == RobotType.PBOT) ? 0.031 : 0.0;
        var config = Constants.Drive.Known.SDS_MODULE_CONFIGURATION;
        // TODO determine if CBOT wheel diameter differs
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
                Drive.Calculated.MAX_ROTATE_SPEED_RAD_PER_SEC_EST, 2 * Math.PI,
                Drive.Calculated.KINEMATICS, newConfig,
                0.1, ks / 12.0, kv / 12.0, ka / 12.0, 3.0, 3.0, 1, 1, 0.25, 0.25);
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
