package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.Config;
import frc.robot.RobotContainer.AXS;
import frc.robot.RobotContainer.TGR;
import frc.robot.Path;
import frc.robot.SwerveHelper;
import swerve.SwerveDrivetrainModel;
import swerve.SwerveInput;
import swerve.SwerveModule;
import swerve.SwerveSubsystem;

public class SwerveDrive extends SwerveSubsystem {
    final static boolean loadedConstants = SwerveHelper.loadSwerveConstants();
    final static WPI_TalonFX FL_drive = new WPI_TalonFX(1);
    final static WPI_TalonFX FL_steer = new WPI_TalonFX(3);
    final static CANCoder FL_cancoder = new CANCoder(2);
    final static SwerveModule fl = new SwerveModule(FL_drive, FL_steer, FL_cancoder,
            Rotation2d.fromDegrees(86.13));
    final static WPI_TalonFX FR_drive = new WPI_TalonFX(4);
    final static WPI_TalonFX FR_steer = new WPI_TalonFX(6);
    final static CANCoder FR_cancoder = new CANCoder(5);
    final static SwerveModule fr = new SwerveModule(FR_drive, FR_steer, FR_cancoder,
            Rotation2d.fromDegrees(3.86));
    final static WPI_TalonFX BL_drive = new WPI_TalonFX(7);
    final static WPI_TalonFX BL_steer = new WPI_TalonFX(9);
    final static CANCoder BL_cancoder = new CANCoder(8);
    final static SwerveModule bl = new SwerveModule(BL_drive, BL_steer, BL_cancoder,
            Rotation2d.fromDegrees(274.30));
    final static WPI_TalonFX BR_drive = new WPI_TalonFX(10);
    final static WPI_TalonFX BR_steer = new WPI_TalonFX(12);
    final static CANCoder BR_cancoder = new CANCoder(11);
    final static SwerveModule br = new SwerveModule(BR_drive, BR_steer, BR_cancoder,
            Rotation2d.fromDegrees(23.90));
    final static WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(Config.PIGEON2_ID);
    final static SwerveDrivetrainModel dt = new SwerveDrivetrainModel(fl, fr, bl, br, pigeon2);
    double timeCharacterizing;
    boolean resetThetaController;
    final Field2d field = new Field2d();

    public SwerveDrive() {
        super(dt);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        super.periodic();
        field.setRobotPose(dt.getPose());
    }

    public Command driveOnPath(Path path) {
        return driveOnPath(path.getPath());
    }

    public Command driveOnPath(PathPlannerTrajectory trajectory) {
        return dt.createCommandForTrajectory(trajectory, this);
    }

    public Command drive() {
        return run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    AXS.Drive_Rotation.getAxis()), TGR.Creep.bool(), false);
        });
    }

    public Command driveWithDesiredAngle(Rotation2d rot) {
        return Commands.sequence(runOnce(() -> resetThetaController = true), run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    0), rot, TGR.Creep.bool(), resetThetaController);
            resetThetaController = false;
        }));
    }

    public Command followPathtoGridPose(Pose2d gridPose) {
        if (gridPose == null)
            return Commands.none();
        return dt.createOnTheFlyPathCommand(dt.getPose(), dt.getSpeeds(), gridPose,
                gridPose.getTranslation().minus(dt.getPose().getTranslation()).getAngle(), 0.0,
                AUTO.kMaxSpeedMetersPerSecond, AUTO.kMaxAccelerationMetersPerSecondSquared, this);
    }

    public Command resetPoseToLimelightPose(Pose2d pose) {
        if (pose == null)
            return Commands.none();
        return Commands.run(() -> dt.setKnownPose(pose)); // Purposefully not requiring the subystem
    }

    public enum GridPosition {
        Left,
        Center,
        Right
    }

    public GridPosition getGridPositionRequest() {
        var left = TGR.GridLeft.bool();
        var right = TGR.GridRight.bool();
        if (left && right) {
            return GridPosition.Center;
        } else if (left) {
            return GridPosition.Left;
        } else if (right) {
            return GridPosition.Right;
        } else {
            return GridPosition.Center;
        }
    }
}
