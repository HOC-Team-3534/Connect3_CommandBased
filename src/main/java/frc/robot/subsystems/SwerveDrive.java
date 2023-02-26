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
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    Pose2d gridPose;
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
        // TODO update POSE using limelight vision
        field.setRobotPose(dt.getPose());
        SmartDashboard.putNumber("Pitch", getSlope());
    }

    public Command balance() {
        return driveStraightAutonomous(0).until(this::isFacingForward)
                .andThen(driveStraightAutonomous(0.15).until(() -> getSlope() < -2))
                .andThen(Commands.print("I'm going on a trip in my favorite rocket ship"))
                .andThen(driveStraightAutonomous(0.15).until(() -> getSlope() > -2))
                .andThen(Commands.waitSeconds(0.5000))
                .andThen(driveStraightAutonomous(-0.15).until(() -> getSlope() < 2))
                .andThen(driveStraightAutonomous(0)).andThen(Commands.print("Yo! I am finished, brother!"));
    }

    public double getSlope() {
        return pigeon2.getPitch();
    }

    public boolean isFacingForward() {
        return Math.abs(dt.getGyroHeading().getDegrees() % 360) < 2.0;
    }

    public Command driveOnPath(Path path) {
        return driveOnPath(path.getPath());
    }

    public Command driveOnPath(PathPlannerTrajectory trajectory) {
        return dt.createCommandForTrajectory(trajectory, this);
    }

    /**
     * Drives the robot using forward backward and left right and rotation inputs.
     * Has Creep mode to move the robot at a lower speed
     * 
     * @return The command that moves the robot
     */
    public Command drive() {
        return run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    AXS.Drive_Rotation.getAxis()), TGR.Creep.bool(), false);
        });
    }

    /**
     * Drives the robot using forward backward and left right inputs. The robot will
     * rotate to the desired angle. Has Creep mode to move the robot at a lower
     * speed
     *
     * @param rot Desired angle to rotate the robot to
     * @return The command that moves the robot with desired angle
     */
    public Command driveWithDesiredAngle(Rotation2d rot) {
        return Commands.sequence(runOnce(() -> resetThetaController = true), run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    0), rot, TGR.Creep.bool(), resetThetaController);
            resetThetaController = false;
        }));
    }

    /**
     * Drives the robot in autonomous mode for balancing on the charge station using
     * a percent output to drive onto it. The rotation of the robot aligns to zero
     * and drives straight
     * 
     * @param percent The percent the robot will drive straight forward
     * @return The command that moves the robot in a straight path
     */
    public Command driveStraightAutonomous(double percent) {
        return Commands.sequence(runOnce(() -> resetThetaController = true), run(() -> {
            dt.setModuleStates(new SwerveInput(percent,
                    0,
                    0), new Rotation2d(), false, resetThetaController);
            resetThetaController = false;
        }));
    }

    /**
     * Follows an autonomously generated path to a specific grid position in which
     * the operator chooses generates a path on the fly and drives the robot in DTM
     * to the position
     * 
     * @param gridPose The grid Position the robot will generate a path to(Left
     *                 Right or Center)
     * @return The command that the robot will use to autonomously follow in DTM to
     *         a grid position
     */

    public Command followPathtoGridPose(Pose2d gridPose) {
        this.gridPose = gridPose;
        if (gridPose == null)
            return Commands.print("Grid Pose Null");
        return dt.createOnTheFlyPathCommand(dt.getPose(), dt.getSpeeds(), gridPose,
                gridPose.getTranslation().minus(dt.getPose().getTranslation()).getAngle(), 0.0,
                AUTO.kMaxSpeedMetersPerSecond, AUTO.kMaxAccelerationMetersPerSecondSquared, this);
    }

    /**
     * Checks to see if there is a valid grid position that limelight picks up
     * 
     * @return Valid grid position
     */
    public boolean isGridPoseValid() {
        return gridPose != null;
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
