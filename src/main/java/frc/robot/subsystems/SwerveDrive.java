package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.Config;
import frc.robot.RobotContainer.AXS;
import frc.robot.RobotContainer.TGR;
import frc.robot.Constants;
import frc.robot.Path;
import frc.robot.SwerveHelper;
import swerve.SwerveDrivetrainModel;
import swerve.SwerveInput;
import swerve.SwerveModule;
import swerve.SwerveSubsystem;

public class SwerveDrive extends SwerveSubsystem {
    // TODO determine CBOT angle offsets
    final static double fl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 86.13
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 273.07 : 134.56;
    final static double fr_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 3.86
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 304.62 : 36.035;
    final static double bl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 274.30
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 130.86 : 174.19;
    final static double br_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 23.90
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 20.39 : 153.37;
    final static boolean loadedConstants = SwerveHelper.loadSwerveConstants();
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
    Pose2d gridPose;
    double timeCharacterizing;
    boolean resetThetaController;
    final Field2d field = new Field2d();
    final ProfiledPIDController xController, yController, thetaController;

    public SwerveDrive() {
        super(dt);

        xController = new ProfiledPIDController(1.75, 0, 17.5, new TrapezoidProfile.Constraints(3.5, 2.0));
        yController = new ProfiledPIDController(1.75, 0, 17.5, new TrapezoidProfile.Constraints(3.5, 2.0));
        thetaController = new ProfiledPIDController(3, 0, 30,
                new TrapezoidProfile.Constraints(3.0 * Math.PI, 2.0 * Math.PI));

        thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        super.periodic();
        field.setRobotPose(dt.getPose());
        SmartDashboard.putNumber("Pitch", getSlope());

    }

    public Command balanceForward() {
        /* height of charge station 9 1/8 inches or ~0.232 meters off the ground */
        var command = (driveStraightAutonomous(0).until(this::isFacingForward))
                .andThen(driveStraightAutonomous(0.25).until(() -> getSlope() < -13.25),
                        driveStraightAutonomous(0.25).until(() -> getSlope() > -13.0),
                        fineTuneBalance());

        command.setName("Balance Forward");
        return command;
    }

    public Command balanceBackward() {
        var command = (driveStraightAutonomous(0).until(this::isFacingForward))
                .andThen(driveStraightAutonomous(-0.25).until(() -> getSlope() > 13.25),
                        driveStraightAutonomous(-0.25).until(() -> getSlope() < 13.0),
                        fineTuneBalance());

        command.setName("Balance Backward");
        return command;
    }

    private Command fineTuneBalance() {
        return runOnce(() -> setMotorBrakeMode()).andThen(run(() -> {
            if (getSlope() > 5)
                driveStraightWithPower(-0.08);
            else if (getSlope() < -5)
                driveStraightWithPower(0.08);
            else
                driveStraightWithPower(0.0);
        }));
    }

    public Command balanceAcrossAndBack() {
        return driveStraightAutonomous(0).until(this::isFacingForward)
                .andThen(driveStraightAutonomous(0.35).until(() -> getSlope() < -12.25),
                        driveStraightAutonomous(0.35).until(() -> getSlope() > 12.0),
                        driveStraightAutonomous(0.35).until(() -> getSlope() < 5),
                        driveStraightAutonomous(0.15).withTimeout(1.5),
                        balanceBackward());
    }

    public SwerveDrivetrainModel getDT() {
        return dt;
    }

    public Command brake() {
        return Commands.runOnce(this::setMotorBrakeMode);
    }

    private void setMotorBrakeMode() {
        FL_drive.setNeutralMode(NeutralMode.Brake);
        FR_drive.setNeutralMode(NeutralMode.Brake);
        BL_drive.setNeutralMode(NeutralMode.Brake);
        BR_drive.setNeutralMode(NeutralMode.Brake);
    }

    public Command coast() {
        return Commands.runOnce(this::setMotorCoastMode);
    }

    private void setMotorCoastMode() {
        FL_drive.setNeutralMode(NeutralMode.Coast);
        FR_drive.setNeutralMode(NeutralMode.Coast);
        BL_drive.setNeutralMode(NeutralMode.Coast);
        BR_drive.setNeutralMode(NeutralMode.Coast);
    }

    public void updatePoseWithVision(Pose2d pose, double latency) {
        dt.updateOdometryWithVision(pose, latency);
    }

    public double getSlope() {
        return pigeon2.getPitch() + ((RobotType.PBOT == Constants.ROBOTTYPE) ? -3.0 : -0.5);
    }

    public boolean isFacingForward() {
        return Math.abs(dt.getGyroHeading().getDegrees() % 360) < 2.0;
    }

    public Command driveOnPath(Path path, boolean resetToInitial, String eventName, Command eventCommand) {
        Map<String, Command> commands = new HashMap<>();
        commands.put(eventName, eventCommand);
        var eventMarkers = path.getPath().getMarkers();
        return new FollowPathWithEvents(driveOnPath(path, resetToInitial), eventMarkers, commands);
    }

    public Command driveOnPath(Path path, boolean resetToInitial) {
        return driveOnPath(path.getPath(), resetToInitial);
    }

    public void setKnownPose(PathPlannerTrajectory trajectory) {
        var initialState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(),
                DriverStation.getAlliance());
        dt.setKnownPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
    }

    public Command driveOnPath(PathPlannerTrajectory trajectory, boolean resetToInitial) {
        return dt.createCommandForTrajectory(trajectory, this, false, true);
    }

    /**
     * Drives the robot using forward backward and left right and rotation inputs.
     * Has Creep mode to move the robot at a lower speed
     * 
     * @return The command that moves the robot
     */
    public Command drive() {
        var command = run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    AXS.Drive_Rotation.getAxis()), TGR.Creep.bool(), false);
        });

        command.setName("Drive Manually");
        return command;
    }

    public Command squareUp() {
        // put from -180 to 180
        var ang = dt.getGyroHeading().rotateBy(new Rotation2d()).getDegrees();
        if (ang < 45 && ang > -45)
            return driveWithDesiredAngle(new Rotation2d());
        else if (ang > 135 || ang < -135)
            return driveWithDesiredAngle(Rotation2d.fromDegrees(180));
        else if (ang < 135 && ang > 45)
            return driveWithDesiredAngle(Rotation2d.fromDegrees(90));
        else
            return driveWithDesiredAngle(Rotation2d.fromDegrees(270));
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
        return runOnce(() -> resetThetaController = true).andThen(run(() -> {
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
        var command = runOnce(() -> resetThetaController = true)
                .andThen(run(() -> {
                    driveStraightWithPower(percent);
                    resetThetaController = false;
                }));

        command.setName("Auton Drive Straight");
        return command;
    }

    private void driveStraightWithPower(double percent) {
        dt.setModuleStates(new SwerveInput(percent, 0, 0), new Rotation2d(), false, resetThetaController);
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
        return dt.createOnTheFlyPathCommand(gridPose,
                gridPose.getTranslation().minus(dt.getPose().getTranslation()).getAngle(), 0.0,
                AUTO.kMaxSpeedMetersPerSecond, AUTO.kMaxAccelerationMetersPerSecondSquared, this);
    }

    public Command followPIDToGridPose(Pose2d gridPose) {
        this.gridPose = gridPose;
        if (gridPose == null)
            return Commands.none();
        xController.reset(getPose().getX());
        yController.reset(getPose().getY());
        thetaController.reset(getPose().getRotation().getRadians());
        xController.setGoal(gridPose.getX());
        yController.setGoal(gridPose.getY());
        thetaController.setGoal(gridPose.getRotation().getRadians());
        return run(() -> {
            dt.setModuleStates(new ChassisSpeeds(xController.calculate(getPose().getX()),
                    yController.calculate(getPose().getY()),
                    thetaController.calculate(getPose().getRotation().getRadians())),
                    false);

        });
    }

    /**
     * Checks to see if there is a valid grid position that limelight picks up
     * 
     * @return Valid grid position
     */
    public boolean isGridPoseValid() {
        return gridPose != null;
    }

    public Pose2d getPose() {
        return dt.getPose();
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
