package frc.robot.subsystems.swerveDrive;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotType;
import frc.robot.RobotContainer.AXS;
import frc.robot.RobotContainer.TGR;
import frc.robot.Constants;
import frc.robot.Path;
import swerve.SwerveInput;
import swerve.SwerveSubsystem;

public class SwerveDrive extends SwerveSubsystem {
    final SwerveDriveIO io;
    final SwerveDriveIOInputsAutoLogged inputs = new SwerveDriveIOInputsAutoLogged();

    double timeCharacterizing;
    final ProfiledPIDController xController, yController, thetaController;
    final Function<GridPosition, Pose2d> gridPoseFunction;
    final Callable<Pose2d> loadingPoseCallable;

    public SwerveDrive(SwerveDriveIO io, Function<GridPosition, Pose2d> gp, Callable<Pose2d> lp) {
        super(io.getDriveTrainModel());
        this.io = io;
        this.gridPoseFunction = gp;
        this.loadingPoseCallable = lp;

        xController = new ProfiledPIDController(1.75, 0, 17.5, new TrapezoidProfile.Constraints(3.5, 2.0));
        yController = new ProfiledPIDController(1.75, 0, 17.5, new TrapezoidProfile.Constraints(3.5, 2.0));
        thetaController = new ProfiledPIDController(3, 0, 30,
                new TrapezoidProfile.Constraints(3.0 * Math.PI, 2.0 * Math.PI));
        thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("SwerveDrive", inputs);

        super.periodic();

        var pose = getPose();
        if (pose != null)
            Logger.getInstance().recordOutput("SwerveDrive/Pose", pose);
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
        return runOnce(io::DriveInBrake).andThen(run(() -> {
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

    public Command brake() {
        return Commands.runOnce(io::DriveInBrake);
    }

    public Command coast() {
        return Commands.runOnce(io::DriveInCoast);
    }

    public void updatePoseWithVision(Pose2d pose, double timestamp) {
        io.updatePoseEstimationWithVision(pose, timestamp);
    }

    public double getSlope() {
        return inputs.pitchDegs + ((RobotType.PBOT == Constants.ROBOTTYPE) ? -3.0 : -0.5);
    }

    public boolean isFacingForward() {
        return Math.abs(inputs.headingDegs) < 2.0;
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

    public Command driveOnPath(PathPlannerTrajectory trajectory, boolean resetToInitial) {
        return io.driveOnPath(trajectory, this, resetToInitial);
    }

    /**
     * Drives the robot using forward backward and left right and rotation inputs.
     * Has Creep mode to move the robot at a lower speed
     * 
     * @return The command that moves the robot
     */
    public Command drive() {
        var command = run(() -> {
            var swerveInput = new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    AXS.Drive_Rotation.getAxis());

            io.drive(swerveInput, TGR.Creep.bool(), false);
        });

        command.setName("Drive Manually");
        return command;
    }

    public Command squareUp() {
        // put from -180 to 180
        var ang = inputs.headingDegs;
        if (ang < 45 && ang > -45)
            return driveWithFixedAngle(new Rotation2d());
        else if (ang > 135 || ang < -135)
            return driveWithFixedAngle(Rotation2d.fromDegrees(180));
        else if (ang < 135 && ang > 45)
            return driveWithFixedAngle(Rotation2d.fromDegrees(90));
        else
            return driveWithFixedAngle(Rotation2d.fromDegrees(270));
    }

    /**
     * Drives the robot using forward backward and left right inputs. The robot will
     * rotate to the desired angle. Has Creep mode to move the robot at a lower
     * speed
     *
     * @param rot Desired angle to rotate the robot to
     * @return The command that moves the robot with desired angle
     */
    public Command driveWithFixedAngle(Rotation2d rot) {
        return runOnce(io::resetThetaController).andThen(run(() -> {
            var swerveInput = new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    0);

            io.driveWithFixedAngle(swerveInput, rot, TGR.Creep.bool());
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
        var command = runOnce(io::resetThetaController)
                .andThen(run(() -> {
                    driveStraightWithPower(percent);
                }));

        command.setName("Auton Drive Straight");
        return command;
    }

    private void driveStraightWithPower(double percent) {
        io.driveStraight(percent);
    }

    private double getDistanceFrom(Pose2d pose) {
        return getPose().getTranslation().getDistance(pose.getTranslation());
    }

    public Command DTMFollowToPose() {
        var grid = gridPoseFunction.apply(getGridPositionRequest());
        Pose2d loading = null;
        try {
            loading = loadingPoseCallable.call();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        if (grid != null && getDistanceFrom(grid) < 2.5)
            return followToPose(grid);
        if (loading != null && getDistanceFrom(loading) < 2.5)
            return followToPose(loading);
        return Commands.none();

    }

    private Command followToPose(Pose2d desiredPose) {

        xController.reset(getPose().getX());
        yController.reset(getPose().getY());
        thetaController.reset(getPose().getRotation().getRadians());
        xController.setGoal(desiredPose.getX());
        yController.setGoal(desiredPose.getY());
        thetaController.setGoal(desiredPose.getRotation().getRadians());
        return run(() -> {
            var speeds = new ChassisSpeeds(xController.calculate(getPose().getX()),
                    yController.calculate(getPose().getY()),
                    thetaController.calculate(getPose().getRotation().getRadians()));
            io.setChassisSpeeds(speeds, false);

        });
    }

    public Command stop() {
        return runOnce(() -> io.setChassisSpeeds(new ChassisSpeeds(), true));
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public Command resetPoseToVisionPose(Pose2d pose) {
        if (pose == null)
            return Commands.none();
        return Commands.run(() -> io.setKnownPose(pose)); // Purposefully not requiring the subystem
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
