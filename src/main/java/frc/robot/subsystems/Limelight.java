package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.GridPosition;

import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.Callable;
import java.util.function.BiConsumer;

public class Limelight extends SubsystemBase {
	// TODO determine why this shift was is a full wheel base, not a half wheel base
	final Translation2d shiftAway = new Translation2d(
			Units.inchesToMeters(14.0 + 12.0) + Constants.Drive.Known.WHEELBASE_METERS / 2.0,
			0);
	final Translation2d shiftSideways = new Translation2d(0, Units.inchesToMeters(22.0));
	NetworkTable table;
	long lastTimeTableSet = 0;
	HashMap<Integer, Translation2d> aprilTagPositions = new HashMap<>();
	final Callable<Pose2d> robotPose;
	final BiConsumer<Pose2d, Double> visionPoseUpdate;

	public Limelight(Callable<Pose2d> robotBose, BiConsumer<Pose2d, Double> visionPoseUpdate) {
		this.robotPose = robotBose;
		this.visionPoseUpdate = visionPoseUpdate;
		getTable();
		aprilTagPositions.put(1, new Translation2d(39.88,
				272.776).times(0.0254));
		aprilTagPositions.put(2, new Translation2d(39.88,
				206.776).times(0.0254));
		aprilTagPositions.put(3, new Translation2d(39.88,
				140.776).times(0.0254));
		aprilTagPositions.put(6, new Translation2d(39.88,
				174.185).times(0.0254));
		aprilTagPositions.put(7, new Translation2d(39.88,
				108.185).times(0.0254));
		aprilTagPositions.put(8, new Translation2d(39.88,
				42.185).times(0.0254));
		SmartDashboard.putNumber("April Tag Number", 0);
	}

	@Override
	public void periodic() {
		super.periodic();
		getBotPose(false);
	}

	public void getTable() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	public Pose2d getBotPose(boolean check) {
		if (!isValid() && RobotBase.isReal())
			return null;
		if (getAprilTag() <= 0)
			return null;
		double[] botPoseArray;
		switch (DriverStation.getAlliance()) {
			case Blue:
				if (check && !Arrays.asList(6, 7, 8).contains(getAprilTag()))
					return null;
				botPoseArray = (table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]));
				break;

			case Red:
				if (check && !Arrays.asList(1, 2, 3).contains(getAprilTag()))
					return null;
				botPoseArray = (table.getEntry("botpose_wpired").getDoubleArray(new double[7]));
				break;

			case Invalid:
				return null;

			default:
				return null;
		}
		var cameraPoseArray = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
		var distanceAway = new Translation2d(cameraPoseArray[0], cameraPoseArray[2]).getNorm();
		SmartDashboard.putNumber("Camera Distance Away from AprilTag", distanceAway);
		if (distanceAway > 2.0)
			return null;
		if (botPoseArray == null || botPoseArray.length < 7)
			return null;
		var pose = new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));
		var latency = botPoseArray[6] / 1000.0;
		if (Constants.EnabledDebugModes.updatePoseWithVisionEnabled)
			visionPoseUpdate.accept(pose, latency);
		return pose;

	}

	public Pose2d getGridPose(GridPosition position) {
		if (!isValid() && RobotBase.isReal())
			return null;
		if (getAprilTag() <= 0)
			return null;
		Translation2d aprilTag;
		switch (DriverStation.getAlliance()) {
			case Blue:
				if (!Arrays.asList(6, 7, 8).contains(getAprilTag()))
					return null;
				aprilTag = aprilTagPositions.get(getAprilTag());
				break;

			case Red:
				if (!Arrays.asList(1, 2, 3).contains(getAprilTag()))
					return null;
				aprilTag = aprilTagPositions.get(getAprilTag());
				break;

			case Invalid:
				return null;

			default:
				return null;
		}
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

	public int getAprilTag() {
		if (RobotBase.isSimulation())
			return (int) SmartDashboard.getNumber("April Tag Number", 0);
		getTable();
		var tagId = (int) table.getEntry("tid").getInteger(0);
		SmartDashboard.putNumber("April Tag Number", tagId);
		return tagId;
	}

	public boolean isValid() {
		if (System.currentTimeMillis() - lastTimeTableSet > 20) {
			lastTimeTableSet = System.currentTimeMillis();
			getTable();
		}
		return table.getEntry("tv").getDouble(0.0) > 0;
	}
}
