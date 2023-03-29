// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.Constants.Drive.Config.DriveCharacterization;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.commands.Autos;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.flipper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.lights.*;
import frc.robot.subsystems.swerveDrive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import java.util.concurrent.Callable;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private static SwerveDrive swerveDrive;
	private static Intake intake;
	private static Elevator elevator;
	private static Flipper flipper;
	private static Lights lights;
	private static Vision vision;
	// The driver station connected controllers are defined here...
	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private static final LoggedDashboardChooser<Callable<Command>> autonChooser = new LoggedDashboardChooser<>(
			"Auton Command");
	private static final LoggedDashboardChooser<ELEVATOR.Height> heightChooser = new LoggedDashboardChooser<>(
			"Auton Elevator Height");

	static final Field2d field = new Field2d();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		if (Robot.isSimulation()) {
			swerveDrive = new SwerveDrive(new SwerveDriveIO() {});
			intake = new Intake(new IntakeIO() {});
			elevator = new Elevator(new ElevatorIO() {});
			flipper = new Flipper(new FlipperIO() {});
			lights = new Lights(new LightsIO() {});
			vision = new Vision(swerveDrive::getPose, swerveDrive::updatePoseWithVision, new VisionIO() {});
		} else {
			swerveDrive = new SwerveDrive(new SwerveDriveIO3534Swerve());
			intake = new Intake(new IntakeIOFalcon500s());
			elevator = new Elevator(new ElevatorIOFalcon500());
			flipper = new Flipper(new FlipperIOTalonSRX());
			lights = new Lights(new LightsIORevBlinkin());
			vision = new Vision(swerveDrive::getPose, swerveDrive::updatePoseWithVision, new VisionIOLimelight());
		}

		// Configure the trigger bindings
		configureBindings();

		// Set Default Commands for Subsystems
		swerveDrive.setDefaultCommand(swerveDrive.drive());
		intake.setDefaultCommand(intake.runIntake());
		lights.setDefaultCommand(lights.runLights());
		flipper.setDefaultCommand(flipper.conditionallyLevel());

		// Autonomous Command Sendable Chooser
		autonChooser.addDefaultOption("No Auton", () -> Commands.none());

		// Autonomous Loading Zone Paths
		autonChooser.addOption("Loading Zone Place Cone Extake 1", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.LoadingZone_PlaceCone_ExtakeCube, null));
		autonChooser.addOption("Loading Zone Place Cube Extake 1", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.LoadingZone_PlaceCube_ExtakeCube, null));
		autonChooser.addOption("Loading Zone Place Cone Extake 2", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.LoadingZone_PlaceCone_ExtakeCube, Path.LoadingZone_ExtakeThird));
		autonChooser.addOption("Loading Zone Place Cube Extake 2", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.LoadingZone_PlaceCube_ExtakeCube, Path.LoadingZone_ExtakeThird));
		autonChooser.addOption("Loading Zone Place Cone and Balance", () -> Autos.place2andBalanceFromSides(swerveDrive,
				intake, elevator, flipper, Path.LoadingZone_PlaceCone_Balance));
		autonChooser.addOption("Loading Zone Place Cube and Balance", () -> Autos.place2andBalanceFromSides(swerveDrive,
				intake, elevator, flipper, Path.LoadingZone_PlaceCube_Balance));
		autonChooser.addOption("Loading Zone Place Cone and Drive Forward", () -> Autos
				.place1AndDriveForwardFromSides(swerveDrive, elevator, flipper, Path.LoadingZone_DriveForwardCone));
		autonChooser.addOption("Loading Zone Place Cube and Drive Forward ", () -> Autos
				.place1AndDriveForwardFromSides(swerveDrive, elevator, flipper, Path.LoadingZone_DriveForwardCube));

		// Autonomous Bump Side Paths
		autonChooser.addOption("Bump Side Place Cone Extake 1", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.BumpSide_PlaceCone_ExtakeCube, null));
		autonChooser.addOption("Bump Side Place Cone Extake 2", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.BumpSide_PlaceCone_ExtakeCube, Path.BumpSide_ExtakeThird, 2.8));
		autonChooser.addOption("Bump Side Place Cube Extake 1", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.BumpSide_PlaceCube_ExtakeCube, null));
		autonChooser.addOption("Bump Side Place Cube Extake 2", () -> Autos.place2FromSides(swerveDrive, intake,
				elevator, flipper, Path.BumpSide_PlaceCube_ExtakeCube, Path.BumpSide_ExtakeThird, 2.8));
		autonChooser.addOption("Bump Side Place Cone Balance", () -> Autos.place2andBalanceFromSides(swerveDrive,
				intake, elevator, flipper, Path.BumpSide_PlaceCone_Balance));
		autonChooser.addOption("Bump Side Place Cube Balance", () -> Autos.place2andBalanceFromSides(swerveDrive,
				intake, elevator, flipper, Path.BumpSide_PlaceCube_Balance));
		autonChooser.addOption("Bump Side Place Cone and Drive Forward", () -> Autos
				.place1AndDriveForwardFromSides(swerveDrive, elevator, flipper, Path.BumpSide_DriveForwardCone));
		autonChooser.addOption("Bump Side Place Cube and Drive Forward", () -> Autos
				.place1AndDriveForwardFromSides(swerveDrive, elevator, flipper, Path.BumpSide_DriveForwardCube));
		// Autonomous Center Paths
		autonChooser.addOption("Center Place Drive Across and Back",
				() -> Autos.place1andBalanceFromCenter(swerveDrive, intake, elevator, flipper));

		// Autonomous Testing Paths
		// autonChooser.addOption("Test Auto Balance Forward", () ->
		// swerveDrive.balanceForward());
		// autonChooser.addOption("Test Auto Balance Backward", () ->
		// swerveDrive.balanceBackward());
		// autonChooser.addOption("Test Auto Balance Across and Back", () ->
		// swerveDrive.balanceAcrossAndBack());

		// Alert if in tuning mode
		if (Constants.tuningMode) {
			new Alert("Tuning mode active, expect decreased network performance.", AlertType.INFO).set(true);
		}

		heightChooser.addDefaultOption("High", Height.HIGH);
		heightChooser.addOption("Mid", Height.MID);
		heightChooser.addOption("Low", Height.OFF);

		SmartDashboard.putData(field);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {
		TGR.DTM.tgr().whileTrue(new ProxyCommand(swerveDrive::DTMFollowToPose));

		// TODO create DTM that aligns so the front can extake

		TGR.PrepareBalance.tgr().whileTrue(new ProxyCommand(() -> swerveDrive.squareUp()));

		TGR.ResetWithLimelight.tgr().onTrue(new ProxyCommand(() -> {
			return swerveDrive.resetPoseToVisionPose(vision.getBotPose());
		}));

		TGR.PlaceHigh.tgr().onTrue(elevator.goToDesiredHeight(Height.HIGH))
				.onFalse(elevator.goToDesiredHeight(Height.OFF));
		TGR.PlaceMid.tgr().onTrue(elevator.goToDesiredHeight(Height.MID))
				.onFalse(elevator.goToDesiredHeight(Height.OFF));

		TGR.FlipUp.tgr().onTrue(flipper.flipUp());
		TGR.FlipDown.tgr().onTrue(new ProxyCommand(flipper::flipDownOrLevel));

		TGR.ConeAtStation.tgr().onTrue(elevator.goToDesiredHeight(Height.LOAD))
				.onFalse(elevator.goToDesiredHeight(Height.OFF));

		// The following triggered commands are for debug purposes only
		TGR.Characterize.tgr().whileTrue(swerveDrive.characterizeDrive(DriveCharacterization.QUASIASTIC_VOLTAGE,
				DriveCharacterization.QUASIASTIC_DURATION));
		TGR.PositiveVoltage.tgr().whileTrue(flipper.flipperVoltage(1.0));
		TGR.NegativeVoltage.tgr().whileTrue(flipper.flipperVoltage(-1.0));

		new Trigger(() -> DriverStation.getMatchTime() < 2.0 && Robot.isTeleopEnabled).onTrue(swerveDrive.brake());

	}

	public static Pose2d visionGridPose() {

		return vision.getGridPose(swerveDrive.getGridPositionRequest());

	}

	public static Pose2d visionLoadingPose() {
		return vision.getLoadingZonePose();
	}

	public static Field2d getField() { return field; }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		try {
			return autonChooser.get().call();
		} catch (Exception e) {
			e.printStackTrace();
		}
		return Commands.none();
	}

	public Command getCoastCommand() { return swerveDrive.coast(); }

	public Command getDriveStopCommand() { return swerveDrive.stop().ignoringDisable(true); }

	public static Height getHeightAutonomous() { return heightChooser.get(); }

	public enum TGR {
		DTM(driverController.leftTrigger(0.15).and(() -> EnabledDebugModes.DTMEnabled)),
		Creep(driverController.leftBumper().or(() -> elevator.getCurrentCommand() != null)),
		PrepareBalance(driverController.a().and(() -> !EnabledDebugModes.CharacterizeEnabled)),
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		ResetWithLimelight(driverController.leftStick().and(() -> !DTM.bool())),

		Intake(driverController.rightTrigger(0.15)), Extake(driverController.rightBumper()),
		ConeAtStation(driverController.x()),

		FlipUp(operatorController.rightTrigger(0.15)), FlipDown(operatorController.leftTrigger(0.15)),

		CubeLights(operatorController.b()), ConeLights(operatorController.x()),
		GridLeft(operatorController.leftBumper()), GridRight(operatorController.rightBumper()),
		PlaceMid(operatorController.a()), PlaceHigh(operatorController.y()),

		PositiveVoltage(driverController.povUp().and(() -> EnabledDebugModes.testingVoltageControl)),
		NegativeVoltage(driverController.povDown().and(() -> EnabledDebugModes.testingVoltageControl));

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}

		public boolean bool() {
			return trigger.getAsBoolean();
		}
	}

	public enum AXS {
		Drive_ForwardBackward(() -> slewRateLimiterX.calculate(-modifyAxis(driverController.getLeftY()))),
		Drive_LeftRight(() -> slewRateLimiterY.calculate(-modifyAxis(driverController.getLeftX()))),
		Drive_Rotation(() -> slewRateLimiterRotation.calculate(-modifyAxis(driverController.getRightX())));

		Callable<Double> callable;

		AXS(Callable<Double> callable) {
			this.callable = callable;
		}

		public double getAxis() {
			try {
				return callable.call().doubleValue();
			} catch (Exception ex) {
				return 0.0;
			}
		}
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);
		return value;
	}
}
