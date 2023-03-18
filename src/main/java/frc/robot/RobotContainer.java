// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.Constants.Drive.Config.DriveCharacterization;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.commands.Autos;
import frc.robot.commands.CommandCombos;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFalcon500;
import frc.robot.subsystems.flipper.Flipper;
import frc.robot.subsystems.flipper.FlipperIO;
import frc.robot.subsystems.flipper.FlipperIOTalonSRX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOTalonSRX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon500s;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIO;
import frc.robot.subsystems.lights.LightsIORevBlinkin;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.robot.subsystems.swerveDrive.SwerveDriveIO;
import frc.robot.subsystems.swerveDrive.SwerveDriveIO3534Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import java.util.concurrent.Callable;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
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
	private static Gripper gripper;
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

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		if (Robot.isSimulation()) {
			swerveDrive = new SwerveDrive(new SwerveDriveIO() {
			});
			intake = new Intake(new IntakeIO() {
			});
			elevator = new Elevator(new ElevatorIO() {
			});
			gripper = new Gripper(new GripperIO() {
			});
			flipper = new Flipper(new FlipperIO() {
			});
			lights = new Lights(new LightsIO() {
			});
			vision = new Vision(swerveDrive::getPose, swerveDrive::updatePoseWithVision, new VisionIO() {
			});
		} else {
			swerveDrive = new SwerveDrive(new SwerveDriveIO3534Swerve());
			intake = new Intake(new IntakeIOFalcon500s());
			elevator = new Elevator(new ElevatorIOFalcon500());
			gripper = new Gripper(new GripperIOTalonSRX());
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
		flipper.setDefaultCommand(flipper.makeSureDown());

		// Autonomous Command Sendable Chooser
		autonChooser.addDefaultOption("No Auton", () -> Commands.none());

		// Autonomous Loading Zone Paths
		autonChooser.addOption("Loading Zone Place 2",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper,
						Path.LoadingZone_PickUp_PlaceSecond,
						null));
		autonChooser.addOption("Loading Zone Place 2 And Pick Up Third",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper,
						Path.LoadingZone_PickUp_PlaceSecond,
						Path.LoadingZone_PickUp_Third));
		autonChooser.addOption("Loading Zone Place Pickup Second and Balance",
				() -> Autos.place1andBalanceFromSides(swerveDrive, intake,
						elevator, gripper, flipper, Path.LoadingZone_PickUp_PlaceWhileMove_BalanceForward));
		autonChooser.addOption("Loading Zone Place 2 While Moving and Balance",
				() -> Autos.place2FromSidesAndBalance(swerveDrive,
						intake, elevator, gripper, flipper, Path.LoadingZone_PickUp_Balance));
		autonChooser.addOption("Loading Zone Place and Drive Forward",
				() -> Autos.place1AndDriveForwardFromSides(swerveDrive, elevator, gripper, flipper,
						Path.LoadingZone_DriveForward));

		// Autonomous Bump Side Paths
		autonChooser.addOption("Bump Side Place 2", () -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper,
				flipper, Path.BumpSide_PickUp_PlaceSecond, null));
		autonChooser.addOption("Bump Side Place 2 Pick Up Third",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper,
						Path.BumpSide_PickUp_PlaceSecond,
						Path.BumpSide_PickUp_Third));
		autonChooser.addOption("Bump Side Place Pickup Second and Balance",
				() -> Autos.place1andBalanceFromSides(swerveDrive, intake,
						elevator, gripper, flipper, Path.BumpSide_PickUp_Balance));
		autonChooser.addOption("Bump Side Place 2 While Moving and Balance",
				() -> Autos.place2FromSidesAndBalance(swerveDrive,
						intake, elevator, gripper, flipper, Path.BumpSide_PickUp_PlaceWhileMove_BalanceForward));
		autonChooser.addOption("Bump Side Place and Drive Forward", () -> Autos
				.place1AndDriveForwardFromSides(swerveDrive, elevator, gripper, flipper, Path.BumpSide_DriveForward));

		// Autonomous Center Paths
		autonChooser.addOption("Center Place Drive Across and Back",
				() -> Autos.place1andBalanceFromCenter(swerveDrive, intake, elevator, gripper, flipper));

		// Autonomous Testing Paths
		// autonChooser.addOption("Test Auto Balance Forward", () ->
		// swerveDrive.balanceForward());
		// autonChooser.addOption("Test Auto Balance Backward", () ->
		// swerveDrive.balanceBackward());
		// autonChooser.addOption("Test Auto Balance Across and Back", () ->
		// swerveDrive.balanceAcrossAndBack());

		// Alert if in tuning mode
		if (Constants.tuningMode) {
			new Alert("Tuning mode active, expect decreased network performance.",
					AlertType.INFO).set(true);
		}

		heightChooser.addDefaultOption("low", Height.LOW);
		heightChooser.addOption("Mid", Height.MID);
		heightChooser.addOption("High", Height.HIGH);
		heightChooser.addOption("OFF", Height.OFF);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
	 * with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
	 * subclasses for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		TGR.DTM.tgr()
				.whileTrue(new ProxyCommand(
						() -> swerveDrive
								.followPIDToGridPose(vision.getGridPose(swerveDrive.getGridPositionRequest())))
						.andThen(CommandCombos.moveElevatorAndPlace(elevator, gripper, flipper)
								.unless(() -> !swerveDrive.isGridPoseValid())));

		// TODO create DTM that aligns so the front can extake

		TGR.PrepareBalance.tgr().whileTrue(new ProxyCommand(() -> swerveDrive.squareUp()));

		TGR.ResetWithLimelight.tgr().onTrue(new ProxyCommand(() -> {
			return swerveDrive.resetPoseToVisionPose(vision.getBotPose());
		}));

		TGR.Ungrip.tgr().whileTrue(gripper.ungrip());
		TGR.Grip.tgr().whileTrue(gripper.grip());

		TGR.PlacePiece.tgr()
				.whileTrue(new ProxyCommand(() -> CommandCombos.moveElevatorAndPlace(elevator, gripper, flipper)));

		TGR.ConeAtStation.tgr().onTrue(elevator.goToDesiredHeight(Height.LOAD))
				.onFalse(elevator.goToDesiredHeight(Height.OFF));

		// The following triggered commands are for debug purposes only
		TGR.Characterize.tgr().whileTrue(swerveDrive.characterizeDrive(DriveCharacterization.QUASIASTIC_VOLTAGE,
				DriveCharacterization.QUASIASTIC_DURATION));
		TGR.PositiveVoltage.tgr().whileTrue(flipper.flipperVoltage(1.0));
		TGR.NegativeVoltage.tgr().whileTrue(flipper.flipperVoltage(-0.65));
		TGR.MoveElevator.tgr().onTrue(elevator.goToDesiredHeight(Height.LOW))
				.onFalse(elevator.goToDesiredHeight(Height.OFF));
		TGR.MoveFlipper.tgr().onTrue(flipper.flip());

		new Trigger(() -> DriverStation.getMatchTime() < 2.0 &&
				Robot.isTeleopEnabled).onTrue(swerveDrive.brake());

	}

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

	public Command getCoastCommand() {
		return swerveDrive.coast();
	}

	public Command getDriveStopCommand() {
		return swerveDrive.stop().ignoringDisable(true);
	}

	public static Height getHeightAutonomous() {
		return heightChooser.get();
	}

	public enum TGR {
		DTM(driverController.leftTrigger(0.15).and(() -> EnabledDebugModes.DTMEnabled)),
		Creep(driverController.leftBumper().or(() -> elevator.getCurrentCommand() != null)),
		PrepareBalance(driverController.a().and(() -> !EnabledDebugModes.CharacterizeEnabled)),
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		ResetWithLimelight(driverController.leftStick().and(() -> !DTM.bool())),

		Intake(driverController.rightTrigger(0.15)),
		Extake(driverController.rightBumper()),
		PlacePiece(driverController.y().and(() -> !DTM.bool())),
		ConeAtStation(driverController.x()),

		Ungrip(operatorController.leftTrigger(0.15)),
		Grip(operatorController.rightTrigger(0.15)),
		CubeLights(operatorController.b()),
		ConeLights(operatorController.x()),
		GridLeft(operatorController.leftBumper()),
		GridRight(operatorController.rightBumper()),
		PlaceMid(operatorController.a()),
		PlaceHigh(operatorController.y()),

		PositiveVoltage(driverController.povUp().and(() -> EnabledDebugModes.testingVoltageControl)),
		NegativeVoltage(driverController.povDown().and(() -> EnabledDebugModes.testingVoltageControl)),
		MoveElevator(driverController.povLeft().and(
				() -> EnabledDebugModes.testingElevatorPos)),
		MoveFlipper(driverController.povRight().and(() -> EnabledDebugModes.testingFlipper));

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
