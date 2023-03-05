// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.EnabledDebugModes;
import frc.robot.Constants.Drive.Config.DriveCharacterization;
import frc.robot.commands.Autos;
import frc.robot.commands.CommandCombos;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

import java.util.concurrent.Callable;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	private static final SwerveDrive swerveDrive = new SwerveDrive();
	private static final Intake intake = new Intake();
	private static final Elevator elevator = new Elevator();
	private static final Gripper gripper = new Gripper();
	private static final Flipper flipper = new Flipper();
	private static final Lights lights = new Lights();
	private static final Limelight limelight = new Limelight(swerveDrive::getPose, swerveDrive::updatePoseWithVision);
	// The driver station connected controllers are defined here...
	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private static final SendableChooser<Callable<Command>> autonChooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		swerveDrive.setDefaultCommand(swerveDrive.drive());
		intake.setDefaultCommand(intake.runIntake());
		lights.setDefaultCommand(lights.runLights());
		// Autonomous Command Sendable Chooser
		autonChooser.setDefaultOption("No Auton", () -> Commands.none());
		autonChooser.addOption("Drive Forward", () -> Autos.driveForward(swerveDrive));
		autonChooser.addOption("Loading Zone Place 2",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper, Path.Loading_Zone_Place2,
						null));
		autonChooser.addOption("Loading Zone Place 2 Pick Up",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper, Path.Loading_Zone_Place2,
						Path.Loading_Zone_Place2_Pick_Up));
		autonChooser.addOption("Bump Side Place 2", () -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper,
				flipper, Path.Bump_Side_Place2, null));
		autonChooser.addOption("Bump Side Place 2 Pick Up",
				() -> Autos.place2FromSides(swerveDrive, intake, elevator, gripper, flipper, Path.Bump_Side_Place2,
						Path.Bump_Side_Place2_Pick_Up));
		autonChooser.addOption("Test Auto Balance Forward", () -> swerveDrive.balanceForward());
		autonChooser.addOption("Test Auto Balance Backward", () -> swerveDrive.balanceBackward());
		autonChooser.addOption("Test Auto Balance Across and Back", () -> swerveDrive.balanceAcrossAndBack());
		autonChooser.addOption("Loading Zone Place 2 and Balance",
				() -> Autos.place1andBalanceFromSides(swerveDrive, intake,
						elevator, gripper, flipper, Path.Loading_Zone_Place_PickUp_Balance));
		autonChooser.addOption("Loading Zone Place Pick Up and Balance",
				() -> Autos.place2FromSidesAndBalance(swerveDrive,
						intake, elevator, gripper, flipper, Path.Loading_Zone_Place2_Balance));

		SmartDashboard.putData(autonChooser);
		SmartDashboard.putData(swerveDrive);
		SmartDashboard.putData(intake);
		SmartDashboard.putData(lights);
		SmartDashboard.putData(elevator);
		SmartDashboard.putData(gripper);
		SmartDashboard.putData(flipper);
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
								.followPathtoGridPose(limelight.getGridPose(swerveDrive.getGridPositionRequest())))
						.andThen(CommandCombos.moveElevatorAndPlace(elevator, gripper, flipper)
								.unless(() -> !swerveDrive.isGridPoseValid())));

		TGR.PrepareBalance.tgr().whileTrue(swerveDrive.driveWithDesiredAngle(new Rotation2d()));

		TGR.Characterize.tgr().whileTrue(swerveDrive.characterizeDrive(DriveCharacterization.QUASIASTIC_VOLTAGE,
				DriveCharacterization.QUASIASTIC_DURATION));

		TGR.Intake.tgr().onTrue(gripper.ungrip());// .onFalse(gripper.gripper());// TODO see if we need to do a slight
													// wait
													// between gripper and ungripper
		TGR.Extake.tgr().onTrue(gripper.ungrip());
		TGR.FlipElement.tgr()
				.onTrue(CommandCombos.reorient(intake, gripper, flipper));

		TGR.ResetWithLimelight.tgr().onTrue(new ProxyCommand(() -> {
			return swerveDrive.resetPoseToLimelightPose(limelight.getBotPose(false));
		}));

		TGR.Flap.tgr().whileTrue(gripper.flap()).onFalse(gripper.ungrip());

		TGR.PlacePiece.tgr().debounce(0.5).whileTrue(CommandCombos.moveElevatorAndPlace(elevator, gripper, flipper));

		TGR.PositiveVoltage.tgr().whileTrue(gripper.gripperVoltage(0.25));
		TGR.NegativeVoltage.tgr().whileTrue(gripper.gripperVoltage(-0.25));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		try {
			return autonChooser.getSelected().call();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return Commands.none();
	}

	public enum TGR {
		DTM(driverController.leftTrigger(0.15).and(() -> EnabledDebugModes.DTMEnabled)),
		Creep(driverController.leftBumper()),
		Intake(driverController.rightTrigger(0.15)),
		Extake(driverController.rightBumper()), // Subject to Change
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		PrepareBalance(driverController.a().and(() -> !EnabledDebugModes.CharacterizeEnabled)),
		Flap(operatorController.rightTrigger(0.15)),
		PlacePiece(driverController.y().and(() -> !DTM.bool())),

		CubeLights(operatorController.b()), // Sets color
		// to violet
		// indicating
		// cube to be
		// picked
		// up
		ConeLights(operatorController.x()), // Sets color
		// to yellow
		// indicating
		// cone to be
		// picked up
		// ResetPoseToZero(driverController.rightStick()),
		ResetWithLimelight(driverController.leftStick().and(() -> !DTM.bool())),
		GridLeft(operatorController.leftBumper()),
		GridRight(operatorController.rightBumper()),
		PlaceMid(operatorController.a()),
		PlaceHigh(operatorController.y()),
		// GripElement(operatorController.rightTrigger(0.15)),
		FlipElement(operatorController.leftTrigger(0.15)),
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
