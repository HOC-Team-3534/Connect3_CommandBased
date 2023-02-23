// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.EnabledDebugModes;
import frc.robot.Constants.Drive.Config.DriveCharacterization;
import frc.robot.commands.Autos;
import frc.robot.extras.Limelight;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveDrive;

import java.util.concurrent.Callable;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final Intake intake = new Intake();
	private final Elevator elevator = new Elevator();
	private final Carriage carriage = new Carriage();
	private final Lights lights = new Lights();
	private final Limelight limelight = new Limelight();
	// The driver station connected controllers are defined here...
	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private final SendableChooser<Command> autonChooser = new SendableChooser<>();

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
		autonChooser.setDefaultOption("No Auton", Commands.none());
		autonChooser.addOption("Drive Forward", Autos.driveForward(swerveDrive));
		SmartDashboard.putData(autonChooser);
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
		TGR.DTM.tgr().whileTrue(Commands.parallel(new ProxyCommand(
				() -> {
					return swerveDrive
							.followPathtoGridPose(limelight.getGridPose(swerveDrive.getGridPositionRequest()));
				}),
				elevator.goToDesiredHeight().andThen(carriage.placeElement())));

		TGR.PrepareBalance.tgr().whileTrue(swerveDrive.driveWithDesiredAngle(new Rotation2d()));

		TGR.Characterize.tgr().whileTrue(swerveDrive.characterizeDrive(DriveCharacterization.QUASIASTIC_VOLTAGE,
				DriveCharacterization.QUASIASTIC_DURATION));

		TGR.Intake.tgr().onTrue(carriage.unclamp()).onFalse(carriage.clampElement());
		TGR.Extake.tgr().onTrue(carriage.unclamp());
		TGR.FlipElement.tgr()
				.onTrue(carriage.flipElement().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

		TGR.ResetWithLimelight.tgr().and(() -> !TGR.DTM.bool()).onTrue(new ProxyCommand(() -> {
			return swerveDrive.resetPoseToLimelightPose(limelight.getBotPose());
		}));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autonChooser.getSelected();
	}

	public enum TGR {
		Creep(driverController.rightTrigger(0.15)),
		Intake(driverController.rightBumper()),
		Extake(driverController.leftBumper()), // Subject to Change
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		PrepareBalance(driverController.a().and(() -> !EnabledDebugModes.CharacterizeEnabled)),
		DTM(driverController.leftTrigger(0.15)),
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
		ResetWithLimelight(driverController.leftStick()),
		GridLeft(operatorController.leftBumper()),
		GridRight(operatorController.rightBumper()),
		PlaceMid(operatorController.a()),
		PlaceHigh(operatorController.y()),
		// ClampElement(operatorController.rightTrigger(0.15)),
		FlipElement(operatorController.leftTrigger(0.15));

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger;
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