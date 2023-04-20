// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Arrays;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public static boolean isAutonomous, isTeleopEnabled;

  private final Alert logReceiverQueueAlert = new Alert("Logging queue exceeded capacity, data will NOT be logged.",
      AlertType.ERROR);

  public Robot() {
    super(Constants.LOOP_PERIOD_SECS);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    var logger = Logger.getInstance();
    setUseTiming(Constants.getMode() != Constants.Mode.REPLAY);
    logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    logger.recordMetadata("Robot", Constants.ROBOTTYPE.toString());
    logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
    case 0:
      logger.recordMetadata("GitDirty", "All changes committed");
      break;
    case 1:
      logger.recordMetadata("GitDirty", "Uncomitted changes");
      break;
    default:
      logger.recordMetadata("GitDirty", "Unknown");
      break;
    }

    switch (Constants.getMode()) {
    case REAL:
      logger.addDataReceiver(new WPILOGWriter(Constants.logFolder));
      logger.addDataReceiver(new NT4Publisher());
      LoggedPowerDistribution.getInstance();
      break;

    case SIM:
      logger.addDataReceiver(new NT4Publisher());
      break;

    case REPLAY:
      String path = LogFileUtil.findReplayLog();
      logger.setReplaySource(new WPILOGReader(path));
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
      break;
    }
    logger.start();

    // Load PathPlanner paths/trajectories from their files. Takes some time.
    Arrays.asList(Path.values()).stream().forEach(path -> path.loadPath());
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated
   * and test.
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    isAutonomous = isAutonomous();
    isTeleopEnabled = isTeleopEnabled();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Log scheduled commands
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler", NetworkTableInstance.getDefault()
        .getEntry("/LiveWindow/Ungrouped/Scheduler/Names").getStringArray(new String[] {}));

    // Check logging fault
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

    Threads.setCurrentThreadPriority(true, 10);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getDriveStopCommand().schedule();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.getCoastCommand().schedule();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.getCoastCommand().schedule();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
