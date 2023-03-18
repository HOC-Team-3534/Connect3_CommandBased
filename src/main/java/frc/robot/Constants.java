// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import swerve.SDSModuleConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static RobotType ROBOTTYPE = RobotType.CBOT;
  public static double LOOP_PERIOD_MILLIS = 20;

  public enum RobotType {
    CBOT,
    PBOT,
    TBOT
  }

  public static final class Drive {
    public static final class Known {
      /**
       * The left-to-right distance between the drivetrain wheels
       * <p>
       * Should be measured from center to center.
       */
      public final static double TRACKWIDTH_METERS = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.578
          : (Constants.ROBOTTYPE == RobotType.PBOT) ? 0.508 : 0.508;
      /**
       * The front-to-back distance between the drivetrain wheels.
       * <p>
       * Should be measured from center to center.
       */
      public final static double WHEELBASE_METERS = (Constants.ROBOTTYPE == RobotType.TBOT) ? 0.578
          : (Constants.ROBOTTYPE == RobotType.PBOT) ? 0.508 : 0.5207;
      public static final SDSModuleConfiguration SDS_MODULE_CONFIGURATION = SDSModuleConfiguration
          .SDSMK4(SDSModuleConfiguration.driveGearRatios.SDSMK4_L2);
      public static final double MAX_DRIVE_MOTOR_RPM = 6380.0; // Falcon500
                                                               // max rpm
    }

    public static final class Calculated {
      public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
          new Translation2d(Drive.Known.TRACKWIDTH_METERS / 2.0,
              Drive.Known.WHEELBASE_METERS / 2.0),
          new Translation2d(Drive.Known.TRACKWIDTH_METERS / 2.0,
              -Drive.Known.WHEELBASE_METERS / 2.0),
          new Translation2d(-Drive.Known.TRACKWIDTH_METERS / 2.0,
              Drive.Known.WHEELBASE_METERS / 2.0),
          new Translation2d(-Drive.Known.TRACKWIDTH_METERS / 2.0,
              -Drive.Known.WHEELBASE_METERS / 2.0));
      // Drivetrain Performance Mechanical limits
      static public final double MAX_FWD_REV_SPEED_MPS_EST = Known.MAX_DRIVE_MOTOR_RPM / 60.0
          / Known.SDS_MODULE_CONFIGURATION.driveGearRatio * Known.SDS_MODULE_CONFIGURATION.wheelCircumference;
      static public final double MAX_ROTATE_SPEED_RAD_PER_SEC_EST = MAX_FWD_REV_SPEED_MPS_EST
          / Math.hypot(Known.TRACKWIDTH_METERS / 2.0, Known.WHEELBASE_METERS / 2.0);
      static public final double MAX_ROTATE_SPEED_RAD_PER_SEC_MOTOR_EST = Known.MAX_DRIVE_MOTOR_RPM / 60
          / Known.SDS_MODULE_CONFIGURATION.angleGearRatio * 2 * Math.PI;
    }

    public static final class Config {
      public static final int PIGEON2_ID = 19;

      public static final class DriveCharacterization {
        public static final double QUASIASTIC_VOLTAGE = 1.0; // voltage
                                                             // per
                                                             // second
                                                             // increase
        public static final double QUASIASTIC_DURATION = 4.5; // duration
                                                              // of
                                                              // test
                                                              // seconds
      }
    }

    public static final class ROBOT {
      public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; // Misc
                                                                 // electronics
      public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; // Nicely
                                                                 // charged
                                                                 // battery
                                                                 // 40mOhm - average batter + cabling
      public static final double BATTERY_NOMINAL_RESISTANCE = 0.040;
      public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage
                                                     // sent
                                                     // to a motor
                                                     // controller
    }

    public static final class AUTO {
      public static final double kMaxSpeedMetersPerSecond = 3.5;
      public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
      public static final double TRAJECTORYXkP = 10;
      public static final double TRAJECTORYYkP = 10;
      public static final double THETACONTROLLERkP = 10;
      // Constraint for the motion profilied robot angle controller
      public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared);
    }
  }

  public static final class ELEVATOR {
    public static final double kElevatorCruiseVelocity = 20000.0;// Counts per 100ms
    public static final double kElevatorAcceleration = 80000.0;// Counts per 100ms per second

    public static enum Height {
      HIGH(210000),
      LOAD(45000),
      MID(130000),
      LOW(45000),
      OFF(0.0);

      public double height;

      Height(double height) {
        this.height = height;
      }
    }
  }

  public static final class EnabledDebugModes {
    public static final boolean CharacterizeEnabled = false;
    public static final boolean DTMEnabled = false;
    public static final boolean updatePoseWithVisionEnabled = true;
    public static final boolean testingVoltageControl = false;
    public static final boolean testingElevatorPos = false;
    public static final boolean testingFlipper = true;
  }
}
