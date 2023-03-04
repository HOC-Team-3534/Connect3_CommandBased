// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Path;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command driveForward(SwerveDrive swerve) {
    return Commands.sequence(swerve.driveOnPath(Path.Drive_Forward_1, true));
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param clamp    the clamp subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end
   *                 at a grid place location
   * @param path2    optional path to follow after placing second piece to go and
   *                 pick up a third piece, MUST start at the same location as
   *                 path1
   * @return autonomous command to place 2 elements and optionally pickup 3rd
   *         piece from one of the sides
   */
  public static Command place2FromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Clamp clamp,
      Flipper flipper, Path path1, Path path2) {
    var command = moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true))
        .andThen(moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve, false));
    return command;
  }

  /**
   * 
   * @param swerve    the swerve drive subsystem
   * @param intake    the intake subsystem
   * @param elevator  the elevator subsystem
   * @param clamp     the clamp subsystem
   * @param flipper   the flipper subsystem
   * @param limelight the limelight subsystem
   * @param path1     the path to follow after placing the first piece, MUST end
   *                  forward of the charge station across from the center grid
   *                  april tag
   * @return autonomous command to place 1 elements and balance on the charge
   *         station from one of the sides
   */
  public static Command place1andBalanceFromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Clamp clamp,
      Flipper flipper, Limelight limelight, Path path1) {
    return moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true), swerve.balanceBackward(limelight));
  }

  /**
   * 
   * @param swerve    the swerve drive subsystem
   * @param intake    the intake subsystem
   * @param elevator  the elevator subsystem
   * @param clamp     the clamp subsystem
   * @param flipper   the flipper subsystem
   * @param limelight the limelight subsystem
   * @return autonomous command to place 1 elements, cross the charge station, and
   *         balance on the charge
   *         station from the center
   */
  public static Command place1andBalanceFromCenter(SwerveDrive swerve, Intake intake, Elevator elevator, Clamp clamp,
      Flipper flipper, Limelight limelight) {
    return moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper).andThen(swerve.balanceAcrossAndBack(limelight));
  }

  /**
   * 
   * @param height   the desired height to place the game element
   * @param elevator the elevator subsystem
   * @param clamp    the clamp subsystem
   * @param flipper  the flipper subsystem
   * @return the autonomomous combo command to moe the elevator to desired height
   *         and then place the game element and bring the elevator back down
   */
  private static Command moveElevatorAndPlace(Height height, Elevator elevator, Clamp clamp, Flipper flipper) {
    return elevator.goToDesiredHeight(height).andThen(clamp.unclamp(), flipper.flip(false))
        .finallyDo((interrupted) -> elevator.goToDesiredHeight(Height.LOW).initialize());
  }

  /**
   * 
   * @param path          the path to follow
   * @param intake        the intake subsystem
   * @param swerve        the swerve drive subsystem
   * @param resetToIntial should the position be reset to the start of the path
   * @return the autonomous combo command to drive along a path while turning on
   *         the intake
   */
  private static Command driveWithIntake(Path path, Intake intake, SwerveDrive swerve, boolean resetToIntial) {
    return Commands.deadline(swerve.driveOnPath(path, resetToIntial), intake.runIntakeAuton());
  }

  // TODO figure out how to write them if they need to be parallel or sequence
  // because the swerve drive and intake need to be run parallel but the rest
  // needs to be sequence. May need to make a new class for each auton

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
