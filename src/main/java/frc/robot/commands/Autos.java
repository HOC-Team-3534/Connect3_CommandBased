// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Path;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param gripper  the gripper subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end
   *                 at a grid place location
   * @param path2    optional path to follow after placing second piece to go and
   *                 pick up a third piece, MUST start at the same location as
   *                 path1
   * @return autonomous command to place 2 elements and optionally pickup 3rd
   *         piece from one of the sides
   */
  public static Command place2FromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Gripper gripper,
      Flipper flipper, Path path1, Path path2) {
    swerve.setKnownPose(path1.getPath());

    var command = moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true))
        .andThen(moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve, false));
    return command;
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param gripper  the gripper subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end
   *                 at the grid side of charge station and have a "place" event
   * @return autonomous command to place 2 elements and optionally pickup 3rd
   *         piece from one of the sides
   */
  public static Command place2FromSidesAndBalance(SwerveDrive swerve, Intake intake, Elevator elevator, Gripper gripper,
      Flipper flipper, Path path1) {
    swerve.setKnownPose(path1.getPath());
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper)
        .andThen(driveWithIntakeWithEvent(path1, intake, swerve, true, "place",
            moveElevatorAndPlace(Height.LOW, elevator, gripper, flipper)))
        .andThen(swerve.balanceForward());
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param gripper  the gripper subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end
   *                 forward of the charge station across from the center grid
   *                 april tag
   * @return autonomous command to place 1 elements and balance on the charge
   *         station from one of the sides
   */
  public static Command place1andBalanceFromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Gripper gripper,
      Flipper flipper, Path path1) {
    swerve.setKnownPose(path1.getPath());
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true), swerve.balanceBackward());
  }

  public static Command place1AndDriveForwardFromSides(SwerveDrive swerve, Elevator elevator, Gripper gripper,
      Flipper flipper, Path path1) {
    swerve.setKnownPose(path1.getPath());
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper)
        .andThen(swerve.driveOnPath(path1, true));
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param gripper  the gripper subsystem
   * @param flipper  the flipper subsystem
   * @return autonomous command to place 1 elements, cross the charge station, and
   *         balance on the charge
   *         station from the center
   */
  public static Command place1andBalanceFromCenter(SwerveDrive swerve, Intake intake, Elevator elevator,
      Gripper gripper,
      Flipper flipper) {

    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, gripper, flipper)
        .andThen(swerve.balanceAcrossAndBack());
  }

  /**
   * 
   * @param height   the desired height to place the game element
   * @param elevator the elevator subsystem
   * @param gripper  the gripper subsystem
   * @param flipper  the flipper subsystem
   * @return the autonomomous combo command to moe the elevator to desired height
   *         and then place the game element and bring the elevator back down
   */
  private static Command moveElevatorAndPlace(Height height, Elevator elevator, Gripper gripper, Flipper flipper) {
    return elevator.goToDesiredHeight(height).withTimeout(3.0)
        .andThen(gripper.ungrip().withTimeout(0.1), flipper.flipUp().asProxy().withTimeout(3.0))
        .finallyDo((interrupted) -> elevator.setPowerZero());
  }

  /**
   * 
   * @param path          the path to follow
   * @param intake        the intake subsystem
   * @param swerve        the swerve drive subsystem
   * @param resetToIntial should the position be reset to the start of the path
   * @param eventName     name of the event from path plannner
   * @param eventCommand  the command to run when the event on the path happens
   * @return the autonomous combo command to drive along a path while turning on
   *         the intake
   */
  private static Command driveWithIntakeWithEvent(Path path, Intake intake, SwerveDrive swerve, boolean resetToIntial,
      String eventName, Command eventCommand) {
    return Commands.deadline(swerve.driveOnPath(path, resetToIntial, eventName, eventCommand), intake.runIntakeAuton());
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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
