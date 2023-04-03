// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Path;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flipper.Flipper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerveDrive.SwerveDrive;
import frc.robot.subsystems.swerveDrive.SwerveDrive.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end at
   *                 a grid place location
   * @param path2    optional path to follow after placing second piece to go and
   *                 pick up a third piece, MUST start at the same location as
   *                 path1
   * @return autonomous command to place 2 elements and optionally pickup 3rd
   *         piece from one of the sides
   */
  public static Command place2FromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Flipper flipper,
      Path path1, Path path2) {

    var command = moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true, true), intake.shootAuton().withTimeout(0.3));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve, false, 3.0), // was at 3.
          intake.shootAuton(0.8).withTimeout(1.0));
    return command;
  }

  public static Command place2FromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Flipper flipper,
      Path path1, Path path2, double intakeTime) {

    var command = moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper)
        .andThen(driveWithIntake(path1, intake, swerve, true, true), intake.shootAuton(0.7).withTimeout(0.4));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve, false, intakeTime),
          intake.shootAuton(0.8).withTimeout(1.0));
    return command;
  }

  public static Command place2FromBumpSide(SwerveDrive swerve, Intake intake, Elevator elevator, Flipper flipper,
      Path path1, Path path2, double intakeTime) {
    var command = moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper)

        .andThen(driveWithIntake(path1, intake, swerve, true, true), intake.shootAuton(0.7).withTimeout(0.4));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve, false, intakeTime),
          intake.shootAuton(0.8).withTimeout(1.0));
    return command;
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param flipper  the flipper subsystem
   * @param path1    the path to follow after placing the first piece, MUST end
   *                 forward of the charge station across from the center grid
   *                 april tag
   * @return autonomous command to place 1 elements and balance on the charge
   *         station from one of the sides
   */
  public static Command place2andBalanceFromSides(SwerveDrive swerve, Intake intake, Elevator elevator, Flipper flipper,
      Path path1) {
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper).andThen(
        driveWithIntake(path1, intake, swerve, true, true), swerve.balance(Direction.Backward, Direction.Backward)
            .alongWith(Commands.waitSeconds(3.0).andThen(intake.shootAuton(0.5).withTimeout(1.0))));
  }

  public static Command place1AndDriveForwardFromSides(SwerveDrive swerve, Elevator elevator, Flipper flipper,
      Path path1) {
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper)
        .andThen(swerve.driveOnPath(path1, true));
  }

  /**
   * 
   * @param swerve   the swerve drive subsystem
   * @param intake   the intake subsystem
   * @param elevator the elevator subsystem
   * @param flipper  the flipper subsystem
   * @return autonomous command to place 1 elements, cross the charge station, and
   *         balance on the charge station from the center
   */
  public static Command place1andBalanceFromCenter(SwerveDrive swerve, Intake intake, Elevator elevator,

      Flipper flipper) {

    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper)
        .andThen(swerve.balanceAcrossAndBack());
  }

  public static Command place2andBalanceFromCenter(SwerveDrive swerve, Intake intake, Elevator elevator,
      Flipper flipper, Path cubePath) {
    return moveElevatorAndPlace(RobotContainer.getHeightAutonomous(), elevator, flipper).andThen(swerve.driveAcross(),
        swerve.correctToStartAndThenDriveOnPath(cubePath), swerve.balance(Direction.Backward, Direction.Backward),
        intake.shootAuton());
  }

  /**
   * 
   * @param height   the desired height to place the game element
   * @param elevator the elevator subsystem
   * @param flipper  the flipper subsystem
   * @return the autonomomous combo command to moe the elevator to desired height
   *         and then place the game element and bring the elevator back down
   */
  private static Command moveElevatorAndPlace(Height height, Elevator elevator, Flipper flipper) {
    return elevator.goToDesiredHeight(height).withTimeout(3.0).andThen(flipper.flipUp().asProxy().withTimeout(3.0))
        .finallyDo((interrupted) -> elevator.off());
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
  private static Command driveWithIntake(Path path, Intake intake, SwerveDrive swerve, boolean resetToIntial,
      boolean firstRun) {
    if (firstRun)
      return Commands.deadline(swerve.driveOnPath(path, resetToIntial),
          Commands.waitSeconds(1.3).andThen(intake.runIntakeAuton().asProxy().withTimeout(2.0)));
    else
      return Commands.deadline(swerve.driveOnPath(path, resetToIntial),
          intake.runIntakeAuton().asProxy().withTimeout(2.10));
  }

  private static Command driveWithIntake(Path path, Intake intake, SwerveDrive swerve, boolean resetToIntial,
      double time) {
    return Commands.deadline(swerve.driveOnPath(path, resetToIntial),
        intake.runIntakeAuton().asProxy().withTimeout(time));
  }

  // private static Command driveWithIntake(Path path, Intake intake, SwerveDrive
  // swerve, boolean resetToIntial,
  // boolean firstRun, boolean bumpside) {
  // if (firstRun)
  // return Commands.deadline(swerve.driveOnPath(path, resetToIntial),
  // Commands.waitSeconds(1.8).andThen(intake.runIntakeAuton().asProxy().withTimeout(1.8)));
  // else
  // return Commands.deadline(swerve.driveOnPath(path, resetToIntial),
  // intake.runIntakeAuton().asProxy().withTimeout(3.5));
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
