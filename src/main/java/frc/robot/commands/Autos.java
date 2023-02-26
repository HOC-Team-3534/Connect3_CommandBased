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
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase driveForward(SwerveDrive swerve) {
    return Commands.sequence(swerve.driveOnPath(Path.Drive_Forward_1));
  }

  public static CommandBase place2(SwerveDrive swerve, Intake intake, Elevator elevator, Clamp clamp,
      Flipper flipper,
      boolean prepPickUp, Path path1, Path path2) {
    var command = moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper)
        .andThen(driveWithIntake(path1, intake, swerve))
        .andThen(moveElevatorAndPlace(Height.HIGH, elevator, clamp, flipper));
    if (path2 != null)
      command = command.andThen(driveWithIntake(path2, intake, swerve).unless(() -> !prepPickUp));
    return command;
  }

  private static CommandBase moveElevatorAndPlace(Height height, Elevator elevator, Clamp clamp, Flipper flip) {
    return elevator.goToDesiredHeight(height).andThen(clamp.unclamp().andThen(flip.flip(false)));
  }

  private static CommandBase driveWithIntake(Path path, Intake intake, SwerveDrive swerve) {
    return Commands.deadline(swerve.driveOnPath(path), intake.runIntakeAuton());
  }

  // TODO figure out how to write them if they need to be parallel or sequence
  // because the swerve drive and intake need to be run parallel but the rest
  // needs to be sequence. May need to make a new class for each auton

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
