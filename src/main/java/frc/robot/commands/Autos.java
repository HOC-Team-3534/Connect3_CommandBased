// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Path;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
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

  // public static CommandBase farLeftPlace2(SwerveDrive swerve, Intake intake,
  // Elevator elevator, Carriage carriage) {
  // return Commands.sequence(elevator.goToDesiredHeight(Height.HIGH),
  // (carriage.placeElement()),
  // (swerve.driveOnPath(Path.Far_Left_Path_Place2)), (intake.runIntakeAuton()),
  // (elevator.goToDesiredHeight(Height.HIGH)), (carriage.placeElement()));//TODO
  // Figure out how to have swerve and intake run in parrallel
  // } This was my first attempt and the one betlow is second attempt

  public static CommandBase farLeftPlace2(SwerveDrive swerve, Intake intake, Elevator elevator, Carriage carriage) {
    return Commands.parallel(elevator.goToDesiredHeight(Height.HIGH).andThen(carriage.placeElement())
        .andThen((swerve.driveOnPath(Path.Far_Left_Path_Place2)), (intake.runIntakeAuton()))
        .andThen((elevator.goToDesiredHeight(Height.HIGH)).andThen((carriage.placeElement()))));
  }

  // TODO figure out how to write them if they need to be parallel or sequence
  // because the swerve drive and intake need to be run parallel but the rest
  // needs to be sequence. May need to make a new class for each auton
  public static CommandBase farLeftPlace2PickUp(SwerveDrive swerve, Intake intake, Elevator elevator,
      Carriage carriage) {
    return Commands.sequence(elevator.goToDesiredHeight(Height.HIGH).andThen(carriage.placeElement())
        .andThen(swerve.driveOnPath(Path.Far_Left_Path_Place2), intake.runIntakeAuton())
        .andThen(elevator.goToDesiredHeight(Height.HIGH).andThen(carriage.placeElement())));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
