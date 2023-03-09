package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;

public final class CommandCombos {

    public static CommandBase moveElevatorAndPlace(Elevator elevator, Gripper gripper, Flipper flipper) {
        return (elevator.goToDesiredHeight()
                .andThen(gripper.ungrip(), flipper.flip()))
                .finallyDo((interrupted) -> elevator.setPowerZero());
    }

    public static Command repositionWithIntake(Intake intake) {
        return Commands.repeatingSequence(intake.runBothBackward(), intake.runBothFast().withTimeout(3.0));
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
