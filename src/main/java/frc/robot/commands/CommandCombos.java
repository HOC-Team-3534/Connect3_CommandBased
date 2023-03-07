package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;

public final class CommandCombos {

    public static CommandBase moveElevatorAndPlace(Elevator elevator, Gripper clamp, Flipper flipper) {
        return (elevator.goToDesiredHeight()
                .andThen(clamp.ungrip(), flipper.flip(false)))
                .finallyDo((interrupted) -> elevator.goToDesiredHeight(Height.OFF).initialize());
    }

    public static CommandBase reorient(Intake intake, Gripper clamp, Flipper flipper) {
        return clamp.ungrip().andThen(flipper.flip(true)).andThen(intake.runJustBottomMotor().withTimeout(3.0))
                .andThen(clamp.grip());
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
