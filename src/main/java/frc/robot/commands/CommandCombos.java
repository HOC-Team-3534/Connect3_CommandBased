package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.subsystems.Clamp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;

public final class CommandCombos {

    public static CommandBase moveElevatorAndPlace(Elevator elevator, Clamp clamp, Flipper flipper) {
        return (elevator.goToDesiredHeight()
                .andThen(clamp.unclamp(), flipper.flip(false)))
                .finallyDo((interrupted) -> elevator.goToDesiredHeight(Height.LOW).initialize());
    }

    public static CommandBase reorient(Intake intake, Clamp clamp, Flipper flipper) {
        return clamp.unclamp().andThen(flipper.flip(true)).andThen(intake.runJustBottomMotor().withTimeout(3.0))
                .andThen(clamp.clamp());
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
