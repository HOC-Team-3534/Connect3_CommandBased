package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flipper.Flipper;
import frc.robot.subsystems.gripper.Gripper;

public final class CommandCombos {

    public static CommandBase moveElevatorAndPlace(Elevator elevator, Gripper gripper, Flipper flipper) {
        return elevator.goToDesiredHeight()
                .andThen(gripper.ungrip().withTimeout(1.0), flipper.flip())
                .finallyDo((interrupted) -> elevator.off());
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
