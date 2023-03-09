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

    public static CommandBase reorient(Intake intake, Gripper gripper, Flipper flipper) {
        return gripper.ungrip().andThen(flipper.flip(), jiggleAround(intake, gripper));
    }

    public static Command gripOnIntake(Intake intake, Gripper gripper) {
        return gripper.ungrip().andThen(Commands.waitSeconds(1.0), Commands.waitUntil(() -> intake.getAmps() < 15.0),
                Commands.waitUntil(() -> intake.getAmps() > 15.5), Commands.waitSeconds(0.5), gripper.grip());
    }

    public static CommandBase jiggleAround(Intake intake, Gripper gripper) {
        return gripper.ungrip()
                .andThen(intake.runBothSlow().withTimeout(2.0),
                        gripper.grip(false), Commands.waitSeconds(0.5), gripper.ungrip(false),
                        intake.runBottomDownTopUp().withTimeout(2.0), gripper.ungrip(false),
                        intake.runBothSlow());
    }

    private CommandCombos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
