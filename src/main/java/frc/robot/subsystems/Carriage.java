package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class Carriage extends SubsystemBase {
    WPI_TalonSRX clamp = new WPI_TalonSRX(18);
    WPI_TalonSRX flipper = new WPI_TalonSRX(19);

    public Command clampElement() {
        return runOnce(() -> clamp.set(0.5)).andThen(Commands.waitSeconds(0.5)).andThen(runOnce(() -> clamp.set(0)));
    }

    public Command unclamp() {
        return runOnce(() -> clamp.set(-0.5)).andThen(Commands.waitSeconds(0.5)).andThen(runOnce(() -> clamp.set(0)));
    }

    public Command flipElement() {
        return Commands.sequence(unclamp(), flip(), clampElement());
    }

    private Command flip() {
        return Commands.sequence(runOnce(() -> flipper.set(0.5)), Commands.waitSeconds(0.5),
                runOnce(() -> flipper.set(-0.5)), Commands.waitSeconds(0.5), runOnce(() -> flipper.set(0)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command placeElement() {
        return Commands.sequence(unclamp(), flip());
    }
}
