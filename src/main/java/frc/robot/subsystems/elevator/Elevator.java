package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.RobotContainer.TGR;

public class Elevator extends SubsystemBase {
    final ElevatorIO io;
    final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    boolean testing = false;
    Height targetHeight = Height.OFF;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);
    }

    public Command goToDesiredHeight(Height height) {
        if (testing)
            return Commands.none();
        Command command = runOnce(() -> targetHeight = height);

        if (height == Height.OFF)
            return command.andThen(runOnce(() -> off()));
        return command.andThen(runOnce(() -> setHeight(height)),
                Commands.waitUntil(() -> isCorrectElevatorHeight()));
    }

    private boolean isCorrectElevatorHeight() {
        return Math.abs(targetHeight.height - inputs.position) < 4000;
    }

    public void off() {
        set(0);
    }

    private void set(double percent) {
        io.set(percent);
        Logger.getInstance().recordOutput("Elevator/PercentOutput_SetPoint", percent);
    }

    private void setHeight(Height height) {
        io.setHeight(height);
        Logger.getInstance().recordOutput("Elevator/Height_SetPoint", height.toString());
    }

    public Command elevatorVoltage(double percent) {
        return startEnd(() -> set(percent), this::off);
    }
}
