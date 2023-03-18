package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gripper extends SubsystemBase {
    final GripperIO io;
    final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

    boolean testing = true;

    public Gripper(GripperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Gripper", inputs);
    }

    public Command grip() {
        if (testing)
            return Commands.none();
        return startEnd(() -> set(0.75), () -> set(0));
    }

    public Command ungrip() {
        if (testing)
            return Commands.none();
        return startEnd(() -> set(-0.75), () -> set(0));
    }

    public Command gripperVoltage(double percent) {
        return startEnd(() -> set(percent), () -> set(0));
    }

    private void set(double percent) {
        io.set(percent);
        Logger.getInstance().recordOutput("Gripper/PercentOutput_SetPoint", percent);
    }
}
