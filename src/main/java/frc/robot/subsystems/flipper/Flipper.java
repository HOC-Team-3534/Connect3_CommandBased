package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = true;
    final FlipperIO io;
    final FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

    long lastTimeDownApplied;
    int counter;

    public Flipper(FlipperIO io) {
        this.io = io;
        Logger.getInstance().recordOutput("Flipper/Testing", testing);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Flipper", inputs);
    }

    public Command flip() {
        if (testing)
            return Commands.none();
        return (flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25),
                flipAndCheck(FlipperPosition.Down)))
                .finallyDo((interrupt) -> set(0)).beforeStarting(() -> counter = 3);
    }

    public Command flipUp() {
        return flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .finallyDo((interrupt) -> set(0)).beforeStarting(() -> counter = 3);
    }

    private Command flipAndCheck(FlipperPosition position) {
        return runOnce(() -> set(position.voltage)).andThen(check(position));
    }

    private Command check(FlipperPosition position) {
        return Commands.waitUntil(() -> inputs.currentAmps > position.currentShutoff);
    }

    private Command stopFlipper() {
        return runOnce(() -> set(0));
    }

    public Command makeSureDown() {
        return run(() -> {
            if (System.currentTimeMillis() - lastTimeDownApplied > 1000 && counter > 0) {
                set(FlipperPosition.Down.voltage);
                lastTimeDownApplied = System.currentTimeMillis();
                counter--;
            } else if (inputs.currentAmps > FlipperPosition.Down.currentShutoff)
                set(0);
        });

    }

    public Command flipperVoltage(double percent) {
        return startEnd(() -> set(percent), () -> set(0));
    }

    private void set(double percent) {
        io.set(percent);
        Logger.getInstance().recordOutput("Flipper/PercentOutput_SetPoint", percent);
    }

    enum FlipperPosition {
        Down(-0.65, 3.0),
        Up(0.65, 3.0);

        public double voltage, currentShutoff;

        FlipperPosition(double voltage, double currentShutoff) {
            this.voltage = voltage;
            this.currentShutoff = currentShutoff;
        }
    }

}
