package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = false;
    final FlipperIO io;
    final FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

    boolean isDown;
    boolean level;

    public Flipper(FlipperIO io) {
        this.io = io;
        Logger.getInstance().recordOutput("Flipper/Testing", testing);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Flipper", inputs);
    }

    public Command flipUp() {
        if (testing)
            return Commands.none();
        return flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .finallyDo((interrupt) -> set(0)).beforeStarting(() -> level = true);
    }

    public Command flipDown() {
        if (testing)
            return Commands.none();
        return flipAndCheck(FlipperPosition.Down).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .beforeStarting(() -> isDown = true);
    }

    public Command makeLevel() {
        return flipDown().andThen(
                flipperVoltage(FlipperPosition.Up.voltage).withTimeout(0.26).beforeStarting(() -> isDown = false));
    }

    public Command flipDownOrLevel() {
        if (testing)
            return Commands.none();
        if (isDown)
            return makeLevel();
        else
            return flipDown();
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

    public Command conditionallyLevel() {
        return run(() -> {
            if (level) {
                level = false;
                makeLevel().schedule();
            }
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
        Down(-1.0, 10.0), Up(1.0, 10.0);

        public double voltage, currentShutoff;

        FlipperPosition(double voltage, double currentShutoff) {
            this.voltage = voltage;
            this.currentShutoff = currentShutoff;
        }
    }

}
