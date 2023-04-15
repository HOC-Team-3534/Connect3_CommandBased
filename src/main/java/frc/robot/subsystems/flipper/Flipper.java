package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = false;
    final FlipperIO io;
    final FlipperIOInputsAutoLogged inputs = new FlipperIOInputsAutoLogged();

    boolean isDown;
    boolean level;
    boolean isGripped;

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
        return ungrip().andThen(flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .finallyDo((interrupt) -> set(0)).beforeStarting(() -> level = true));
    }

    public Command flipDown() {
        if (testing)
            return Commands.none();
        return flipAndCheck(FlipperPosition.Down).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .beforeStarting(() -> isDown = true);
    }

    public Command flipDownWithGrip() {
        if (testing)
            return Commands.none();
        return flipAndCheck(FlipperPosition.Down).andThen(grip(), stopFlipper(), Commands.waitSeconds(0.25))
                .beforeStarting(() -> isDown = true);
    }

    public Command makeLevel() {
        return ungrip().andThen(flipDown().andThen(
                flipperVoltage(FlipperPosition.Up.voltage).withTimeout(0.26).beforeStarting(() -> isDown = false)));
    }

    public Command getReadyToPlace() {
        return flipDown().andThen(
                flipperVoltage(FlipperPosition.Up.voltage).withTimeout(0.10).beforeStarting(() -> isDown = false));
    }

    public Command flipDownOrLevel() {
        if (testing)
            return Commands.none();
        if (isDown)
            return makeLevel();
        else if (TGR.ConeLights.bool())
            return flipDownWithGrip();
        else
            return flipDown();
    }

    public Command flipDownGripChoice() {
        if (testing)
            return Commands.none();
        if (TGR.ConeLights.bool())
            return flipDownWithGrip();
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
                flipDown().schedule();
            }
            if (isGripped) {
                gripperVoltage(0.10);
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

    private Command grip() {
        isGripped = true;
        if (testing)
            return Commands.none();
        return runOnce(() -> setGripper(GripperPosition.Gripped.gripVoltage));
    }

    private Command ungrip() {
        isGripped = false;
        if (testing)
            return Commands.none();
        return runOnce(() -> setGripper(GripperPosition.UnGripped.gripVoltage));
    }

    private void setGripper(double ungripped) {
        io.setGripper(ungripped);
    }

    public Command gripperVoltage(double percent) {
        return startEnd(() -> setGripper(percent), () -> setGripper(0));
    }

    enum FlipperPosition {
        Down(-1.0, 10.0), Up(1.0, 10.0);

        public double voltage, currentShutoff;

        FlipperPosition(double voltage, double currentShutoff) {
            this.voltage = voltage;
            this.currentShutoff = currentShutoff;
        }
    }

    enum GripperPosition {
        Gripped(0.3, 0.11), UnGripped(-0.3, 0.25);

        public double gripVoltage, gripCurentShutoff;

        GripperPosition(double voltage, double currentShutoff) {
            this.gripCurentShutoff = currentShutoff;
            this.gripVoltage = voltage;
        }

    }

}
