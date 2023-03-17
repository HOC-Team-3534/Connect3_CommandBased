package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX flipper;

    long lastTimeDownApplied;
    int counter;

    public Flipper() {
        flipper = new WPI_TalonSRX(19);
        flipper.configFactoryDefault();
        flipper.setInverted(true);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Flipper Output Current", flipper.getSupplyCurrent());
    }

    public Command flip() {
        if (testing)
            return Commands.none();
        return (flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25),
                flipAndCheck(FlipperPosition.Down)))
                .finallyDo((interrupt) -> flipper.set(0)).beforeStarting(() -> counter = 3);
    }

    public Command flipUp() {
        return flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25))
                .finallyDo((interrupt) -> flipper.set(0)).beforeStarting(() -> counter = 3);
    }

    private Command flipAndCheck(FlipperPosition position) {
        return runOnce(() -> flipper.set(position.voltage)).andThen(check(position));
    }

    private Command check(FlipperPosition position) {
        return Commands.waitUntil(() -> flipper.getSupplyCurrent() > position.currentShutoff);
    }

    private Command stopFlipper() {
        return runOnce(() -> flipper.set(0));
    }

    public Command makeSureDown() {
        return run(() -> {
            if (System.currentTimeMillis() - lastTimeDownApplied > 1000 && counter > 0) {
                flipper.set(FlipperPosition.Down.voltage);
                lastTimeDownApplied = System.currentTimeMillis();
                counter--;
            } else if (flipper.getSupplyCurrent() > FlipperPosition.Down.currentShutoff)
                flipper.set(0);
        });

    }

    public Command flipperVoltage(double percent) {
        return startEnd(() -> flipper.set(percent), () -> flipper.set(0));
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
