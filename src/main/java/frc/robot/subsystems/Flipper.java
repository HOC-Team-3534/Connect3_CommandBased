package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = false;
    WPI_TalonSRX flipper;
    double targetPosition = 0;
    boolean currentControlled = true;

    long lastTimeDownApplied;
    int counter;

    public Flipper() {
        // if (!testing) {
        flipper = new WPI_TalonSRX(19);
        flipper.configFactoryDefault();
        flipper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
        flipper.setInverted(true);
        flipper.setSensorPhase(true);
        flipper.config_kP(0, 15);
        flipper.config_kI(0, 0);
        flipper.config_kD(0, 150);
        flipper.config_kF(0, 0);
        flipper.configMotionAcceleration(1600.0);
        flipper.configMotionCruiseVelocity(400.0);
        flipper.configMotionSCurveStrength(1);
        flipper.setSelectedSensorPosition(0);

        // }

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Flipper", getPosition());
        SmartDashboard.putNumber("Flipper Output Current", flipper.getSupplyCurrent());
    }

    public Command flip() {
        if (testing)
            return Commands.none();
        return (flipAndCheck(FlipperPosition.Up).andThen(stopFlipper(), Commands.waitSeconds(0.25),
                flipAndCheck(FlipperPosition.Down)))
                .finallyDo((interrupt) -> flipper.set(0)).beforeStarting(() -> counter = 3);
    }

    private Command flipAndCheck(FlipperPosition position) {
        Command command;
        if (currentControlled) {
            command = runOnce(() -> flipper.set(position.voltage));
        } else {
            command = runOnce(() -> {
                targetPosition += position.displacement;
                flipper.set(ControlMode.MotionMagic, targetPosition);
            });
        }
        return command.andThen(check(position));
    }

    private Command check(FlipperPosition position) {
        if (currentControlled) {
            return Commands.waitUntil(() -> flipper.getSupplyCurrent() > position.currentShutoff);
        } else {
            return Commands.waitUntil(() -> atPosition());
        }
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

    private boolean atPosition() {
        return Math.abs(getPosition() - targetPosition) <= 30;
    }

    public double getPosition() {
        // if (testing)
        // return 0;
        return flipper.getSelectedSensorPosition();
    }

    enum FlipperPosition {
        Down(-0.25, 0.6, -462),
        Up(0.25, 0.6, 400);

        public double voltage, currentShutoff, displacement;

        FlipperPosition(double voltage, double currentShutoff, double position) {
            this.voltage = voltage;
            this.currentShutoff = currentShutoff;
            this.displacement = position;
        }
    }

}
