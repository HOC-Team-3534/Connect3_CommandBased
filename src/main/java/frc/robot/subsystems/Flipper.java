package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = false;
    WPI_TalonSRX flipper;
    double targetPosition = 0;
    boolean currentControlled = true;

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

    private CommandBase changeFlipper(FlipperPosition position, boolean check) {
        if (testing)
            return Commands.none();
        if (currentControlled) {
            var command = runOnce(() -> flipper.set(position.voltage));
            if (check)
                command = command
                        .andThen(Commands.waitUntil(() -> flipper.getSupplyCurrent() > position.currentShutoff));
            return command;
        }
        var command = runOnce(() -> {
            targetPosition += position.displacement;
            flipper.set(ControlMode.MotionMagic, targetPosition);
        });
        if (check)
            command = command.andThen(Commands.waitUntil(() -> atPosition()));
        return command;
    }

    public Command flip(boolean checkDown) {
        if (testing)
            return Commands.none();
        return (changeFlipper(FlipperPosition.Up, true).withTimeout(0.25)
                .andThen(changeFlipper(FlipperPosition.Down, checkDown).withTimeout(0.25)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
        Down(-0.25, 0.0, -462),
        Up(0.25, 0.0, 400);

        public double voltage, currentShutoff, displacement;

        FlipperPosition(double voltage, double currentShutoff, double position) {
            this.voltage = voltage;
            this.currentShutoff = currentShutoff;
            this.displacement = position;
        }
    }

}
