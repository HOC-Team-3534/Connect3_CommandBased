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
        flipper.configMotionAcceleration(160.0);
        flipper.configMotionCruiseVelocity(80.0);
        flipper.configMotionSCurveStrength(1);
        flipper.setSelectedSensorPosition(0);

        // }

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Flipper", getPosition());
    }

    private CommandBase changeFlipper(FlipperPosition position) {
        if (testing)
            return Commands.none();
        targetPosition += position.displacement;
        return runOnce(() -> flipper.set(ControlMode.MotionMagic, targetPosition))
                .andThen(Commands.waitUntil(() -> atPosition()));
    }

    public Command flip(boolean checkDown) {
        if (testing)
            return Commands.none();
        return (changeFlipper(FlipperPosition.Up).until(() -> atPosition())
                .andThen(changeFlipper(FlipperPosition.Down)
                        .until(() -> atPosition() || !checkDown)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command flipperVoltage(double percent) {
        return startEnd(() -> flipper.set(percent), () -> flipper.set(0));
    }

    private boolean atPosition() {
        return Math.abs(getPosition() - targetPosition) <= 10;
    }

    public double getPosition() {
        // if (testing)
        // return 0;
        return flipper.getSelectedSensorPosition();
    }

    enum FlipperPosition {
        Down(-462),
        Up(400);

        public double displacement;

        FlipperPosition(double position) {
            this.displacement = position;
        }
    }

}
