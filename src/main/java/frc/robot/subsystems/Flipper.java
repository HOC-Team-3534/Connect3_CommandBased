package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX flipper;

    public Flipper() {
        if (!testing) {
            flipper = new WPI_TalonSRX(19);
            flipper.configFactoryDefault();
            flipper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
            flipper.config_kP(0, 0);
            flipper.config_kI(0, 0);
            flipper.config_kD(0, 0);
            flipper.config_kF(0, 0);
            flipper.configMotionAcceleration(1.0);
            flipper.configMotionCruiseVelocity(1.0);
            flipper.configMotionSCurveStrength(1);

        }

    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Flipper", getPosition());
    }

    private CommandBase changeFlipper(FlipperPosition position) {
        if (testing)
            return Commands.none();
        return runOnce(() -> flipper.set(ControlMode.MotionMagic, position.position))
                .andThen(Commands.waitUntil(() -> atPosition(position)));
    }

    public Command flip(boolean checkDown) {
        if (testing)
            return Commands.none();
        return (changeFlipper(FlipperPosition.Up).until(() -> atPosition(FlipperPosition.Up))
                .andThen(changeFlipper(FlipperPosition.Down)
                        .until(() -> atPosition(FlipperPosition.Down) || !checkDown)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private boolean atPosition(FlipperPosition position) {
        return Math.abs(getPosition() - position.position) <= 0;
    }

    public double getPosition() {
        if (testing)
            return 0;
        return flipper.getSelectedSensorPosition();
    }

    enum FlipperPosition {
        Down(0),
        Up(0);

        public double position;

        FlipperPosition(double position) {
            this.position = position;
        }
    }

}
