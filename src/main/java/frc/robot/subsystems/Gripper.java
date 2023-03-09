package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gripper extends SubsystemBase {
    boolean testing = false;
    WPI_TalonSRX gripper;

    boolean wackyControl = true;

    public Gripper() {
        // if (!testing) {
        gripper = new WPI_TalonSRX(18);
        gripper.configFactoryDefault();
        gripper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
        gripper.setInverted(true);
        gripper.setSensorPhase(false);
        gripper.config_kP(0, 12);
        gripper.config_kI(0, 0);
        gripper.config_kD(0, 120);
        gripper.config_kF(0, 0);
        gripper.configMotionAcceleration(1000.0);
        gripper.configMotionCruiseVelocity(400.0);
        gripper.configMotionSCurveStrength(1);
        gripper.setSelectedSensorPosition(0);
        // }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count gripper", getPosition());
    }

    private CommandBase changeGripper(GripperPosition position, boolean check) {
        if (testing)
            return Commands.none();

        if (wackyControl) {
            var command = startEnd(() -> gripper.set(position.percent), () -> gripper.set(0))
                    .withTimeout(position.time);
            if (position == GripperPosition.Closed)
                command = command.until(() -> gripper.getSupplyCurrent() > 2.0);
            return command;
        }

        var command = runOnce(() -> gripper.set(ControlMode.MotionMagic, position.position));
        if (check)
            command = command.andThen(Commands.waitUntil(() -> atPosition(position)));
        return command;
    }

    public void gripManually() {
        gripper.set(ControlMode.MotionMagic, GripperPosition.Closed.position);
    }

    public CommandBase grip(boolean check) {
        if (testing)
            return Commands.none();
        return changeGripper(GripperPosition.Closed, check);
    }

    public Command grip() {
        return grip(true);
    }

    public CommandBase ungrip(boolean check) {
        if (testing)
            return Commands.none();
        return changeGripper(GripperPosition.Open, check);
    }

    public Command ungrip() {
        return ungrip(true);
    }

    public Command flap() {
        return Commands.repeatingSequence(grip(false), Commands.waitSeconds(1.0),
                ungrip(false), Commands.waitSeconds(1.0));
    }

    public Command gripperVoltage(double percent) {
        return startEnd(() -> gripper.set(percent), () -> gripper.set(0));
    }

    private boolean atPosition(GripperPosition position) {
        return Math.abs(getPosition() - position.position) <= 5;
    }

    public double getPosition() {
        // if (testing)
        // return 0;
        return gripper.getSelectedSensorPosition();
    }

    enum GripperPosition {
        Open(0, -0.75, 1.0),
        Closed(245, 0.75, 1.0);

        public double position, percent, time;

        GripperPosition(double position, double percent, double time) {
            this.position = position;
            this.percent = percent;
            this.time = time;
        }
    }

}
