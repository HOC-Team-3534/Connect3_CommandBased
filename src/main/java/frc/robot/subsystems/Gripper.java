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
    boolean testing = true;
    WPI_TalonSRX gripper;

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
        gripper.configMotionAcceleration(200.0);
        gripper.configMotionCruiseVelocity(100.0);
        gripper.configMotionSCurveStrength(1);
        gripper.setSelectedSensorPosition(0);
        // }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count gripper", getPosition());
    }

    private CommandBase changeGripper(gripperPosition position) {
        if (testing)
            return Commands.none();
        return runOnce(() -> gripper.set(ControlMode.MotionMagic, position.position))
                .andThen(Commands.waitUntil(() -> atPosition(position)));
    }

    public CommandBase grip() {
        if (testing)
            return Commands.none();
        return changeGripper(gripperPosition.Closed);
    }

    public CommandBase ungrip() {
        if (testing)
            return Commands.none();
        return changeGripper(gripperPosition.Open);
    }

    public Command flap() {
        return Commands.repeatingSequence(grip(), Commands.waitSeconds(0.2), ungrip(), Commands.waitSeconds(0.2));
    }

    public Command gripperVoltage(double percent) {
        return startEnd(() -> gripper.set(percent), () -> gripper.set(0));
    }

    private boolean atPosition(gripperPosition position) {
        return Math.abs(getPosition() - position.position) <= 5;
    }

    public double getPosition() {
        // if (testing)
        // return 0;
        return gripper.getSelectedSensorPosition();
    }

    enum gripperPosition {
        Open(0),
        Closed(245);

        public double position;

        gripperPosition(double position) {
            this.position = position;
        }
    }

}
