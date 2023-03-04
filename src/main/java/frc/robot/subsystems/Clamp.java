package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Clamp extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX clamp;

    public Clamp() {
        if (!testing) {
            clamp = new WPI_TalonSRX(18);
            clamp.configFactoryDefault();
            clamp.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
            clamp.config_kP(0, 0);
            clamp.config_kI(0, 0);
            clamp.config_kD(0, 0);
            clamp.config_kF(0, 0);
            clamp.configMotionAcceleration(1.0);
            clamp.configMotionCruiseVelocity(1.0);
            clamp.configMotionSCurveStrength(1);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Clamp", getPosition());
    }

    private CommandBase changeClamp(ClampPosition position) {
        if (testing)
            return Commands.none();
        return runOnce(() -> clamp.set(ControlMode.MotionMagic, position.position))
                .andThen(Commands.waitUntil(() -> atPosition(position)));
    }

    public CommandBase clamp() {
        if (testing)
            return Commands.none();
        return changeClamp(ClampPosition.Closed);
    }

    public CommandBase unclamp() {
        if (testing)
            return Commands.none();
        return changeClamp(ClampPosition.Open);
    }

    public Command flap() {
        return Commands.repeatingSequence(clamp(), Commands.waitSeconds(0.2), unclamp(), Commands.waitSeconds(0.2));
    }

    private boolean atPosition(ClampPosition position) {
        return Math.abs(getPosition() - position.position) <= 0;
    }

    public double getPosition() {
        if (testing)
            return 0;
        return clamp.getSelectedSensorPosition();
    }

    enum ClampPosition {
        Open(0),
        Closed(0);

        public double position;

        ClampPosition(double position) {
            this.position = position;
        }
    }

}
