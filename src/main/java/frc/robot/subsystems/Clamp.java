package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Clamp extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX clamp;
    Encoder clampEncoder;
    ProfiledPIDController clampPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    public Clamp() {
        if (!testing) {
            clamp = new WPI_TalonSRX(18);
            clampEncoder = new Encoder(2, 3);
            clampEncoder.reset();
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
        return new ProfiledPIDCommand(clampPID, clampEncoder::getDistance, () -> position.position,
                (percentOutput, setPoint) -> clamp.set(percentOutput), this)
                .until(() -> atPosition(position));
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

    private boolean atPosition(ClampPosition position) {
        return Math.abs(clampEncoder.getDistance() - position.position) <= 0;
    }

    public double getPosition() {
        if (testing)
            return 0;
        return clampEncoder.getDistance();
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
