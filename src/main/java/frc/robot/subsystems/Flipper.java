package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flipper extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX flipper;
    Encoder flipperEncoder;

    ProfiledPIDController flipperPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    public Flipper() {
        if (!testing) {
            flipper = new WPI_TalonSRX(19);
            flipperEncoder = new Encoder(0, 1);
            flipperEncoder.reset();
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
        return new ProfiledPIDCommand(flipperPID, flipperEncoder::getDistance, () -> position.position,
                (percentOutput, setPoint) -> flipper.set(percentOutput), this);
    }

    public Command flip(boolean checkDown) {
        if (testing)
            return Commands.none();
        return changeFlipper(FlipperPosition.Up).until(() -> atPosition(FlipperPosition.Up))
                .andThen(changeFlipper(FlipperPosition.Down)
                        .until(() -> atPosition(FlipperPosition.Down) || !checkDown));
    }

    private boolean atPosition(FlipperPosition position) {
        return Math.abs(flipperEncoder.getDistance() - position.position) <= 0;
    }

    public double getPosition() {
        if (testing)
            return 0;
        return flipperEncoder.getDistance();
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
