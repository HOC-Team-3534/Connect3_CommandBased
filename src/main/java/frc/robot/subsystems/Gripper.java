package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Gripper extends SubsystemBase {
    boolean testing = true;
    WPI_TalonSRX gripper;

    public Gripper() {
        gripper = new WPI_TalonSRX(18);
        gripper.configFactoryDefault();
        gripper.setInverted(true);
    }

    public Command grip() {
        if (testing)
            return Commands.none();
        return startEnd(() -> gripper.set(0.75), () -> gripper.set(0));
    }

    public Command ungrip() {
        if (testing)
            return Commands.none();
        return startEnd(() -> gripper.set(-0.75), () -> gripper.set(0));
    }

    public Command gripperVoltage(double percent) {
        return startEnd(() -> gripper.set(percent), () -> gripper.set(0));
    }
}
