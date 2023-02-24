package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Intake extends SubsystemBase {
    WPI_TalonFX topMotor = new WPI_TalonFX(15);
    WPI_TalonFX botMotor = new WPI_TalonFX(16);

    public Command runIntake() {
        return runEnd(() -> {
            if (TGR.Intake.bool())
                setBothMotors(1.0);
            else if (TGR.Extake.bool())
                setBothMotors(-1.0);
            else
                setBothMotors(0);
        }, () -> setBothMotors(0));
    }

    public Command runIntakeAuton() {
        return startEnd(() -> setBothMotors(1.0), () -> setBothMotors(0));
    }

    private void setBothMotors(double percent) {
        topMotor.set(percent);
        botMotor.set(percent);
    }

    public Command runJustBottomMotor() {
        return run(() -> {
            botMotor.set(1.0);
            topMotor.set(0);
        });
    }
}
