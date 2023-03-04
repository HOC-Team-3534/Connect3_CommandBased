package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Intake extends SubsystemBase {
    WPI_TalonFX topMotor, botMotor;
    boolean testing = true;

    public Intake() {
        if (!testing) {
            topMotor = new WPI_TalonFX(15);
            botMotor = new WPI_TalonFX(16);
            topMotor.configFactoryDefault();
            botMotor.configFactoryDefault();
            topMotor.setInverted(true);
            botMotor.setInverted(true);
        }
    }

    public Command runIntake() {
        if (testing)
            return Commands.none();
        return runEnd(() -> {
            if (TGR.Intake.bool())
                setBothMotors(0.5);
            else if (TGR.Extake.bool())
                setBothMotors(-0.5);
            else
                setBothMotors(0);
        }, () -> setBothMotors(0));
    }

    public Command runIntakeAuton() {
        if (testing)
            return Commands.none();
        return startEnd(() -> setBothMotors(1.0), () -> setBothMotors(0));
    }

    private void setBothMotors(double percent) {

        topMotor.set(percent);
        botMotor.set(percent);
    }

    public Command runJustBottomMotor() {
        if (testing)
            return Commands.none();
        return run(() -> {
            botMotor.set(1.0);
            topMotor.set(0);
        });
    }
}
