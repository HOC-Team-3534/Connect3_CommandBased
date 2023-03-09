package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Intake extends SubsystemBase {
    WPI_TalonFX topMotor, botMotor;
    boolean testing = false;

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

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Current", botMotor.getSupplyCurrent());
    }

    public double getAmps() {
        return botMotor.getSupplyCurrent();
    }

    public Command runIntake() {
        if (testing)
            return Commands.none();
        return runEnd(() -> {
            if (TGR.Intake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(0.3);
                else
                    setBothMotors(0.8);
            else if (TGR.Extake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(-0.3);
                else
                    setBothMotors(-0.8);
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

    public Command runBothSlow() {
        if (testing)
            return Commands.none();
        return run(() -> {
            botMotor.set(0.50);
            topMotor.set(0.25);
        });
    }

    public Command runBothFinal() {
        if (testing)
            return Commands.none();
        return run(() -> {
            botMotor.set(1.0);
            topMotor.set(0.65);
        });
    }

    public Command runBottomDownTopUp() {
        if (testing)
            return Commands.none();
        return run(() -> {
            botMotor.set(-0.3);
            topMotor.set(0.25);
        });
    }

}
