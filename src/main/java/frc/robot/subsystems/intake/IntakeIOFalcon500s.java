package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Robot;

public class IntakeIOFalcon500s implements IntakeIO {
    WPI_TalonFX topMotor, botMotor;
    double uprightPose;

    public IntakeIOFalcon500s() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        botMotor = new WPI_TalonFX(15);
        topMotor = new WPI_TalonFX(16);
        topMotor.configFactoryDefault();
        botMotor.configFactoryDefault();
        topMotor.setInverted(true);
        botMotor.setInverted(true);
        uprightPose = topMotor.getSelectedSensorPosition();
    }

    @Override
    public void setTop(double percent) {
        topMotor.set(percent);
    }

    @Override
    public void setBottom(double percent) {
        botMotor.set(percent);
    }

    public void goToUpRight() {
        double difference = topMotor.getSelectedSensorPosition() - uprightPose;
        double rotations = difference % (2048 / 5);
        if (rotations < 0)
            rotations += (2048 / 5);
        rotations %= 2048;

        topMotor.set(ControlMode.Position, rotations);
    }
}
