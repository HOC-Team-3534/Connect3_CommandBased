package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Robot;

public class IntakeIOFalcon500s implements IntakeIO {
    WPI_TalonFX topMotor, botMotor;
    double uprightPose;

    public IntakeIOFalcon500s() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        botMotor = new WPI_TalonFX(16);
        topMotor = new WPI_TalonFX(15);
        topMotor.configFactoryDefault();
        botMotor.configFactoryDefault();
        topMotor.setInverted(true);
        botMotor.setInverted(true);
        topMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topMotor.setSelectedSensorPosition(0);
        topMotor.config_kP(0, 0.05);
        uprightPose = 0;
    }

    @Override
    public void setTop(double percent) {
        topMotor.set(percent);
    }

    @Override
    public void setBottom(double percent) {
        botMotor.set(percent);
    }

    @Override
    public void goToUpRight() {
        setBottom(0.0);
        double position = topMotor.getSelectedSensorPosition();
        double difference = position - uprightPose;
        double halfRotationsOfIntake = difference % (2048.0 / 10.0);// Ratio is 5 so half 5 * 2 = 10
        if (halfRotationsOfIntake < 0)
            halfRotationsOfIntake += (2048.0 / 10.0);
        double rot0To1 = halfRotationsOfIntake / 204.8;
        if (rot0To1 > 0.5) {
            position += 204.8 - halfRotationsOfIntake;
        } else {
            position -= halfRotationsOfIntake;
        }
        topMotor.set(ControlMode.Position, position);

    }
}
