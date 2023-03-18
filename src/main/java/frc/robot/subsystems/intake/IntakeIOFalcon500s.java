package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Robot;

public class IntakeIOFalcon500s implements IntakeIO {
    WPI_TalonFX topMotor, botMotor;

    public IntakeIOFalcon500s() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        topMotor = new WPI_TalonFX(15);
        botMotor = new WPI_TalonFX(16);
        topMotor.configFactoryDefault();
        botMotor.configFactoryDefault();
        topMotor.setInverted(true);
        botMotor.setInverted(true);
    }
}
