package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class IntakeIOFalcon500s implements IntakeIO {
    WPI_TalonFX topMotor, botMotor;
    int[] inChassisInFrame = { 0 }; // Need to find values

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

    private double closestChassisValue() {
        double currentEncoderCount = topMotor.getSelectedSensorPosition();
        double closestValue = inChassisInFrame[0];
        for (int i = 1; i < inChassisInFrame.length; i++) {
            if (Math.abs(currentEncoderCount - inChassisInFrame[i]) > closestValue) {
                closestValue = inChassisInFrame[i];
            }
        }
        return closestValue;
    }

    private boolean inChassisFrame() {
        return Math.abs(topMotor.getSelectedSensorPosition() - closestChassisValue()) < 200;
    }

    @Override
    public void intakeIntoChassis() {
        if (!inChassisFrame())
            topMotor.set(ControlMode.MotionMagic, closestChassisValue());
    }

    @Override
    public void setTop(double percent) {
        topMotor.set(percent);
    }

    @Override
    public void setBottom(double percent) {
        botMotor.set(percent);
    }
}
