package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.ELEVATOR.Height;

public class ElevatorIOFalcon500 implements ElevatorIO {
    WPI_TalonFX elevatorMotor;

    public ElevatorIOFalcon500() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        elevatorMotor = new WPI_TalonFX(14);
        var elevatorInverted = (Constants.ROBOTTYPE == RobotType.PBOT) ? false : true;
        elevatorMotor.setInverted(elevatorInverted);
        elevatorMotor.setSensorPhase(false);
        elevatorMotor.setSelectedSensorPosition(0);
        elevatorMotor
                .configMotionCruiseVelocity(
                        ELEVATOR.kElevatorCruiseVelocity);
        elevatorMotor
                .configMotionAcceleration(ELEVATOR.kElevatorAcceleration);
        elevatorMotor.configMotionSCurveStrength(2);
        elevatorMotor.config_kP(0, 0.05);
        elevatorMotor.config_kI(0, 0);
        elevatorMotor.config_kD(0, 0.5);
        elevatorMotor.config_kF(0, 0);
        elevatorMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = elevatorMotor.getSelectedSensorPosition();
    }

    @Override
    public void set(double percent) {
        elevatorMotor.set(percent);
    }

    @Override
    public void setHeight(Height height) {
        elevatorMotor.set(ControlMode.MotionMagic, height.height);
    }
}
