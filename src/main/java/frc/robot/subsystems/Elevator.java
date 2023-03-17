package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.Constants.RobotType;
import frc.robot.RobotContainer.TGR;

public class Elevator extends SubsystemBase {
    WPI_TalonFX elevatorMotor;
    boolean testing = true;
    Height targetHeight = Height.OFF;

    final static boolean elevatorInverted = (Constants.ROBOTTYPE == RobotType.PBOT) ? false : true;

    public Elevator() {
        elevatorMotor = new WPI_TalonFX(14);
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

    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Encoder Count Elevator", getPosition());
        SmartDashboard.putBoolean("Elevator At Position", isCorrectElevatorHeight());
    }

    public double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public Command goToDesiredHeight() {
        if (testing)
            return Commands.none();
        return goToDesiredHeight(getDesiredHeight());
    }

    public Command goToDesiredHeight(Height height) {
        if (testing)
            return Commands.none();
        Command command = runOnce(() -> targetHeight = height);

        if (height == Height.OFF)
            return command.andThen(runOnce(() -> setPowerZero()));
        return command.andThen(runOnce(() -> changeHeight(height)),
                Commands.waitUntil(() -> isCorrectElevatorHeight()));
    }

    public void setPowerZero() {
        elevatorMotor.set(0);
    }

    public Height getDesiredHeight() {
        Height desiredHeight = Height.LOW;
        if (TGR.PlaceHigh.bool() && TGR.PlaceMid.bool()) {
            desiredHeight = Height.LOW;
        } else if (TGR.PlaceHigh.bool())
            desiredHeight = Height.HIGH;
        else if (TGR.PlaceMid.bool())
            desiredHeight = Height.MID;
        return desiredHeight;
    }

    private void changeHeight(ELEVATOR.Height height) {
        elevatorMotor.set(ControlMode.MotionMagic, height.height);
    }

    private boolean isCorrectElevatorHeight() {
        return Math.abs(targetHeight.height - elevatorMotor.getSelectedSensorPosition()) < 4000;
    }

    public Command elevatorVoltage(double percent) {
        return startEnd(() -> elevatorMotor.set(percent), () -> elevatorMotor.set(0));
    }
}
