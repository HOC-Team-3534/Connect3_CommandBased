package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.ELEVATOR.Height;
import frc.robot.RobotContainer.TGR;

public class Elevator extends SubsystemBase {
    WPI_TalonFX elevatorMotor = new WPI_TalonFX(14);

    public Elevator() {
        elevatorMotor
                .configMotionCruiseVelocity(ELEVATOR.kElevatorCruiseVelocity / ELEVATOR.kElevatorCountsToInches / 10.0);
        elevatorMotor
                .configMotionAcceleration(ELEVATOR.kElevatorAcceleration / ELEVATOR.kElevatorCountsToInches / 10.0);
        elevatorMotor.configMotionSCurveStrength(1);
        elevatorMotor.config_kP(0, 0);// TODO config and find the values by
                                      // tuning
        elevatorMotor.config_kI(0, 0);
        elevatorMotor.config_kD(0, 0);// TODO config and find the values by
                                      // tuning
        elevatorMotor.config_kF(0, 0);// TODO config and find the values by
                                      // tuning
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public Command goToDesiredHeight() {
        return runOnce(() -> changeHeight(getDesiredHeight()))
                .andThen(Commands.waitUntil(() -> isCorrectElevatorHeight()));
    }

    public Height getDesiredHeight() {
        Height desiredHeight = Height.LOW;
        if (TGR.PlaceHigh.bool() && TGR.PlaceMid.bool()) {

        } else if (TGR.PlaceHigh.bool())
            desiredHeight = Height.HIGH;
        else if (TGR.PlaceMid.bool())
            desiredHeight = Height.MID;
        return desiredHeight;
    }

    private void changeHeight(ELEVATOR.Height height) {
        elevatorMotor.set(ControlMode.MotionMagic, height.height / ELEVATOR.kElevatorCountsToInches);
    }

    private boolean isCorrectElevatorHeight() {
        return Math.abs(elevatorMotor.getClosedLoopTarget() - elevatorMotor.getSelectedSensorPosition()) < 20; // TODO
                                                                                                               // determine
                                                                                                               // valid
                                                                                                               // window
    }
}
