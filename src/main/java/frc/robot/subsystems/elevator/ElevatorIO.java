package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.ELEVATOR.Height;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void set(double percent) {
    }

    public default void setHeight(Height height) {
    }

}
