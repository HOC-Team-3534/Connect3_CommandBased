package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setTop(double percent) {}

    public default void setBottom(double percent) {

    }

    public default void intakeIntoChassis() {}
}
