package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {

    }

    public default void updateInputs(LightsIOInputs inputs) {
    }

    public default void set(double percent) {

    }
}
