package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.AutoLog;

public interface FlipperIO {
    @AutoLog
    public static class FlipperIOInputs {
        public double currentAmps;
    }

    public default void updateInputs(FlipperIOInputs inputs) {

    }

    public default void set(double percent) {

    }

}
