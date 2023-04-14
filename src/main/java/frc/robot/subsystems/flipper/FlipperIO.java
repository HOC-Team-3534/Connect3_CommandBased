package frc.robot.subsystems.flipper;

import org.littletonrobotics.junction.AutoLog;

public interface FlipperIO {
    @AutoLog
    public static class FlipperIOInputs {
        public double currentAmps;
        public double currentGripAmps;

    }

    public default void updateInputs(FlipperIOInputs inputs) {

    }

    public default void set(double percent) {

    }

    public default void setGripper(double percent) {

    }

}
