package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
    @AutoLog
    public static class GripperIOInputs {
        public double currentAmps;
    }

    public default void updateInputs(GripperIOInputs inputs) {
    }

    public default void set(double percent) {
    }
}
