package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Robot;

public class GripperIOTalonSRX implements GripperIO {
    WPI_TalonSRX gripper;

    public GripperIOTalonSRX() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        gripper = new WPI_TalonSRX(18);
        gripper.configFactoryDefault();
        gripper.setInverted(true);
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.currentAmps = gripper.getSupplyCurrent();
    }

    @Override
    public void set(double percent) {
        gripper.set(percent);
    }
}
