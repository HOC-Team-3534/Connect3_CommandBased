package frc.robot.subsystems.flipper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

import frc.robot.Constants.RobotType;
import frc.robot.Robot;

public class FlipperIOTalonSRX implements FlipperIO {
    WPI_TalonSRX flipper;
    WPI_TalonSRX gripper;

    public FlipperIOTalonSRX() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        gripper = new WPI_TalonSRX(18);
        flipper = new WPI_TalonSRX(19);
        gripper.configFactoryDefault();
        flipper.configFactoryDefault();
        final boolean isInverted = (Constants.ROBOTTYPE == RobotType.PBOT) ? false : false;
        flipper.setInverted(isInverted);
        gripper.setInverted(true);
    }

    @Override
    public void updateInputs(FlipperIOInputs inputs) {
        inputs.currentAmps = flipper.getSupplyCurrent();
        inputs.currentGripAmps = gripper.getSupplyCurrent();
    }

    @Override
    public void set(double percent) {
        flipper.set(percent);
    }

    @Override
    public void setGripper(double percent) {
        gripper.set(percent);
    }
}
