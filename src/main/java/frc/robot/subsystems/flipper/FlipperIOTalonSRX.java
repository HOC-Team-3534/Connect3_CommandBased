package frc.robot.subsystems.flipper;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Robot;

public class FlipperIOTalonSRX implements FlipperIO {
    WPI_TalonSRX flipper;

    public FlipperIOTalonSRX() {
        if (Robot.isSimulation())
            throw new RuntimeException("Simulation should not instantiate any IO");

        flipper = new WPI_TalonSRX(19);
        flipper.configFactoryDefault();
        flipper.setInverted(true);
    }

    @Override
    public void updateInputs(FlipperIOInputs inputs) {
        inputs.currentAmps = flipper.getSupplyCurrent();
    }

    @Override
    public void set(double percent) {
        flipper.set(percent);
    }
}