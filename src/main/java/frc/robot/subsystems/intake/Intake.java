package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Intake extends SubsystemBase {
    final IntakeIO io;
    final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    boolean testing = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

    public Command runIntake() {
        if (testing)
            return Commands.none();
        return runEnd(() -> {
            // TODO determine power outputs for intake for best results
            if (TGR.Intake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(0.3);
                else
                    setBothMotors(0.85);
            else if (TGR.Extake.bool())
                if (TGR.CubeLights.bool())
                    setBothMotors(-0.3);
                else
                    setBothMotors(-0.8);
            else
                setBothMotors(0);
        }, () -> setBothMotors(0));
    }

    public Command runIntakeAuton() {
        if (testing)
            return Commands.none();
        return startEnd(() -> setBothMotors(0.3), () -> setBothMotors(0));
    }

    private void setBothMotors(double percent) {
        setTop(percent);
        setBottom(percent);
    }

    private void setTop(double percent) {
        io.setTop(percent);
        Logger.getInstance().recordOutput("Intake/PercentOutputTop_SetPoint", percent);
    }

    private void setBottom(double percent) {
        io.setBottom(percent);
        Logger.getInstance().recordOutput("Intake/PercentOutputBottom_SetPoint", percent);
    }
}