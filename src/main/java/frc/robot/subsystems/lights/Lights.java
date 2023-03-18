package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Lights extends SubsystemBase {

    final LightsIO io;
    final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

    public Lights(LightsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Lights", inputs);
    }

    public Command runLights() {
        return run(() -> {
            if (TGR.ConeLights.bool())
                set(0.69); // YELLOW
            else if (TGR.CubeLights.bool())
                set(0.91); // PURPLE
            else if (TGR.DTM.bool())
                set(0.77); // GREEN
            else
                neutral();
        });
    }

    private void set(double percent) {
        io.set(percent);
        Logger.getInstance().recordOutput("Lights/PercentOutput_SetPoint", percent);
    }

    public void neutral() {
        // This sets it to blue need to have a chooser in smart dashboard to see
        // what
        // color we are
        switch (DriverStation.getAlliance()) {
            case Blue:
                set(0.87);
                break;

            case Red:
                set(0.61);
                break;

            default:
                set(0.0);
                break;
        }
    }
}
