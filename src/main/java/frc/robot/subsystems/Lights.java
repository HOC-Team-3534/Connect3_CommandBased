package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.TGR;

public class Lights extends SubsystemBase {
    Spark spark = new Spark(0);

    public Command runLights() {
        return run(() -> {
            if (TGR.ConeLights.bool())
                spark.set(0.69); // YELLOW
            else if (TGR.CubeLights.bool())
                spark.set(0.91); // PURPLE
            else if (TGR.DTM.bool())
                spark.set(0.77); // GREEN
            else
                neutral();
        });
    }

    public void neutral() {
        // This sets it to blue need to have a chooser in smart dashboard to see
        // what
        // color we are
        switch (DriverStation.getAlliance()) {
            case Blue:
                spark.set(0.87);
                break;

            case Red:
                spark.set(0.61);
                break;

            default:
                spark.set(0.0);
                break;
        }
    }
}
