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
                yellow();
            else if (TGR.CubeLights.bool())
                purple();
            else if (TGR.DTM.bool())
                green();
            else
                neutral();
        });
    }

    public void green() {
        spark.set(0.77);
    }

    public void yellow() {
        spark.set(0.69);
    }

    public void purple() {
        spark.set(0.91);
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
