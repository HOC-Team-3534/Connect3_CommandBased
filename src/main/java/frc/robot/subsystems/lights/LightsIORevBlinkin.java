package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LightsIORevBlinkin implements LightsIO {
    Spark spark = new Spark(0);

    public void set(double percent) {
        spark.set(percent);
    }
}
