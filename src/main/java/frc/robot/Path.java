package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.Drive.AUTO;

public enum Path {
    Drive_Forward_1("Drive Forward Path"),
    Loading_Zone_Place2("Loading Zone Place 2"),
    Loading_Zone_Place2_Pick_Up("Loading Zone Place 2 Pick Up"),
    Bump_Side_Place2("Bump Side Place 2"),
    Bump_Side_Place2_Pick_Up("Bump Side Place 2 Pick Up"),
    Loading_Zone_Place2_Balance("Loading Zone Place 2 Balance"),
    Loading_Zone_Place_PickUp_Balance("Loading Zone Place Balance");

    String pathName;
    PathPlannerTrajectory path;

    Path() {
        this.pathName = this.name();
    }

    Path(String pathName) {
        this.pathName = pathName;
    }

    public void loadPath() {
        path = PathPlanner.loadPath(pathName, AUTO.kMaxSpeedMetersPerSecond,
                AUTO.kMaxAccelerationMetersPerSecondSquared);
    }

    public PathPlannerTrajectory getPath() {
        return path;
    }
}