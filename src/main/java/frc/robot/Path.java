package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.Drive.AUTO;

public enum Path {
    Drive_Forward_1("Drive Forward Path"),
    Far_Left_Path_Place2("Loading Zone Place 2"),
    Far_Left_Path_Place2_Pick_Up("Loading Zone Place 2 Pick Up"),
    Far_Right_Path_Place2("Bump Side Place 2"),
    Far_Right_Path_PickUp("Bump Side Place 2 Pick Up");

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