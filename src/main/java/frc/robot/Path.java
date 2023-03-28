package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.Drive.AUTO;

public enum Path {
    BumpSide_PickUp_Third("Bump Side PickUp Third"), BumpSide_PickUp_PlaceSecond("Bump Side Pickup PlaceSecond"),
    BumpSide_PickUp_Balance("Bump Side PickUp Balance"), BumpSide_DriveForwardCone("Bump Side Drive Forward"),
    BumpSide_DriveForwardCube("Bump Side Place Cube Drive Forward"),
    LoadingZone_PickUp_Third("Loading Zone PickUp Third"),
    LoadingZone_PickUp_PlaceSecond("Loading Zone PickUp PlaceSecond"),
    LoadingZone_PickUp_Balance("Loading Zone PickUp Balance"),
    LoadingZone_DriveForwardCone("Loading Zone Drive Forward"),
    LoadingZone_DriveForwardCube("Loading Zone Place Cube Drive Forward");

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

    public PathPlannerTrajectory getPath() { return path; }
}