package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.Drive.AUTO;

public enum Path {
    BumpSide_PlaceCone_ExtakeCube("Bump Side Place Cone Extake"),
    BumpSide_PlaceCube_ExtakeCube("Bump Side Place Cube Extake"), BumpSide_ExtakeThird("Bump Side PickUp Third"),
    BumpSide_PlaceCone_Balance("Bump Side Cone Balance"), BumpSide_PlaceCube_Balance("Bump Side Cube Balance"),
    BumpSide_DriveForwardCone("Bump Side Drive Forward"),
    BumpSide_DriveForwardCube("Bump Side Place Cube Drive Forward"),
    BumpSide_GetFirstPieceAfterBump("Bump Side Extake PickUp First"),
    BumpSide_GetSecondPieceAfterBump("Bump Side Extake PickUp Second"),
    // Loading Zone Paths
    LoadingZone_PlaceCone_ExtakeCube("Loading Zone Place Cone Extake 2"),
    LoadingZone_PlaceCube_ExtakeCube("Loading Zone Place Cube Get Second"),
    LoadingZone_ExtakeThird("Loading Zone PickUp Third"), LoadingZone_PlaceCone_Balance("Loading Zone Cone Balance"),
    LoadingZone_PlaceCube_Balance("Loading Zone Cube Balance"),
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