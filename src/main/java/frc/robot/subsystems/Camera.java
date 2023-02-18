package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Camera subsystem to encompass both PhotonCameras
public class Camera extends SubsystemBase{
    private PhotonCamera frontCamera; 

    public Camera() {
        frontCamera = new PhotonCamera(Constants.cameraName); //aprilTagCamera
    }
    @Override
    public void periodic() {
        //Get latest result from camera. If there are targets, put the id of the "best target" defined by photonlib onto the smartdashboard. No target places -1 on dash.
        //https://docs.photonvision.org/en/latest/docs/getting-started/pipeline-tuning/reflectiveAndShape/contour-filtering.html#contour-grouping-and-sorting
        frontCamera.setPipelineIndex(1);
        var res = frontCamera.getLatestResult();
        if (res.hasTargets()) { //if targets sighted
            double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
            Units.inchesToMeters(21), Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(res.getBestTarget().getPitch()));
            Transform3d pose = res.getBestTarget().getBestCameraToTarget(); 
            var bestTarget = res.getBestTarget(); //get the "best target"
            SmartDashboard.putNumber("ID", bestTarget.getFiducialId()); //put "best target" ID onto smartdash
            SmartDashboard.putNumber("Range", range);
            SmartDashboard.putNumber("Z", Units.radiansToDegrees(pose.getRotation().getAngle()));
        }
        else {
            SmartDashboard.putNumber("ID", -1); //no target found
            SmartDashboard.putNumber("Range", -1);
        }
        SmartDashboard.putBoolean("Has AprilTag Target", frontCamera.getLatestResult().hasTargets());
    }
    @Override
    public void simulationPeriodic() {
        
    }
    public PhotonCamera getFrontCamera() {
        //return camera object for command manipulation
        return frontCamera;
    }
}