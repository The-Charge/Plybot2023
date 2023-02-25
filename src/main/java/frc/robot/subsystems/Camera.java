package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

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
        var res = frontCamera.getLatestResult();
        if (res.hasTargets()) { //if targets sighted
            // double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
            // Units.inchesToMeters(21), Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(res.getBestTarget().getPitch()));

            double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
            Units.inchesToMeters(15), Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(res.getBestTarget().getPitch()));
            
            Transform3d pose = res.getBestTarget().getBestCameraToTarget(); 
            var bestTarget = res.getBestTarget(); //get the "best target"
            
            //SmartDashboard.putNumber("ID", bestTarget.getFiducialId()); //put "best target" ID onto smartdash
            SmartDashboard.putNumber("Range", range);
            SmartDashboard.putNumber("X", pose.getX());
            SmartDashboard.putNumber("X adjusted", pose.getX() - Units.inchesToMeters(14.5));
            SmartDashboard.putNumber("Y", pose.getY());
            SmartDashboard.putNumber("Rotation dif", Units.radiansToDegrees(pose.getRotation().getAngle()));
            SmartDashboard.putNumber("Pose Ambiguity",res.getBestTarget().getPoseAmbiguity());
            SmartDashboard.putNumber("Pipeline Index", frontCamera.getPipelineIndex());
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
    public void setFrontCamera(int index) {
        frontCamera.setPipelineIndex(index);
    }
    public void setFrontCamera() {
        frontCamera.setPipelineIndex(Math.abs(frontCamera.getPipelineIndex() - 1));
        frontCamera.setLED(VisionLEDMode.kDefault);
    }
    public PhotonCamera getFrontCamera() {
        //return camera object for command manipulation
        return frontCamera;
    }
}