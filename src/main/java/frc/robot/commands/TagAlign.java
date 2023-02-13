package frc.robot.commands;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;



public class TagAlign extends CommandBase {
    private final DriveTrain m_drivetrain;
    private final Camera m_camera;
    public double TARGET_HEIGHT_METERS = 0; //Height of target (to be changed)
    final double GOAL_RANGE_METERS = Units.feetToMeters(2); //distance to reach between tag and robot
    public double range, bestTargetYaw;
    public boolean fin = false; //is finished, probably unnecessary
    public int side = 0; //multiplier for positioning

    public TagAlign(DriveTrain m_drivetrain, Camera m_camera, int side){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
        SmartDashboard.putNumber("Goal Distance", GOAL_RANGE_METERS);
        m_drivetrain.setBrake();
        this.side = side;
    }
    @Override
    public void execute() {
        //speeds for arcadedrive
        double forwardSpeed, rotationSpeed;

        //get result, if there are targets within the cameraview
        var result = m_camera.getFrontCamera().getLatestResult();
        if (result.hasTargets()) {
            //calculate range between robot and bestTarget
            if (result.getBestTarget().getFiducialId() == 7) {
                TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13); //11.63 + 9 1/8 <-faked distance for plybot testing
            }
            else {
                TARGET_HEIGHT_METERS = Units.inchesToMeters(24.38);
            }
            range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            double yaw = result.getBestTarget().getYaw();
            PathPlannerTrajectory traj1 = PathPlanner.generatePath(
                new PathConstraints(Constants.kMaxSpeedMetersPerSecond , Constants.kMaxAccelerationMetersPerSecondSquared), 
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(yaw)), // position, heading
                new PathPoint(new Translation2d(range * Math.sin(yaw) + side * Constants.nodeSideDistanceMeters, range * Math.cos(yaw)), Rotation2d.fromDegrees(90)) // position (plus/minus side we aim for), heading
            );
            SmartDashboard.putNumber("Range", range); //Distance from center of Robot to the Apriltag center
            SmartDashboard.putNumber("Yaw", yaw); //Angle between Robot and Tag
            SmartDashboard.putNumber("X", range * Math.sin(yaw)); //X distance between Robot and Tag
            SmartDashboard.putNumber("Y", range * Math.cos(yaw)); //Y distance between Robot and Tag
            //new AutonomousCommand(m_drivetrain, traj1); //Send generated path to be run.
            /*Strategy Discussion: Fully automate the whole scoring process?
             * Utilize FollowPathWithEvents?
             * Otherwise, perhaps just use sequence of commands.
             * 
             * TagAlign -> AutonomousCommand -> Score 
             * 
             * TagAlign Button Pressed (Which Tag to Align to, which side of the node) -> Robot goes to said position
             * Two different Methods
             * 1. Utilize surrounding Apriltags to draw path?
             * 2. Wait till Tag is in viewing distance? Can probably tune TagCamera to target specific ID
             * 
             * Considerations: What if friendly robot is also in the scoring area? How do we avoid them? Waypoints?
             * What about the obstacles on the field? How can we exclude certain positions from being reached given that an obstacle is there? Pose Estimation via AprilTagFieldLayout + Path Planner Trajectory combo?
             * 
             * Scoring Button Pressed (Which position to score into) -> Robot adjusts distance between itself and node to accomidate distances of the arm -> ReflectiveTapeCamera helps adjust to line up with poles if need be 
             * -> Arm lowers -> Claw opens (Piece hopefully drops) -> ???
             * ???: Return to Driver input? Automatically drive out of the "trench"?
            */

            fin = true;
        }
        else {
            //no targets -> run tankDrive until there is a target in sight
            m_drivetrain.run(-RobotContainer.getInstance().getleftJoystick().getY(),-RobotContainer.getInstance().getrightJoystick().getY());
        }
        SmartDashboard.putBoolean("TagAlign Finished", fin);
    }
    @Override
    public boolean isFinished() {
        // if (Math.abs(GOAL_RANGE_METERS - range) < 0.05 && bestTargetYaw < 2) {
        //     //m_drivetrain.setCoastMode();
        //     return true;
        // }
        // return false;
        return fin;
    }

}