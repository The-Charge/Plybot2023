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
    public boolean fin = false;

    public TagAlign(DriveTrain m_drivetrain, Camera m_camera){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
        SmartDashboard.putNumber("Goal Distance", GOAL_RANGE_METERS);
        m_drivetrain.setBrake();
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
                TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
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
                new PathPoint(new Translation2d(Math.cos(yaw), Math.sin(yaw)), Rotation2d.fromDegrees(90)) // position, heading
            );
            SmartDashboard.putNumber("Range", range);
            SmartDashboard.putNumber("Yaw", yaw);
            SmartDashboard.putNumber("X", Math.cos(yaw));
            SmartDashboard.putNumber("Y", Math.sin(yaw));
            //new AutonomousCommand(m_drivetrain, traj1);
            fin = true;
        }
        else {
            //no targets -> run tankDrive until there is a target in sight
            m_drivetrain.run(-RobotContainer.getInstance().getleftJoystick().getY(),-RobotContainer.getInstance().getrightJoystick().getY());
        }
        SmartDashboard.putBoolean("TagAlign Finished", isFinished());
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
