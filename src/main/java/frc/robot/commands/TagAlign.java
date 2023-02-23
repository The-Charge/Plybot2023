package frc.robot.commands;

import java.util.HashMap;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class TagAlign extends CommandBase {
    private final DriveTrain m_drivetrain;
    private final Camera m_camera;
    public double TARGET_HEIGHT_METERS = 0; //Height of target (to be changed)
    public double range;
    
    public int side = 0; //multiplier for positioning
    private SequentialCommandGroup group;
    Command fullAuto;
    boolean fin = false;

    public TagAlign(DriveTrain m_drivetrain, Camera m_camera, int side){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
        m_drivetrain.setBrake();
        this.side = side;
    }
    @Override
    public void execute() {
      //get result, if there are targets within the cameraview
        var result = m_camera.getFrontCamera().getLatestResult();
        if (result.hasTargets()) {
          //calculate range between robot and bestTarget
            if (result.getBestTarget().getFiducialId() == 7) {
              TARGET_HEIGHT_METERS = Units.inchesToMeters(21);
            }

            range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            
            //For Traj1
            double angle = Units.radiansToDegrees(result.getBestTarget().getBestCameraToTarget().getRotation().getAngle()); 
            double x = result.getBestTarget().getBestCameraToTarget().getX() - Units.inchesToMeters(14.5); 
            double y = result.getBestTarget().getBestCameraToTarget().getY(); 

            //For Traj2
            //double x = PoseEstimator.getTag(7).get().getX();
            //double y = PoseEstimator.getTag(7).get().getY();
            Rotation2d z = PoseEstimator.getTag(7).get().getRotation().toRotation2d();

            //Field Oriented Trajectory
            PathPlannerTrajectory traj2 = PathPlanner.generatePath(new PathConstraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared),
             new PathPoint(new Translation2d(PoseEstimator.getCurrentPose().getX(), PoseEstimator.getCurrentPose().getY()), Rotation2d.fromDegrees(m_drivetrain.getHeading())),
             new PathPoint(new Translation2d(x, y), z)
             );
            //Relative Trajectory
            PathPlannerTrajectory traj1 = PathPlanner.generatePath( //origin -> apriltag
                new PathConstraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared), //1.5, 0.5
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), // position, heading
                new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(-angle - 180)) // position (plus/minus side we aim for), heading
            );

            //HashMap for Events
            HashMap<String, Command> eventMap = new HashMap<>();
            
            RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
              m_drivetrain::getPose,
              m_drivetrain::resetOdometry,
              new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
              Constants.kDriveKinematics,
              new SimpleMotorFeedforward(
                      Constants.ksVolts,
                      Constants.kvVoltSecondsPerMeter,
                      Constants.kaVoltSecondsSquaredPerMeter),
              m_drivetrain::getWheelSpeeds,
              new PIDConstants(Constants.kPDriveVel, Constants.kIDriveVel, Constants.kDDriveVel),
              m_drivetrain::tankDriveVolts,
              eventMap,
              false,
              m_drivetrain);

          fullAuto = autoBuilder.fullAuto(traj1);

          group = new SequentialCommandGroup(
              fullAuto.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)));
          
        //group.schedule(); //schedule command for running
        PoseEstimator.addTrajectory(traj1); //add trajectory to the field object

        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("Goal Angle", angle - 180);

        SmartDashboard.putNumber("X - 14 inch", x); //X distance between Robot and Tag
        SmartDashboard.putNumber("Y", y); //Y distance between Robot and Tag
        fin = true;
      }
    }

    @Override
    public boolean isFinished() {
        if ((group == null) || fin) {
            return true;
        }
        else {
            SmartDashboard.putBoolean("TagAlign Finished", group.isScheduled());
            return group.isFinished();
        }
    }

}
