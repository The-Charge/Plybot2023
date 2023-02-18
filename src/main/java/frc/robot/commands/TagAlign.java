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
    public double range, bestTargetYaw;
    public boolean fin = false; //is finished, probably unnecessary
    public int side = 0; //multiplier for positioning
    private SequentialCommandGroup group;
    Command fullAuto;

    public TagAlign(DriveTrain m_drivetrain, Camera m_camera, int side){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
        m_drivetrain.setBrake();
        this.side = side;
    }
    @Override
    public void initialize() {
      //get result, if there are targets within the cameraview
      var result = m_camera.getFrontCamera().getLatestResult();
      if (result.hasTargets()) {
          //calculate range between robot and bestTarget
          if (result.getBestTarget().getFiducialId() == 7) {
              TARGET_HEIGHT_METERS = Units.inchesToMeters(21);
          }

          range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

          //double yaw = -1 * result.getBestTarget().getYaw();
          //double y = range * Math.sin(Units.degreesToRadians(yaw));
          //double x = range * Math.cos(Units.degreesToRadians(yaw))- Units.inchesToMeters(14); 
          
           double angle = Units.radiansToDegrees(result.getBestTarget().getBestCameraToTarget().getRotation().getAngle()); 
          double x = result.getBestTarget().getBestCameraToTarget().getX() - Units.inchesToMeters(14.5); 
          double y = result.getBestTarget().getBestCameraToTarget().getY(); 
          double position = m_drivetrain.getHeading();

          PathPlannerTrajectory traj1 = PathPlanner.generatePath( //origin -> apriltag
              new PathConstraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared), //1.5, 0.5
              new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), // position, heading
              new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(-angle - 180)) // position (plus/minus side we aim for), heading
        );
        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("Goal Angle", angle - 180);
        PathPlannerTrajectory traj2 = PathPlanner.generatePath( //origin -> 90 degree 1 meter forward
              new PathConstraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared), //1.5, 0.5
              new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), // position, heading
              new PathPoint(new Translation2d(1, 0), Rotation2d.fromDegrees(-90)) // position (plus/minus side we aim for), heading
        );
          SmartDashboard.putNumber("MaxSpeedMeters/Second", Constants.kMaxSpeedMetersPerSecond);
          SmartDashboard.putNumber("Range TagAlign", range); //Distance from center of Robot to the Apriltag center
          //SmartDashboard.putNumber("Yaw", yaw); //Angle between Robot and Tag
          SmartDashboard.putNumber("X - 14 inch", x); //X distance between Robot and Tag
          SmartDashboard.putNumber("Y", y); //Y distance between Robot and Tag

          HashMap<String, Command> eventMap = new HashMap<>();
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

          //SmartDashboard.putString("Blah", m_drivetrain.getPose().toString());
          fullAuto = autoBuilder.fullAuto(traj1);

          group = new SequentialCommandGroup(
              fullAuto.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)));
          
        group.schedule(); //schedule command for running
      }
    }

    @Override
    public void execute() {
    }
    @Override
    public boolean isFinished() {
        if (group == null) {
            return true;
        }
        else {
            SmartDashboard.putBoolean("TagAlign Finished", group.isScheduled());
            return group.isFinished();
        }
    }

}
