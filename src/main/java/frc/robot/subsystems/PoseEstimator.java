// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
  private DriveTrain m_DriveTrain;
  private Camera m_Camera;
  private final static Field2d field2d = new Field2d();
  private final PhotonPoseEstimator photonPoseEstimator;
  private static DifferentialDrivePoseEstimator poseEstimator;
  private Optional<EstimatedRobotPose> photonEstimatedRobotPose = Optional.empty();
  private static AprilTagFieldLayout layout;

  public PoseEstimator(DriveTrain drivetrain, Camera camera) {

    m_DriveTrain = drivetrain;
    m_Camera = camera;
    //Load AprilTagFieldLayout (Has Apriltag poses etc.)
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    }
    catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    //Create PoseEstimator Objects
    photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, camera.getFrontCamera(), Constants.robotToCamera);
    poseEstimator = new DifferentialDrivePoseEstimator(Constants.kDriveKinematics, Rotation2d.fromDegrees(m_DriveTrain.getHeading()), m_DriveTrain.getLeftEncoderDistance(), m_DriveTrain.getRightEncoderDistance(), new Pose2d());
    SmartDashboard.putData(field2d);
    SmartDashboard.putString("Tag Pose", getTag(7).get().toString());
  }

  @Override
  public void periodic() {
    photonEstimatedRobotPose = photonPoseEstimator.update();
    //if there is pose estimate from tag data
    if (photonEstimatedRobotPose.isPresent()) {
      EstimatedRobotPose pose = photonEstimatedRobotPose.get();
      try {
        poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
      } catch (ConcurrentModificationException e) {}
      //update the poseEstimator
      poseEstimator.update(Rotation2d.fromDegrees(m_DriveTrain.getHeading()), m_DriveTrain.getLeftEncoderDistance(), m_DriveTrain.getRightEncoderDistance());
      //Adjust positioning on field2d based on which alliance color (Different origin points)
      if (DriverStation.getAlliance() == Alliance.Red) {
        field2d.setRobotPose(new Pose2d(Constants.fieldLength - getCurrentPose().getX(), Constants.fieldWidth - getCurrentPose().getY(), new Rotation2d(getCurrentPose().getRotation().getRadians() + Math.PI)));
      }
      else { 
        field2d.setRobotPose(getCurrentPose());
      }
    }
    SmartDashboard.putNumber("X Pose", getCurrentPose().getX());
    SmartDashboard.putNumber("Y Pose", getCurrentPose().getY());
    SmartDashboard.putNumber("Heading", getCurrentPose().getRotation().getDegrees());
  }
  //Returns latest pose from DifferentialDrivePoseEstimator
  public static Pose2d getCurrentPose() {
    SmartDashboard.putString("Current Estimated Position", poseEstimator.getEstimatedPosition().toString());
    return poseEstimator.getEstimatedPosition();
  }
  //Plot Trajectory onto field2d
  public static void addTrajectory(PathPlannerTrajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }
  //get tag pose information (for tagAlign)
  public static Optional<Pose3d> getTag(int ID) {
    return layout.getTagPose(ID);
  }
}