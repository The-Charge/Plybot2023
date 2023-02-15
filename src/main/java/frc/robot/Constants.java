// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
   /**
    * public static final class DriveConstants {
    *   public static final int kLeftMotor1Port = 0;
    *   public static final int kLeftMotor2Port = 1;
    *   public static final int kRightMotor1Port = 2;
    *   public static final int kRightMotor2Port = 3; 
    * }
    */ 
    public static final double ksVolts = 1.8516;
    public static final double kvVoltSecondsPerMeter = 4.9995;
    public static final double kaVoltSecondsSquaredPerMeter = 1.0077/6;
    public static final double kPDriveVel = 20;
    public static final double kIDriveVel = 1;
    public static final double kDDriveVel = 0.05;
    

    public static final double kTrackwidthMeters = .67847; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final String cameraName = "HD_USB_Camera";

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9.25);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(6.75);

    public static final double nodeSideDistanceMeters = Units.inchesToMeters(6.25 + 0);


    // private static final List<AprilTag> targetPoses = Collections.unmodifiableList(
    //   List.of(
    //       new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77),Units.inchesToMeters(42.19),Units.inchesToMeters(15.22),new Rotation3d(0,0, Units.degreesToRadians(180)))),
    //       new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45),Units.inchesToMeters(108.19),Units.inchesToMeters(15.22),new Rotation3d(0,0, Units.degreesToRadians(0))))
    //       ));

    // public static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(targetPoses, 0,0); //Experimental AprilTagFieldLayout
    /*Hopefully with accurate measurement from research tomorrow:
     * Robot takes range calculation when a tag is seen, does trig, gets a coordinate.
     * Concern: How to continuously update this value? More cameras? 
     * AprilTagFieldLayout: ID Number, Coordinate (x,y) (not sure about z...), Rotation (180 or 0. Might need to vary depending on which side we are on...)
     * 
     * Need two different lists of tags depending on which side?
     * Probably can set the tag values depending on isRedSide() within the TagAlign Command...
     */
}

