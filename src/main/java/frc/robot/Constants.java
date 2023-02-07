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

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    public static final double kaVoltSecondsSquaredPerMeter = 1.0077 / 6; //1.0077
    public static final double kPDriveVel = 1.7814;
    public static final double kIDriveVel = 1;
    public static final double kDDriveVel = 0.05;

    public static final double kTrackwidthMeters = .67847 * 2; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 3 / 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1 / 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


    public static final double NUMBER_OF_PATHWAYS = 2;
}

