// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.*;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
 import com.ctre.phoenix.motorcontrol.FeedbackDevice;
 import com.ctre.phoenix.motorcontrol.NeutralMode;
 import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
 import edu.wpi.first.wpilibj.I2C;
 import edu.wpi.first.wpilibj.SPI;
 import edu.wpi.first.wpilibj.SerialPort.Port;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS



    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private AHRS navx;
private WPI_TalonSRX leftFrontMotor;
private WPI_TalonSRX leftRearMotor;
private MotorControllerGroup left;
private WPI_TalonSRX rightFrontMotor;
private WPI_TalonSRX rightRearMotor;
private MotorControllerGroup right;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
navx = new AHRS(SPI.Port.kMXP);
 
 

leftFrontMotor = new WPI_TalonSRX(1);
 
    /* Factory default hardware to prevent unexpected behavior */
leftFrontMotor.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
leftFrontMotor.setInverted(false);
leftFrontMotor.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
leftFrontMotor.configNominalOutputForward(0, 30);
leftFrontMotor.configNominalOutputReverse(0, 30);
leftFrontMotor.configPeakOutputForward(1, 30);
leftFrontMotor.configPeakOutputReverse(-1, 30);
        
/* Configure Sensor */
        // Phase sensor to have positive increment when driving Talon Forward (Green LED)
leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
leftFrontMotor.setSensorPhase(false);


        /* Set gains in slot0 - see documentation */
leftFrontMotor.selectProfileSlot(0, 0);
leftFrontMotor.config_kF(0, 0.0, 30);
leftFrontMotor.config_kP(0, 0.0, 30);
leftFrontMotor.config_kI(0, 0.0, 30);
leftFrontMotor.config_kD(0, 0.0, 30);

leftRearMotor = new WPI_TalonSRX(2);
 
    /* Factory default hardware to prevent unexpected behavior */
leftRearMotor.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
leftRearMotor.setInverted(false);
leftRearMotor.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
leftRearMotor.configNominalOutputForward(0, 30);
leftRearMotor.configNominalOutputReverse(0, 30);
leftRearMotor.configPeakOutputForward(1, 30);
leftRearMotor.configPeakOutputReverse(-1, 30);

left = new MotorControllerGroup(leftFrontMotor, leftRearMotor  );
 addChild("left",left);
 

rightFrontMotor = new WPI_TalonSRX(7);
 
    /* Factory default hardware to prevent unexpected behavior */
rightFrontMotor.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
rightFrontMotor.setInverted(true);
rightFrontMotor.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
rightFrontMotor.configNominalOutputForward(0, 30);
rightFrontMotor.configNominalOutputReverse(0, 30);
rightFrontMotor.configPeakOutputForward(1, 30);
rightFrontMotor.configPeakOutputReverse(-1, 30);
        
/* Configure Sensor */
        // Phase sensor to have positive increment when driving Talon Forward (Green LED)
rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
rightFrontMotor.setSensorPhase(false);


        /* Set gains in slot0 - see documentation */
rightFrontMotor.selectProfileSlot(0, 0);
rightFrontMotor.config_kF(0, 0.0, 30);
rightFrontMotor.config_kP(0, 0.0, 30);
rightFrontMotor.config_kI(0, 0.0, 30);
rightFrontMotor.config_kD(0, 0.0, 30);   

rightRearMotor = new WPI_TalonSRX(8);
 
    /* Factory default hardware to prevent unexpected behavior */
rightRearMotor.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
rightRearMotor.setInverted(true);
rightRearMotor.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
rightRearMotor.configNominalOutputForward(0, 30);
rightRearMotor.configNominalOutputReverse(0, 30);
rightRearMotor.configPeakOutputForward(1, 30);
rightRearMotor.configPeakOutputReverse(-1, 30);        

right = new MotorControllerGroup(rightFrontMotor, rightRearMotor  );
 addChild("right",right);
 

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    
    //try { navx = new AHRS(SPI.Port.kMXP);} catch (RuntimeException ex ) {DriverStation.reportError( ex.getMessage(), true);} Timer.delay(1.0);
    //LiveWindow.addSensor("DriveTrain", "navx", navx);

    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
          /* Display 6-axis Processed Angle Data                                      */
          SmartDashboard.putBoolean(  "IMU_Connected",        navx.isConnected());
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    navx.isCalibrating());
          SmartDashboard.putNumber(   "IMU_Yaw",              navx.getYaw());
          SmartDashboard.putNumber(   "IMU_Pitch",            navx.getPitch());
          SmartDashboard.putNumber(   "IMU_Roll",             navx.getRoll());
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   navx.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     navx.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         navx.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       navx.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          navx.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          navx.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         navx.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       navx.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           navx.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           navx.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       navx.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       navx.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            navx.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            navx.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            navx.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           navx.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           navx.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           navx.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             navx.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             navx.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             navx.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           navx.getTempC());


           /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      navx.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          navx.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          navx.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          navx.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          navx.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       navx.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     navx.getUpdateCount());

        
          //displays encoder ticks
          SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
          SmartDashboard.putNumber("Right Encoder", getRightEncoder());
    }  
    
    public void run(double l, double r) {
        //runs numbers from joystick y axis
        leftFrontMotor.set(l);
        rightFrontMotor.set(r);
      }

      public double getLeftEncoder(){
        return leftFrontMotor.getSelectedSensorPosition();
      }

      public double getRightEncoder(){
        return rightFrontMotor.getSelectedSensorPosition();
      }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

