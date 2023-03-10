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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
  private Solenoid shiftSpeed;

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  private double gyroOffset;
  private static boolean isReversed = false;
  /**
  *
  
  */
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private static final double LEFT_ENCODER_TICKS_PER_METER = 17462;
  private static final double RIGHT_ENCODER_TICKS_PER_METER = 17464;
  private final double HUNDRED_MS_TO_SEC = 10;

  public DriveTrain() {
    // navx = new AHRS(Port.kUSB);
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
    // Phase sensor to have positive increment when driving Talon Forward (Green
    // LED)
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

    left = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    addChild("left", left);

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
    // Phase sensor to have positive increment when driving Talon Forward (Green
    // LED)
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

    right = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
    addChild("right", right);

    shiftSpeed = new Solenoid(1, PneumaticsModuleType.REVPH, 2);
    addChild("shiftSpeed", shiftSpeed);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // try { navx = new AHRS(SPI.Port.kMXP);} catch (RuntimeException ex )
    // {DriverStation.reportError( ex.getMessage(), true);} Timer.delay(1.0);
    // LiveWindow.addSensor("DriveTrain", "navx", navx);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", navx.isConnected());
    SmartDashboard.putNumber("IMU_Yaw", navx.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", navx.getPitch());

    // displays encoder ticks
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", getRightEncoder());

    // display motor group distance
    SmartDashboard.putNumber("Left Distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Distance", getRightEncoderDistance());

    SmartDashboard.putNumber("Get Heading", getHeading());

    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());
  }

  public void run(double l, double r) {
    if (isReversed) {
      l *= -1;
      r *= -1;
    }
    SmartDashboard.putBoolean("Motors Reversed", isReversed);
    // runs numbers from joystick y axis
    left.set(l);
    right.set(r);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoderVelocity(),
        getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  public void resetOdometry(Pose2d pose) {
    zeroHeading();
    resetEncoders();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
  }

  public void zeroHeading() {
    // navx.reset();
    gyroOffset = navx.getAngle();
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees() - gyroOffset;
  }

  public double getLeftEncoder() {
    return leftFrontMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightFrontMotor.getSelectedSensorPosition();
  }

  public void ShiftHigh() {
    shiftSpeed.set(false);
  }

  public void ShiftLow() {
    shiftSpeed.set(true);
  }

  public boolean getReversed() {
    return isReversed;
  }

  public void setReversed(boolean r) {
    isReversed = r;
  }

  public double getLeftEncoderVelocity() {
    return leftFrontMotor.getSelectedSensorVelocity() * HUNDRED_MS_TO_SEC / LEFT_ENCODER_TICKS_PER_METER;
  }

  public double getRightEncoderVelocity() {
    return rightFrontMotor.getSelectedSensorVelocity() * HUNDRED_MS_TO_SEC / RIGHT_ENCODER_TICKS_PER_METER;
  }

  public double getLeftEncoderDistance() {
    return getLeftEncoder() / LEFT_ENCODER_TICKS_PER_METER;
  }

  public double getRightEncoderDistance() {
    return getRightEncoder() / RIGHT_ENCODER_TICKS_PER_METER;
  }

  public void resetEncoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public void setBrake() {
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftRearMotor.setNeutralMode(NeutralMode.Brake);

    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightRearMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftRearMotor.setNeutralMode(NeutralMode.Coast);

    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightRearMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
