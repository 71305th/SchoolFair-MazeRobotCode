// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_TalonSRX leftLead = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
  private final WPI_TalonSRX rightLead = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
  private final WPI_TalonSRX leftFollow = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonSRX rightFollow = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);


  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
          leftFollow,
          leftLead);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          rightFollow,
          rightLead);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private Pose2d savedPose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetEncoders();
    zeroHeading();

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 1023.0 / 6800.0;
    talonConfig.slot0.kP = 0.8;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = .25;

    rightFollow.configAllSettings(talonConfig);
    rightLead.configAllSettings(talonConfig);
    leftFollow.configAllSettings(talonConfig);
    leftLead.configAllSettings(talonConfig);

    rightFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
    rightLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
    leftFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
    leftLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
    
    setNeutralMode(NeutralMode.Brake);
    
    rightFollow.setInverted(true);
    rightLead.setInverted(true);
    rightLead.setSensorPhase(true);
    leftLead.setSensorPhase(true);
    rightLead.overrideLimitSwitchesEnable(false);
    leftLead.overrideLimitSwitchesEnable(false);

    // Sets the distance per pulse for the encoders
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), 
        stepsToMeters(getLeftEncoderPosition()), 
        stepsToMeters(getRightEncoderPosition()));
    
    SmartDashboard.putNumber("left encoder", -leftLead.getSelectedSensorPosition()*DriveConstants.kEncoderDistanceRatio);
    SmartDashboard.putNumber("right encoder", rightLead.getSelectedSensorPosition()*DriveConstants.kEncoderDistanceRatio);
    SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Velocity", stepsPerDecisecToMetersPerSec(leftLead.getSelectedSensorVelocity()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        stepsPerDecisecToMetersPerSec(leftLead.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightLead.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.zeroYaw();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
    m_odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    arcadeDrive(fwd,rot,false);
  }

  public void arcadeDrive(double speed, double rotation, boolean useSquares){
    m_drive.arcadeDrive(speed, rotation, useSquares);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */

  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
    m_drive.tankDrive(leftSpeed, rightSpeed, useSquares);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    leftFollow.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity));
    leftLead.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity));
    rightFollow.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity));
    rightLead.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }
  public void setUseDifferentialDrive(boolean useDifferentialDrive) {
    m_drive.setSafetyEnabled(useDifferentialDrive);
    if (!useDifferentialDrive) {
      leftFollow.follow(leftLead);
      rightFollow.follow(rightLead);
    }
  }

  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new InstantCommand(() -> setUseDifferentialDrive(false), this)
        .andThen(new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(2, 0.7),
            DriveConstants.feedForward,
            DriveConstants.kDriveKinematics,
            this::getWheelSpeeds,
            new PIDController(0.8, 0, 0),
            new PIDController(0.8, 0, 0),
            this::tankDriveVolts,
            this))
        .andThen(new InstantCommand(() -> {
            setUseDifferentialDrive(true);
            arcadeDrive(0, 0);
        }, this));
  }
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (-leftLead.getSelectedSensorPosition()*DriveConstants.kEncoderDistanceRatio + rightLead.getSelectedSensorPosition()*DriveConstants.kEncoderDistanceRatio) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    leftFollow.setNeutralMode(neutralMode);
    leftLead.setNeutralMode(neutralMode);
    rightFollow.setNeutralMode(neutralMode);
    rightLead.setNeutralMode(neutralMode);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }
  public static double metersToSteps(double meters) {
    return (meters / DriveConstants.kWheelCircumference) * DriveConstants.kEncoderCPR;
  }

  public WPI_TalonSRX getLeftTalonSRX() {
    return leftLead;
  }

  public WPI_TalonSRX getRightTalonSRX() {
    return rightLead;
  }

  public double getLeftEncoderPosition() {
    return leftLead.getSelectedSensorPosition(0);
  }

  public double getRightEncoderPosition() {
    return rightLead.getSelectedSensorPosition(0);
  }

  public void saveCurrentPose(){
    savedPose = getPose();
  }

  public Pose2d getSavedPose(){
    return savedPose;
  }
  public static double insToRevs(double inches) {
    return inches / DriveConstants.kWheelCircumference;
  }
  public static double insToSteps(double inches) {
    return (insToRevs(inches) * DriveConstants.kEncoderCPR);
  }
  public static double insPerSecToStepsPerDecisec(double inchesPerSec) {
    return insToSteps(inchesPerSec) * .1;
  }
  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }
  public static double stepsToMeters(double steps) {
    return (DriveConstants.kWheelCircumference / DriveConstants.kEncoderCPR) * steps;
  }
  public void testMotor(){
    tankDriveVolts(12, 12);
  }
  public void stopMotor(){
    tankDriveVolts(0, 0);
  }

}
