// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;
  private final Field2d field = new Field2d();
  private Pose2d estimatedPose = new Pose2d();

  private double estimatedXSpeed;
  private double estimatedZRotation;
  private double lastTimestamp = Timer.getFPGATimestamp();

  // This is only for simulation/visualization of timed autos, not precision odometry.
  private static final double EST_MAX_LINEAR_SPEED_MPS = 3.0;
  private static final double EST_MAX_ANGULAR_SPEED_RAD_PER_SEC = 3.0;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("Drive/Field", field);
    field.setRobotPose(estimatedPose);
    configurePathPlanner();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    if (dt <= 0 || dt > 0.1) {
      field.setRobotPose(estimatedPose);
      return;
    }

    double theta = estimatedPose.getRotation().getRadians();
    double omega = estimatedZRotation * EST_MAX_ANGULAR_SPEED_RAD_PER_SEC;
    double vx = estimatedXSpeed * EST_MAX_LINEAR_SPEED_MPS;

    double newTheta = theta + omega * dt;
    double avgTheta = (theta + newTheta) * 0.5;
    double dx = vx * dt * Math.cos(avgTheta);
    double dy = vx * dt * Math.sin(avgTheta);

    estimatedPose = new Pose2d(
        estimatedPose.getX() + dx,
        estimatedPose.getY() + dy,
        new Rotation2d(newTheta));

    field.setRobotPose(estimatedPose);
    SmartDashboard.putNumber("Drive/EstimatedPoseX", estimatedPose.getX());
    SmartDashboard.putNumber("Drive/EstimatedPoseY", estimatedPose.getY());
    SmartDashboard.putNumber("Drive/EstimatedHeadingDeg", estimatedPose.getRotation().getDegrees());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    estimatedXSpeed = xSpeed;
    estimatedZRotation = zRotation;
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public void resetEstimatedPose(Pose2d pose) {
    estimatedPose = pose;
    field.setRobotPose(estimatedPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(
        estimatedXSpeed * EST_MAX_LINEAR_SPEED_MPS,
        0.0,
        estimatedZRotation * EST_MAX_ANGULAR_SPEED_RAD_PER_SEC);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    double xSpeed = MathUtil.clamp(speeds.vxMetersPerSecond / EST_MAX_LINEAR_SPEED_MPS, -1.0, 1.0);
    double zRotation = MathUtil.clamp(speeds.omegaRadiansPerSecond / EST_MAX_ANGULAR_SPEED_RAD_PER_SEC, -1.0, 1.0);
    driveArcade(xSpeed, zRotation);
  }

  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getEstimatedPose,
          this::resetEstimatedPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPLTVController(0.02),
          config,
          () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
          this);
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner AutoBuilder configuration failed: " + e.getMessage(), false);
    }
  }
}
