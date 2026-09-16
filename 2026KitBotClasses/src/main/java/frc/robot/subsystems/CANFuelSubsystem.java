// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private static final double MIN_VALID_LAUNCH_RPM = 500.0;
  private static final double MIN_AT_SPEED_RATIO = 0.90;
  private static final double MAX_ALLOWED_RPM_TOLERANCE = 400.0;
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final RelativeEncoder launcherEncoder;
  private boolean launcherRpmModeEnabled = false;
  private double launcherTargetRpm = 0.0;
  private double launcherCommandVolts = 0.0;
  private double feederCommandVolts = 0.0;
  private double launcherRpmErrorIntegral = 0.0;
  private double previousLauncherRpmError = 0.0;
  private double previousLauncherTargetRpm = 0.0;
  private double lastTimestamp = Timer.getFPGATimestamp();

  private double getSafeMaxCommandVolts() {
    double configured = SmartDashboard.getNumber("Launcher/MaxCommandVolts", SHOOTER_MAX_COMMAND_VOLTS);
    if (configured < 1.0) {
      return SHOOTER_MAX_COMMAND_VOLTS;
    }
    return MathUtil.clamp(configured, 1.0, 12.0);
  }

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // Shooter and feeder are configured as brushless NEO-class motors.
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    // Feeder direction correction for current robot wiring/gearbox orientation.
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    // Shooter is currently spinning opposite of desired direction on this robot.
    launcherConfig.inverted(false);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherEncoder = intakeLauncherRoller.getEncoder();

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putBoolean("Launcher/UseRpmMode", USE_LAUNCHER_RPM_MODE);
    SmartDashboard.putNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
    SmartDashboard.putNumber("Spin-up launcher RPM", SPIN_UP_LAUNCHER_RPM);
    SmartDashboard.putNumber("Launcher/RpmToVoltsKs", LAUNCHER_RPM_TO_VOLTS_KS);
    SmartDashboard.putNumber("Launcher/RpmToVoltsKv", LAUNCHER_RPM_TO_VOLTS_KV);
    SmartDashboard.putNumber("Launcher/RpmKp", LAUNCHER_RPM_KP);
    SmartDashboard.putNumber("Launcher/RpmKi", LAUNCHER_RPM_KI);
    SmartDashboard.putNumber("Launcher/RpmKd", LAUNCHER_RPM_KD);
    SmartDashboard.putNumber("Launcher/RpmTolerance", LAUNCHER_RPM_TOLERANCE);
    SmartDashboard.putNumber("Launcher/RpmIntegralMax", LAUNCHER_RPM_INTEGRAL_MAX);
    SmartDashboard.putBoolean("Launcher/FeedWhenAtSpeedOnly", false);
    SmartDashboard.putNumber("Launcher/MinFeedDelaySeconds", 0.20);
    SmartDashboard.putNumber("Launcher/MaxCommandVolts", SHOOTER_MAX_COMMAND_VOLTS);
    SmartDashboard.putNumber("Fuel/FeederCanId", FEEDER_MOTOR_ID);
    SmartDashboard.putNumber("Fuel/LauncherCanId", INTAKE_LAUNCHER_MOTOR_ID);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage) {
    launcherRpmModeEnabled = false;
    launcherTargetRpm = 0.0;
    launcherRpmErrorIntegral = 0.0;
    previousLauncherRpmError = 0.0;
    double maxCommandVolts = getSafeMaxCommandVolts();
    launcherCommandVolts = MathUtil.clamp(voltage, -maxCommandVolts, maxCommandVolts);
    intakeLauncherRoller.setVoltage(launcherCommandVolts);
  }

  public void setLauncherTargetRpm(double targetRpm) {
    // Cap closed-loop shooter speed to template launch RPM.
    launcherTargetRpm = MathUtil.clamp(targetRpm, 0.0, LAUNCHING_LAUNCHER_RPM);
    if (Math.abs(launcherTargetRpm - previousLauncherTargetRpm) > 50.0) {
      launcherRpmErrorIntegral = 0.0;
      previousLauncherRpmError = 0.0;
    }
    previousLauncherTargetRpm = launcherTargetRpm;
    launcherRpmModeEnabled = true;
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage) {
    double maxCommandVolts = getSafeMaxCommandVolts();
    feederCommandVolts = MathUtil.clamp(voltage, -maxCommandVolts, maxCommandVolts);
    feederRoller.setVoltage(feederCommandVolts);
  }

  // A method to stop the rollers
  public void stop() {
    launcherRpmModeEnabled = false;
    launcherTargetRpm = 0.0;
    previousLauncherTargetRpm = 0.0;
    launcherCommandVolts = 0.0;
    feederCommandVolts = 0.0;
    launcherRpmErrorIntegral = 0.0;
    previousLauncherRpmError = 0.0;
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;
    if (dt <= 0.0 || dt > 0.1) {
      dt = 0.02;
    }

    // Use speed magnitude so control/feed gating works regardless of encoder sign.
    double measuredRpm = Math.abs(launcherEncoder.getVelocity());
    if (launcherRpmModeEnabled) {
      double ks = SmartDashboard.getNumber("Launcher/RpmToVoltsKs", LAUNCHER_RPM_TO_VOLTS_KS);
      double kv = SmartDashboard.getNumber("Launcher/RpmToVoltsKv", LAUNCHER_RPM_TO_VOLTS_KV);
      double kp = SmartDashboard.getNumber("Launcher/RpmKp", LAUNCHER_RPM_KP);
      double ki = SmartDashboard.getNumber("Launcher/RpmKi", LAUNCHER_RPM_KI);
      double kd = SmartDashboard.getNumber("Launcher/RpmKd", LAUNCHER_RPM_KD);
      double integralMax = Math.max(0.0, SmartDashboard.getNumber("Launcher/RpmIntegralMax", LAUNCHER_RPM_INTEGRAL_MAX));
      double maxCommandVolts = getSafeMaxCommandVolts();

      double errorRpm = launcherTargetRpm - measuredRpm;
      launcherRpmErrorIntegral += errorRpm * dt;
      launcherRpmErrorIntegral = MathUtil.clamp(launcherRpmErrorIntegral, -integralMax, integralMax);
      double errorDerivative = (errorRpm - previousLauncherRpmError) / dt;
      previousLauncherRpmError = errorRpm;

      // Blend feedforward and PID correction and clamp to allowable voltage.
      double ffVolts = (kv * launcherTargetRpm) + (launcherTargetRpm > 0.0 ? ks : 0.0);
      double pidVolts = (kp * errorRpm) + (ki * launcherRpmErrorIntegral) + (kd * errorDerivative);
      launcherCommandVolts = MathUtil.clamp(ffVolts + pidVolts, -maxCommandVolts, maxCommandVolts);
      intakeLauncherRoller.setVoltage(launcherCommandVolts);

      SmartDashboard.putNumber("Launcher/RpmError", errorRpm);
    }

    boolean atSpeed = isLauncherAtSpeedInternal(measuredRpm);
    SmartDashboard.putBoolean("Launcher/RpmModeEnabled", launcherRpmModeEnabled);
    SmartDashboard.putBoolean("Launcher/AtSpeed", atSpeed);
    SmartDashboard.putNumber("Launcher/MaxCommandVoltsActive", getSafeMaxCommandVolts());
    SmartDashboard.putNumber("Launcher/TargetRpm", launcherTargetRpm);
    SmartDashboard.putNumber("Launcher/MeasuredRpm", measuredRpm);
    SmartDashboard.putNumber("Launcher/CommandVolts", launcherCommandVolts);
    SmartDashboard.putNumber("Feeder/CommandVolts", feederCommandVolts);
    SmartDashboard.putNumber("Launcher/AppliedOutput", intakeLauncherRoller.getAppliedOutput());
    SmartDashboard.putNumber("Feeder/AppliedOutput", feederRoller.getAppliedOutput());
  }

  public boolean isLauncherAtSpeed() {
    return isLauncherAtSpeedInternal(Math.abs(launcherEncoder.getVelocity()));
  }

  private boolean isLauncherAtSpeedInternal(double measuredRpm) {
    if (!launcherRpmModeEnabled) {
      return false;
    }
    if (launcherTargetRpm < MIN_VALID_LAUNCH_RPM || measuredRpm < MIN_VALID_LAUNCH_RPM) {
      return false;
    }
    double rpmTolerance = MathUtil.clamp(
        SmartDashboard.getNumber("Launcher/RpmTolerance", LAUNCHER_RPM_TOLERANCE),
        0.0,
        MAX_ALLOWED_RPM_TOLERANCE);
    return measuredRpm >= (launcherTargetRpm * MIN_AT_SPEED_RATIO)
        && Math.abs(launcherTargetRpm - measuredRpm) <= rpmTolerance;
  }
}
