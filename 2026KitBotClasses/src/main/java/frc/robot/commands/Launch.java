// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.LAUNCHING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_RPM;
import static frc.robot.Constants.FuelConstants.SHOOTER_MAX_COMMAND_VOLTS;
import static frc.robot.Constants.FuelConstants.USE_LAUNCHER_RPM_MODE;
import static frc.robot.Constants.VisionConstants.AUTO_LAUNCH_ADJUST_ENABLED;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  /** Creates a new Launch command. */

  private final CANFuelSubsystem fuelSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Timer launchTimer = new Timer();
  private boolean feedLatched = false;

  public Launch(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    this.limelightSubsystem = null;
  }

  public Launch(CANFuelSubsystem fuelSystem, LimelightSubsystem limelightSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    this.limelightSubsystem = limelightSystem;
  }

  private void applyLaunchSetpoints() {
    boolean autoAdjustEnabled = SmartDashboard.getBoolean("Vision/AutoLaunchAdjustEnabled", AUTO_LAUNCH_ADJUST_ENABLED);
    boolean useRpmMode = SmartDashboard.getBoolean("Launcher/UseRpmMode", USE_LAUNCHER_RPM_MODE);

    if (useRpmMode) {
      double launcherRpm = SmartDashboard.getNumber("Launching launcher RPM", LAUNCHING_LAUNCHER_RPM);
      if (autoAdjustEnabled && limelightSubsystem != null && limelightSubsystem.hasHubTagTarget()) {
        launcherRpm = limelightSubsystem.getRecommendedLaunchRpm();
      }
      SmartDashboard.putNumber("Vision/AppliedLaunchRpm", launcherRpm);
      fuelSubsystem.setLauncherTargetRpm(launcherRpm);
    } else {
      double launcherVoltage = SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
      if (autoAdjustEnabled && limelightSubsystem != null && limelightSubsystem.hasHubTagTarget()) {
        launcherVoltage = limelightSubsystem.getRecommendedLaunchVoltage();
      }
      double maxCommandVolts = Math.max(0.0,
          SmartDashboard.getNumber("Launcher/MaxCommandVolts", SHOOTER_MAX_COMMAND_VOLTS));
      launcherVoltage = MathUtil.clamp(launcherVoltage, -maxCommandVolts, maxCommandVolts);
      SmartDashboard.putNumber("Vision/AppliedLaunchVolts", launcherVoltage);
      fuelSubsystem.setIntakeLauncherRoller(launcherVoltage);
    }

    // Keep feeder disabled until launcher reaches target speed.
    double minFeedDelaySeconds = Math.max(0.0, SmartDashboard.getNumber("Launcher/MinFeedDelaySeconds", 0.20));
    boolean feedWhenAtSpeedOnly = SmartDashboard.getBoolean("Launcher/FeedWhenAtSpeedOnly", true) && useRpmMode;
    boolean delayElapsed = launchTimer.hasElapsed(minFeedDelaySeconds);
    boolean atSpeed = fuelSubsystem.isLauncherAtSpeed();
    if (delayElapsed && (!feedWhenAtSpeedOnly || atSpeed || feedLatched)) {
      feedLatched = true;
    }
    boolean allowFeed = delayElapsed && (!feedWhenAtSpeedOnly || feedLatched);
    double feederVoltage = allowFeed
        ? SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE)
        : 0.0;
    fuelSubsystem.setFeederRoller(feederVoltage);
    SmartDashboard.putBoolean("Launcher/FeedDelayElapsed", delayElapsed);
    SmartDashboard.putBoolean("Launcher/FeedAtSpeed", atSpeed);
    SmartDashboard.putBoolean("Launcher/FeedLatched", feedLatched);
    SmartDashboard.putBoolean("Launcher/FeedAllowed", allowFeed);
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for launching
  @Override
  public void initialize() {
    feedLatched = false;
    launchTimer.restart();
    applyLaunchSetpoints();
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    // Keep updating shooter command so moving closer/farther changes speed in real time.
    applyLaunchSetpoints();
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    feedLatched = false;
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
