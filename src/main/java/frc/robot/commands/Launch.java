// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.LAUNCHING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE;
import static frc.robot.Constants.VisionConstants.AUTO_LAUNCH_ADJUST_ENABLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  /** Creates a new Launch command. */

  private final CANFuelSubsystem fuelSubsystem;
  private final LimelightSubsystem limelightSubsystem;

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

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for launching
  @Override
  public void initialize() {
    boolean autoAdjustEnabled = SmartDashboard.getBoolean("Vision/AutoLaunchAdjustEnabled", AUTO_LAUNCH_ADJUST_ENABLED);
    double launcherVoltage = SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);

    if (autoAdjustEnabled && limelightSubsystem != null && limelightSubsystem.hasHubTagTarget()) {
      launcherVoltage = limelightSubsystem.getRecommendedLaunchVoltage();
    }

    SmartDashboard.putNumber("Vision/AppliedLaunchVolts", launcherVoltage);
    fuelSubsystem.setIntakeLauncherRoller(launcherVoltage);
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
