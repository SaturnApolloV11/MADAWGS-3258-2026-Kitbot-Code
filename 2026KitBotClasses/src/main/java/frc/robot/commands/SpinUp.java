// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUp extends Command {
  /** Creates a new SpinUp command. */

  private final CANFuelSubsystem fuelSubsystem;

  public SpinUp(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  private void applySpinUpSetpoints() {
    boolean useRpmMode = SmartDashboard.getBoolean("Launcher/UseRpmMode", USE_LAUNCHER_RPM_MODE);
    if (useRpmMode) {
      fuelSubsystem.setLauncherTargetRpm(SmartDashboard.getNumber("Spin-up launcher RPM", SPIN_UP_LAUNCHER_RPM));
    } else {
      fuelSubsystem.setIntakeLauncherRoller(
          SmartDashboard.getNumber("Spin-up launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    }
    // Template behavior: run feeder away from shooter during spin-up.
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    applySpinUpSetpoints();
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    // Continuously refresh target so tuning changes apply while holding the trigger.
    applySpinUpSetpoints();
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    // Preserve shooter momentum when SpinUp times out and Launch starts next.
    // Only stop if SpinUp is canceled/interrupted.
    if (interrupted) {
      fuelSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
