package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.AUTO_DRIVE_BACK_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_DRIVE_BACK_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SPINUP_SECONDS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class PreloadMobilityTimedAuto extends SequentialCommandGroup {
  public PreloadMobilityTimedAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    double spinUpSeconds = SmartDashboard.getNumber("Auto/SpinUpSeconds", AUTO_SPINUP_SECONDS);
    double launchSeconds = SmartDashboard.getNumber("Auto/LaunchSeconds", AUTO_LAUNCH_SECONDS);
    double driveBackSpeed = SmartDashboard.getNumber("Auto/DriveBackSpeed", AUTO_DRIVE_BACK_SPEED);
    double driveBackSeconds = SmartDashboard.getNumber("Auto/DriveBackSeconds", AUTO_DRIVE_BACK_SECONDS);

    // Back up first, then spin up and shoot from the new position.
    addCommands(
        new AutoDrive(driveSubsystem, driveBackSpeed, 0.0).withTimeout(driveBackSeconds),
        new SpinUp(fuelSubsystem).withTimeout(spinUpSeconds),
        new Launch(fuelSubsystem).withTimeout(launchSeconds));
  }
}


