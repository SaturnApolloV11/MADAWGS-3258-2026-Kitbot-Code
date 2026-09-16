package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.AUTO_FIRST_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_FIRST_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_NEUTRAL_PICKUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_RETURN_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_RETURN_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SETTLE_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_TO_NEUTRAL_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_TO_NEUTRAL_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class LeftNeutralAuto extends SequentialCommandGroup {
  public LeftNeutralAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    double firstSpinUpSeconds = SmartDashboard.getNumber("Auto/FirstSpinUpSeconds", AUTO_FIRST_SPINUP_SECONDS);
    double firstLaunchSeconds = SmartDashboard.getNumber("Auto/FirstLaunchSeconds", AUTO_FIRST_LAUNCH_SECONDS);
    double toNeutralSpeed = SmartDashboard.getNumber("Auto/ToNeutralSpeed", AUTO_TO_NEUTRAL_SPEED);
    double toNeutralSeconds = SmartDashboard.getNumber("Auto/ToNeutralSeconds", AUTO_TO_NEUTRAL_SECONDS);
    double neutralPickupSeconds = SmartDashboard.getNumber("Auto/NeutralPickupSeconds", AUTO_NEUTRAL_PICKUP_SECONDS);
    double returnSpeed = SmartDashboard.getNumber("Auto/ReturnSpeed", AUTO_RETURN_SPEED);
    double returnSeconds = SmartDashboard.getNumber("Auto/ReturnSeconds", AUTO_RETURN_SECONDS);
    double secondSpinUpSeconds = SmartDashboard.getNumber("Auto/SecondSpinUpSeconds", AUTO_SECOND_SPINUP_SECONDS);
    double secondLaunchSeconds = SmartDashboard.getNumber("Auto/SecondLaunchSeconds", AUTO_SECOND_LAUNCH_SECONDS);
    double settleSeconds = SmartDashboard.getNumber("Auto/SettleSeconds", AUTO_SETTLE_SECONDS);

    addCommands(
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "1-FirstSpinUp")),
        new SpinUp(fuelSubsystem).withTimeout(firstSpinUpSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "2-FirstLaunch")),
        new Launch(fuelSubsystem).withTimeout(firstLaunchSeconds),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "3-DriveToNeutral+Intake")),
        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, toNeutralSpeed, 0.0).withTimeout(toNeutralSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "4-NeutralPickup")),
        new ParallelDeadlineGroup(
            new WaitCommand(neutralPickupSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "5-Return+Intake")),
        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, returnSpeed, 0.0).withTimeout(returnSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "6-SecondSpinUp")),
        new SpinUp(fuelSubsystem).withTimeout(secondSpinUpSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "7-SecondLaunch")),
        new Launch(fuelSubsystem).withTimeout(secondLaunchSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "Complete")));
  }
}
