package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.AUTO_FIRST_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_FIRST_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_TURN;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_PICKUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_TURN;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SETTLE_SECONDS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class LeftDepotAuto extends SequentialCommandGroup {
  public LeftDepotAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    double firstSpinUpSeconds = SmartDashboard.getNumber("Auto/FirstSpinUpSeconds", AUTO_FIRST_SPINUP_SECONDS);
    double firstLaunchSeconds = SmartDashboard.getNumber("Auto/FirstLaunchSeconds", AUTO_FIRST_LAUNCH_SECONDS);
    double outSpeed = SmartDashboard.getNumber("Auto/LeftMiddleOutSpeed", AUTO_LEFT_MIDDLE_OUT_SPEED);
    double outTurn = SmartDashboard.getNumber("Auto/LeftMiddleOutTurn", AUTO_LEFT_MIDDLE_OUT_TURN);
    double outSeconds = SmartDashboard.getNumber("Auto/LeftMiddleOutSeconds", AUTO_LEFT_MIDDLE_OUT_SECONDS);
    double pickupSeconds = SmartDashboard.getNumber("Auto/LeftMiddlePickupSeconds", AUTO_LEFT_MIDDLE_PICKUP_SECONDS);
    double returnSpeed = SmartDashboard.getNumber("Auto/LeftMiddleReturnSpeed", AUTO_LEFT_MIDDLE_RETURN_SPEED);
    double returnTurn = SmartDashboard.getNumber("Auto/LeftMiddleReturnTurn", AUTO_LEFT_MIDDLE_RETURN_TURN);
    double returnSeconds = SmartDashboard.getNumber("Auto/LeftMiddleReturnSeconds", AUTO_LEFT_MIDDLE_RETURN_SECONDS);
    double secondSpinUpSeconds = SmartDashboard.getNumber("Auto/SecondSpinUpSeconds", AUTO_SECOND_SPINUP_SECONDS);
    double secondLaunchSeconds = SmartDashboard.getNumber("Auto/SecondLaunchSeconds", AUTO_SECOND_LAUNCH_SECONDS);
    double settleSeconds = SmartDashboard.getNumber("Auto/SettleSeconds", AUTO_SETTLE_SECONDS);

    addCommands(
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "1-FirstSpinUp")),
        new SpinUp(fuelSubsystem).withTimeout(firstSpinUpSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "2-FirstLaunch")),
        new Launch(fuelSubsystem).withTimeout(firstLaunchSeconds),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "3-LeftToMiddle+Intake")),
        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, outSpeed, outTurn).withTimeout(outSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "4-MiddlePickup")),
        new ParallelDeadlineGroup(
            new WaitCommand(pickupSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "5-ReturnToShoot+Intake")),
        new ParallelDeadlineGroup(
            new AutoDrive(driveSubsystem, returnSpeed, returnTurn).withTimeout(returnSeconds),
            new Intake(fuelSubsystem)),
        new WaitCommand(settleSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "6-SecondSpinUp")),
        new SpinUp(fuelSubsystem).withTimeout(secondSpinUpSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "7-SecondLaunch")),
        new Launch(fuelSubsystem).withTimeout(secondLaunchSeconds),
        new InstantCommand(() -> SmartDashboard.putString("Auto/Stage", "Complete")));
  }
}
