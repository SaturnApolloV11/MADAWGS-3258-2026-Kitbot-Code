package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.DRIVE_SCALING;
import static frc.robot.Constants.OperatorConstants.ROTATION_SCALING;
import static frc.robot.Constants.VisionConstants.AIM_ROTATION_MAX;
import static frc.robot.Constants.VisionConstants.AIM_TX_DEADBAND_DEG;
import static frc.robot.Constants.VisionConstants.AIM_TX_KP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveAimAssist extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final CommandXboxController controller;

  public DriveAimAssist(
      CANDriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    // Keep normal driver throttle on left stick while auto-aim only adjusts rotation.
    double xSpeed = -controller.getLeftY() * DRIVE_SCALING;
    double zRotation = -controller.getRightX() * ROTATION_SCALING;

    if (limelightSubsystem.hasHubTagTarget()) {
      double tx = limelightSubsystem.getTx();
      double kp = SmartDashboard.getNumber("Vision/AimTxKp", AIM_TX_KP);
      double maxRot = SmartDashboard.getNumber("Vision/AimRotationMax", AIM_ROTATION_MAX);
      double deadbandDeg = SmartDashboard.getNumber("Vision/AimTxDeadbandDeg", AIM_TX_DEADBAND_DEG);
      if (Math.abs(tx) > deadbandDeg) {
        // Positive tx means target is right of center; command opposite turn to re-center.
        zRotation = MathUtil.clamp(-tx * kp, -maxRot, maxRot);
      } else {
        // Inside deadband, hold heading instead of hunting around zero.
        zRotation = 0.0;
      }
    }

    SmartDashboard.putNumber("Vision/AimAssistRotationCmd", zRotation);
    driveSubsystem.driveArcade(xSpeed, zRotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
