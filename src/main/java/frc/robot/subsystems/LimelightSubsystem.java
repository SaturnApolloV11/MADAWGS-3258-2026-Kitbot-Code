package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.LIMELIGHT_TABLE_NAME;
import static frc.robot.Constants.VisionConstants.HUB_TAG_ID;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_MAX;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_MIN;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_TY_INTERCEPT;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_TY_SLOPE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable limelightTable;
  private final NetworkTableEntry tvEntry;
  private final NetworkTableEntry txEntry;
  private final NetworkTableEntry tyEntry;
  private final NetworkTableEntry taEntry;
  private final NetworkTableEntry tidEntry;

  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);
    tvEntry = limelightTable.getEntry("tv");
    txEntry = limelightTable.getEntry("tx");
    tyEntry = limelightTable.getEntry("ty");
    taEntry = limelightTable.getEntry("ta");
    tidEntry = limelightTable.getEntry("tid");
  }

  public boolean hasTarget() {
    return tvEntry.getDouble(0.0) >= 1.0;
  }

  public double getTx() {
    return txEntry.getDouble(0.0);
  }

  public double getTy() {
    return tyEntry.getDouble(0.0);
  }

  public double getTargetArea() {
    return taEntry.getDouble(0.0);
  }

  public double getTagId() {
    return tidEntry.getDouble(-1.0);
  }

  public boolean hasHubTagTarget() {
    if (!hasTarget()) {
      return false;
    }
    double configuredTagId = SmartDashboard.getNumber("Vision/HubTagId", HUB_TAG_ID);
    return Math.abs(getTagId() - configuredTagId) < 0.5;
  }

  public double getRecommendedLaunchVoltage() {
    double slope = SmartDashboard.getNumber("Vision/LaunchTySlope", LAUNCH_VOLTS_TY_SLOPE);
    double intercept = SmartDashboard.getNumber("Vision/LaunchTyIntercept", LAUNCH_VOLTS_TY_INTERCEPT);
    double minVolts = SmartDashboard.getNumber("Vision/LaunchVoltsMin", LAUNCH_VOLTS_MIN);
    double maxVolts = SmartDashboard.getNumber("Vision/LaunchVoltsMax", LAUNCH_VOLTS_MAX);
    return MathUtil.clamp((slope * getTy()) + intercept, minVolts, maxVolts);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putBoolean("Vision/HasHubTagTarget", hasHubTagTarget());
    SmartDashboard.putNumber("Vision/TxDeg", getTx());
    SmartDashboard.putNumber("Vision/TyDeg", getTy());
    SmartDashboard.putNumber("Vision/TargetArea", getTargetArea());
    SmartDashboard.putNumber("Vision/TagId", getTagId());
    SmartDashboard.putNumber("Vision/RecommendedLaunchVolts", getRecommendedLaunchVoltage());
  }
}
