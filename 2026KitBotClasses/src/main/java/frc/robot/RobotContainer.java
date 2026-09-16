// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.AutoConstants.AUTO_FIRST_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_FIRST_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_DRIVE_BACK_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_DRIVE_BACK_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_NEUTRAL_PICKUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_RETURN_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_RETURN_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_RIGHT_TO_NEUTRAL_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_RIGHT_TO_NEUTRAL_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_RIGHT_TO_NEUTRAL_TURN;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_LAUNCH_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SECOND_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SETTLE_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_SPINUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_OUT_TURN;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_PICKUP_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_SPEED;
import static frc.robot.Constants.AutoConstants.AUTO_LEFT_MIDDLE_RETURN_TURN;
import static frc.robot.Constants.AutoConstants.AUTO_TO_NEUTRAL_SECONDS;
import static frc.robot.Constants.AutoConstants.AUTO_TO_NEUTRAL_SPEED;
import static frc.robot.Constants.FuelConstants.INTAKING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKING_INTAKE_VOLTAGE;
import static frc.robot.Constants.VisionConstants.AIM_ROTATION_MAX;
import static frc.robot.Constants.VisionConstants.AIM_TX_DEADBAND_DEG;
import static frc.robot.Constants.VisionConstants.AIM_TX_KP;
import static frc.robot.Constants.VisionConstants.AUTO_LAUNCH_ADJUST_ENABLED;
import static frc.robot.Constants.VisionConstants.HUB_TAG_ID;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_MAX;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_MIN;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_TY_INTERCEPT;
import static frc.robot.Constants.VisionConstants.LAUNCH_VOLTS_TY_SLOPE;
import static frc.robot.Constants.VisionConstants.LAUNCH_RPM_MAX;
import static frc.robot.Constants.VisionConstants.LAUNCH_RPM_MIN;
import static frc.robot.Constants.VisionConstants.LAUNCH_RPM_TY_INTERCEPT;
import static frc.robot.Constants.VisionConstants.LAUNCH_RPM_TY_SLOPE;
import static frc.robot.Constants.VisionConstants.SWEET_SPOT_TY_CENTER_DEG;
import static frc.robot.Constants.VisionConstants.SWEET_SPOT_TY_TOLERANCE_DEG;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_STREAM_URL;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_TABLE_NAME;
import java.util.Set;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAimAssist;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LeftDepotAuto;
import frc.robot.commands.Launch;
import frc.robot.commands.LeftNeutralAuto;
import frc.robot.commands.PreloadMobilityTimedAuto;
import frc.robot.commands.RightNeutralAuto;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerPathPlannerCommands();
    publishAutoTunables();
    publishVisionConfig();
    setupUsbDriverCamera();
    setupLimelightStream();
    configureAutonomousChooser();
    setupShuffleboardWidgets();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Allow both driver and operator controllers to run fuel actions for pit testing.
    operatorController.leftBumper().or(driverController.leftBumper()).whileTrue(new Intake(fuelSubsystem));
    operatorController.rightBumper().or(driverController.rightBumper())
        .whileTrue(new SpinUp(fuelSubsystem)
            .withTimeout(frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS)
            .andThen(new Launch(fuelSubsystem, limelightSubsystem))
            .finallyDo(() -> fuelSubsystem.stop()));
    operatorController.a().or(driverController.a()).whileTrue(new Eject(fuelSubsystem));

    // While the driver left bumper is held, use Limelight horizontal error for
    // rotation auto-aim assist.
    driverController.leftBumper().whileTrue(new DriveAimAssist(driveSubsystem, limelightSubsystem, driverController));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the currently selected autonomous routine from SmartDashboard.
    return Autos.primaryAuto(driveSubsystem, fuelSubsystem);
  }

  private void publishAutoTunables() {
    SmartDashboard.putString("Auto/Stage", "Idle");

    SmartDashboard.putNumber("Auto/SpinUpSeconds", AUTO_SPINUP_SECONDS);
    SmartDashboard.putNumber("Auto/LaunchSeconds", AUTO_LAUNCH_SECONDS);
    SmartDashboard.putNumber("Auto/DriveBackSpeed", AUTO_DRIVE_BACK_SPEED);
    SmartDashboard.putNumber("Auto/DriveBackSeconds", AUTO_DRIVE_BACK_SECONDS);

    SmartDashboard.putNumber("Auto/FirstSpinUpSeconds", AUTO_FIRST_SPINUP_SECONDS);
    SmartDashboard.putNumber("Auto/FirstLaunchSeconds", AUTO_FIRST_LAUNCH_SECONDS);
    SmartDashboard.putNumber("Auto/ToNeutralSpeed", AUTO_TO_NEUTRAL_SPEED);
    SmartDashboard.putNumber("Auto/ToNeutralSeconds", AUTO_TO_NEUTRAL_SECONDS);
    SmartDashboard.putNumber("Auto/NeutralPickupSeconds", AUTO_NEUTRAL_PICKUP_SECONDS);
    SmartDashboard.putNumber("Auto/ReturnSpeed", AUTO_RETURN_SPEED);
    SmartDashboard.putNumber("Auto/ReturnSeconds", AUTO_RETURN_SECONDS);
    SmartDashboard.putNumber("Auto/SecondSpinUpSeconds", AUTO_SECOND_SPINUP_SECONDS);
    SmartDashboard.putNumber("Auto/SecondLaunchSeconds", AUTO_SECOND_LAUNCH_SECONDS);
    SmartDashboard.putNumber("Auto/SettleSeconds", AUTO_SETTLE_SECONDS);
    SmartDashboard.putNumber("Auto/LeftMiddleOutSpeed", AUTO_LEFT_MIDDLE_OUT_SPEED);
    SmartDashboard.putNumber("Auto/LeftMiddleOutTurn", AUTO_LEFT_MIDDLE_OUT_TURN);
    SmartDashboard.putNumber("Auto/LeftMiddleOutSeconds", AUTO_LEFT_MIDDLE_OUT_SECONDS);
    SmartDashboard.putNumber("Auto/LeftMiddlePickupSeconds", AUTO_LEFT_MIDDLE_PICKUP_SECONDS);
    SmartDashboard.putNumber("Auto/LeftMiddleReturnSpeed", AUTO_LEFT_MIDDLE_RETURN_SPEED);
    SmartDashboard.putNumber("Auto/LeftMiddleReturnTurn", AUTO_LEFT_MIDDLE_RETURN_TURN);
    SmartDashboard.putNumber("Auto/LeftMiddleReturnSeconds", AUTO_LEFT_MIDDLE_RETURN_SECONDS);
    SmartDashboard.putNumber("Auto/RightToNeutralSpeed", AUTO_RIGHT_TO_NEUTRAL_SPEED);
    SmartDashboard.putNumber("Auto/RightToNeutralTurn", AUTO_RIGHT_TO_NEUTRAL_TURN);
    SmartDashboard.putNumber("Auto/RightToNeutralSeconds", AUTO_RIGHT_TO_NEUTRAL_SECONDS);
  }

  private void publishVisionConfig() {
    SmartDashboard.putString("Vision/TableName", LIMELIGHT_TABLE_NAME);
    SmartDashboard.putString("Vision/StreamUrl", LIMELIGHT_STREAM_URL);
    SmartDashboard.putBoolean("Vision/AutoLaunchAdjustEnabled", AUTO_LAUNCH_ADJUST_ENABLED);
    SmartDashboard.putNumber("Vision/LaunchTySlope", LAUNCH_VOLTS_TY_SLOPE);
    SmartDashboard.putNumber("Vision/LaunchTyIntercept", LAUNCH_VOLTS_TY_INTERCEPT);
    SmartDashboard.putNumber("Vision/LaunchVoltsMin", LAUNCH_VOLTS_MIN);
    SmartDashboard.putNumber("Vision/LaunchVoltsMax", LAUNCH_VOLTS_MAX);
    SmartDashboard.putNumber("Vision/LaunchRpmTySlope", LAUNCH_RPM_TY_SLOPE);
    SmartDashboard.putNumber("Vision/LaunchRpmTyIntercept", LAUNCH_RPM_TY_INTERCEPT);
    SmartDashboard.putNumber("Vision/LaunchRpmMin", LAUNCH_RPM_MIN);
    SmartDashboard.putNumber("Vision/LaunchRpmMax", LAUNCH_RPM_MAX);
    SmartDashboard.putNumber("Vision/SweetSpotTyCenterDeg", SWEET_SPOT_TY_CENTER_DEG);
    SmartDashboard.putNumber("Vision/SweetSpotTyToleranceDeg", SWEET_SPOT_TY_TOLERANCE_DEG);
    SmartDashboard.putNumber("Vision/AimTxKp", AIM_TX_KP);
    SmartDashboard.putNumber("Vision/AimRotationMax", AIM_ROTATION_MAX);
    SmartDashboard.putNumber("Vision/AimTxDeadbandDeg", AIM_TX_DEADBAND_DEG);
    SmartDashboard.putNumber("Vision/HubTagId", HUB_TAG_ID);
  }

  private void setupLimelightStream() {
    try {
      CameraServer.addCamera(new HttpCamera("Limelight", LIMELIGHT_STREAM_URL));
    } catch (RuntimeException ignored) {
      // Leave stream setup optional so robot code runs even if camera stream is unavailable.
    }
  }

  private void setupUsbDriverCamera() {
    // USB cameras may re-enumerate between boots, so try common indices.
    for (int deviceIndex : new int[] { 0, 1 }) {
      try {
        UsbCamera usbCamera = CameraServer.startAutomaticCapture("DriverCam", deviceIndex);
        usbCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        usbCamera.setResolution(320, 240);
        usbCamera.setFPS(20);
        applyDriverCamQuarterTurnClockwise(usbCamera);
        SmartDashboard.putNumber("Vision/DriverCamDeviceIndex", deviceIndex);
        return;
      } catch (RuntimeException ignored) {
        // Try the next index.
      }
    }
    // Leave stream setup optional so robot code runs even if no USB camera is connected.
  }

  private void applyDriverCamQuarterTurnClockwise(UsbCamera usbCamera) {
    // Not all UVC cameras expose rotation controls; try common property names.
    var rotate = usbCamera.getProperty("rotate");
    if (rotate.isValid()) {
      rotate.set(90);
      SmartDashboard.putString("Vision/DriverCamRotationApplied", "rotate=90");
      return;
    }

    var rotation = usbCamera.getProperty("rotation");
    if (rotation.isValid()) {
      rotation.set(90);
      SmartDashboard.putString("Vision/DriverCamRotationApplied", "rotation=90");
      return;
    }

    SmartDashboard.putString("Vision/DriverCamRotationApplied", "unsupported");
  }

  private void setupShuffleboardWidgets() {
    try {
      ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
      autoTab.add("Auto Mode", autoChooser)
          .withWidget(BuiltInWidgets.kComboBoxChooser)
          .withSize(4, 1)
          .withPosition(0, 0);

      ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
      visionTab.add("Driver Camera", CameraServer.getVideo("DriverCam"))
          .withWidget(BuiltInWidgets.kCameraStream)
          .withSize(6, 4)
          .withPosition(0, 0);
      visionTab.addCamera("Limelight Stream", "Limelight", LIMELIGHT_STREAM_URL)
          .withSize(6, 4)
          .withPosition(6, 0);
    } catch (RuntimeException ignored) {
      // Keep widget setup non-fatal if Shuffleboard is unavailable.
    }
  }

  private void configureAutonomousChooser() {
    // Set the options to show up in the Dashboard for selecting auto modes.
    autoChooser.setDefaultOption("LeftDepotAuto (Timed)",
        Commands.defer(() -> new LeftDepotAuto(driveSubsystem, fuelSubsystem),
            Set.of(driveSubsystem, fuelSubsystem)));
    autoChooser.addOption("LeftDepotAuto (PathPlanner)",
        AutoBuilder.buildAuto("LeftDepotAuto"));
    autoChooser.addOption("LeftNeutralAuto (Timed)",
        Commands.defer(() -> new LeftNeutralAuto(driveSubsystem, fuelSubsystem),
            Set.of(driveSubsystem, fuelSubsystem)));
    autoChooser.addOption("LeftNeutralAuto (PathPlanner)",
        AutoBuilder.buildAuto("LeftNeutralAuto"));
    autoChooser.addOption("RightNeutralAuto (PathPlanner)",
        AutoBuilder.buildAuto("RightNeutralAuto"));
    autoChooser.addOption("Preload + Mobility (Timed)",
        Commands.defer(() -> new PreloadMobilityTimedAuto(driveSubsystem, fuelSubsystem),
            Set.of(driveSubsystem, fuelSubsystem)));
    autoChooser.addOption("RightNeutralAuto (Timed)",
        Commands.defer(() -> new RightNeutralAuto(driveSubsystem, fuelSubsystem),
            Set.of(driveSubsystem, fuelSubsystem)));
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void registerPathPlannerCommands() {
    NamedCommands.registerCommand("FirstShoot",
        Commands.sequence(
            new SpinUp(fuelSubsystem)
                .withTimeout(SmartDashboard.getNumber("Auto/FirstSpinUpSeconds", AUTO_FIRST_SPINUP_SECONDS)),
            new Launch(fuelSubsystem, limelightSubsystem)
                .withTimeout(SmartDashboard.getNumber("Auto/FirstLaunchSeconds", AUTO_FIRST_LAUNCH_SECONDS))));

    NamedCommands.registerCommand("SecondShoot",
        Commands.sequence(
            new SpinUp(fuelSubsystem)
                .withTimeout(SmartDashboard.getNumber("Auto/SecondSpinUpSeconds", AUTO_SECOND_SPINUP_SECONDS)),
            new Launch(fuelSubsystem, limelightSubsystem)
                .withTimeout(SmartDashboard.getNumber("Auto/SecondLaunchSeconds", AUTO_SECOND_LAUNCH_SECONDS))));

    NamedCommands.registerCommand("IntakeOn",
        Commands.runOnce(() -> {
          fuelSubsystem.setIntakeLauncherRoller(
              SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
          fuelSubsystem.setFeederRoller(
              SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
        }));

    NamedCommands.registerCommand("IntakeOff", Commands.runOnce(fuelSubsystem::stop));
  }
}










