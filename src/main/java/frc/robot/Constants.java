// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_TABLE_NAME = "limelight";
    public static final String LIMELIGHT_STREAM_URL = "http://limelight.local:5800/stream.mjpg";
    public static final double HUB_TAG_ID = 7.0;

    public static final boolean AUTO_LAUNCH_ADJUST_ENABLED = true;
    public static final double LAUNCH_VOLTS_TY_SLOPE = -0.08;
    public static final double LAUNCH_VOLTS_TY_INTERCEPT = 10.6;
    public static final double LAUNCH_VOLTS_MIN = 8.5;
    public static final double LAUNCH_VOLTS_MAX = 12.0;

    public static final double AIM_TX_KP = 0.035;
    public static final double AIM_ROTATION_MAX = 0.45;
    public static final double AIM_TX_DEADBAND_DEG = 1.0;
  }

  public static final class AutoConstants {
    public static final double AUTO_SPINUP_SECONDS = 1.0;
    public static final double AUTO_LAUNCH_SECONDS = 2.0;
    public static final double AUTO_DRIVE_BACK_SPEED = 0.5;
    public static final double AUTO_DRIVE_BACK_SECONDS = 1.5;

    public static final double AUTO_FIRST_SPINUP_SECONDS = 1.0;
    public static final double AUTO_FIRST_LAUNCH_SECONDS = 5.5;
    public static final double AUTO_TO_NEUTRAL_SPEED = 0.65;
    public static final double AUTO_TO_NEUTRAL_SECONDS = 2.35;
    public static final double AUTO_NEUTRAL_PICKUP_SECONDS = 0.65;
    public static final double AUTO_RETURN_SPEED = -0.62;
    public static final double AUTO_RETURN_SECONDS = 2.4;
    public static final double AUTO_SECOND_SPINUP_SECONDS = 0.75;
    public static final double AUTO_SECOND_LAUNCH_SECONDS = 3.8;
    public static final double AUTO_SETTLE_SECONDS = 0.2;
  }
}
