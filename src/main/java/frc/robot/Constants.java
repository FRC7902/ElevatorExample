// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public static class ElevatorConstants {
    // CAN IDs
    public static final int LEADER_MOTOR_CAN_ID = 1;
    public static final int FOLLOWER_MOTOR_CAN_ID = 2;

    // Current Limits
    public static final double STATOR_CURRENT_LIMIT = 70.0;
    public static final double SUPPLY_CURRENT_LIMIT = 50.0;

    // Physical Constants
    public static final double GEARING = 8.125;
    public static final double CARRIAGE_MASS = Units.lbsToKilograms(24);
    public static final double DRUM_RADIUS = Units.inchesToMeters(1);
    public static final double METERS_PER_MOTOR_ROTATION =
            (DRUM_RADIUS * 2 * Math.PI) / GEARING;

    // Elevator Dimensions
    public static final double HEIGHT_METERS = Units.inchesToMeters(42);
    public static final double MIN_HEIGHT_METERS = 0.2;
    public static final double MAX_HEIGHT_METERS = 1.27;

    // Motion Constraints
    public static final double MAX_VELOCITY =
            2 / METERS_PER_MOTOR_ROTATION;
    // rotations per second
    public static final  double  MOTION_MAGIC_ACCEL = 160;

    // PID Constants
    public static double PID_P = 20;
    public static double PID_I = 0;
    public static double PID_D = 5;

    public static double ERROR_METERS = 0.01;
  }
}
