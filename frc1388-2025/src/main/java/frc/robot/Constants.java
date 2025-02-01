// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }


  public static class SwerveModuleConstants {
    public static final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192); // ask calvin about the math

    public static final double DIST_PER_MOTOR_ROTATION  = 5.65 / 131.14; // calvins magic numbers

    public static final double DRIVE_MOTOR_P = 0.001;
    public static final double DRIVE_MOTOR_I = 0;
    public static final double DRIVE_MOTOR_D = 0;

    public static final double ROTATION_MOTOR_P = 0.007;
    public static final double ROTATION_MOTOR_I = 0;
    public static final double ROTATION_MOTOR_D = 0;

    public static final double ROTATION_TOLERANCE = 5;
  }

public static class ElevatorSubsystemConstants {
  public static final double kInchesPerMillimeters = 1/25.4;
  public static final double kElevatorPowerLimit = 0.5;
  public static final double kElevatorEndRangePowerLimit = 0.1;
  public static final double kElevatorEndRange = 5.25; //inches 2
  public static final double kElevatorMaxHeight = 37; //inches
  public static final double kElevatorTopEndRange = kElevatorMaxHeight - kElevatorEndRange; //inches
  public static final double kElevatorBottomEndRange = kElevatorEndRange; //inches

  public static final double kElevatorTolerance = 0;

  public static final double kElevatorPIDP = 0.185;
  public static final double kElevatorPIDI = 0;
  public static final double kElevatorPIDD = 0;
  public static final double kElevatorLimitSwitchZero = 0.25; //starting point after encoder set to 0 due to limit switch variability
  // public static final double kElevatorPIDP = 0.0325;
  public static final double kTicksPerInch = 1;

  }


  public static class RobotConstants {
    public static final double ROBOT_WIDTH = 0.9144;
    public static final double ROBOT_LENGTH = 0.9144;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = 4.0; // 5.76 l3+ meters per second

    public static final int FRONT_RIGHT_DRIVE_MOTOR_CANID = 1;
    public static final int FRONT_RIGHT_ROTATION_MOTOR_CANID = 5;
    public static final int FRONT_RIGHT_CANCODER = 9;
    public static final String FRONT_RIGHT_ENCODER_OFFSET_KEY = "2024/frontRightEncoderOffset";

    public static final int FRONT_LEFT_DRIVE_MOTOR_CANID = 2;
    public static final int FRONT_LEFT_ROTATION_MOTOR_CANID = 6;
    public static final int FRONT_LEFT_CANCODER = 10;
    public static final String FRONT_LEFT_ENCODER_OFFSET_KEY = "2024/frontLeftEncoderOffset";

    public static final int BACK_LEFT_DRIVE_MOTOR_CANID = 3;
    public static final int BACK_LEFT_ROTATION_MOTOR_CANID = 7;
    public static final int BACK_LEFT_CANCODER = 11;
    public static final String BACK_LEFT_ENCODER_OFFSET_KEY = "2024/backLeftEncoderOffset";

    public static final int BACK_RIGHT_DRIVE_MOTOR_CANID = 4;
    public static final int BACK_RIGHT_ROTATION_MOTOR_CANID = 8;
    public static final int BACK_RIGHT_CANCODER = 12;
    public static final String BACK_RIGHT_ENCODER_OFFSET_KEY = "2024/backRightEncoderOffset";

    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double MANUAL_CONTROL_ANGLE_DEADBAND = 0.5;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;
  }




public static final class RobotContainerConstants {
  public static final int kElevatorMotorCANIDR = 13;
  public static final int kElevatorMotorCANIDL = 14;
  public static final int kElevatorTopLimitChannel = 9;
  public static final int kElevatorBottomLimitChannel = 8;
  public static final int kLaserCanCANID = 29;
}  

public static final class ElevatorCommandConstants {
  public static final double kElevatorDeadband = 0.1;
}
}
