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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

public static class ElevatorSubsystemConstants {
  public static final double kInchesPerMillimeters = 1/25.4;
  public static final double kElevatorPowerLimit = 0.4;
  public static final double kElevatorTolerance = 0.5;
  public static final double kElevatorPIDP = 0.0325;


  }

public static final class RobotContainerConstants {
  public static final int kElevatorMotorCANIDR = 7;
  public static final int kElevatorMotorCANIDL = 8;
  public static final int kElevatorTopLimitChannel = 9;
  public static final int kElevatorBottomLimitChannel = 8;
  public static final int kLaserCanCANID = 29;
}  
}
