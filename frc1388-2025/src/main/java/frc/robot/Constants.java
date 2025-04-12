// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class SwerveModuleConstants {
    // Drive train encoders
    public static final double SWERVE_MK4I_L2_RATIO = 6.75;
    public static final double SWERVE_MK4I_L3_RATIO = 6.12;
    public static final double WHEEL_ROTATIONS_PER_MOTOR_ROTATION = 1 / SWERVE_MK4I_L3_RATIO;
    public static final double WHEEL_DIAMETER_INCHES = 3.96;
    public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * 0.0254;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double DIST_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE * WHEEL_ROTATIONS_PER_MOTOR_ROTATION;

    public static final double DRIVE_MOTOR_P = 0.001;
    public static final double DRIVE_MOTOR_I = 0;
    public static final double DRIVE_MOTOR_D = 0;

    public static final double ROTATION_MOTOR_P = 0.009; //0.007
    public static final double ROTATION_MOTOR_I = 0;
    public static final double ROTATION_MOTOR_D = 0;

    public static final double ROTATION_TOLERANCE = 5;
  }

  public static class ElevatorSubsystemConstants {
    public static final double kElevatorPowerLimit = 1;
    public static final double kElevatorPowerLimitDown = 0.4;
    public static final double kElevatorEndRangePowerLimit = 0.1;

    public static final double kElevatorEndRange = 5.25; //inches 2
    public static final double kElevatorMaxHeight = 50.5; //inches
    public static final double kElevatorTopEndRange = kElevatorMaxHeight - 6.5; //inches
    public static final double kElevatorBottomEndRange = kElevatorEndRange; //inches

    public static final double kElevatorTolerance = 0;

    public static final double kElevatorPIDP = 0.085;
    public static final double kElevatorPIDI = 0;
    public static final double kElevatorPIDD = 0;

    public static final double kElevatorLimitSwitchZero = 0; //starting point after encoder set to 0 due to limit switch variability
    // public static final double kElevatorPIDP = 0.0325;
    public static final double kGearboxRatio = 9.0;       // input to output gear reduction
    public static final int    kSprocketToothCount = 22;
    public static final double kChainPitch = 0.25;        // inches
    public static final double kSprocketCircumference = kChainPitch * kSprocketToothCount;  // inches
    public static final double kChainInchesPerMotorRotation = kSprocketCircumference / kGearboxRatio;
    public static final double kCarriageInchesPerMotorRotation = kChainInchesPerMotorRotation * 2;  // carriage moves twice the rate of

    public static final double kSecondsPerMinute = 60;
    public static final double kDistancePerVelocityScale = kCarriageInchesPerMotorRotation / kSecondsPerMinute;
    public static final double kElevatorOffsetAccountSpeed = 0.5;
  }
  
  public static class ElevatorCommandConstants {
    public static final double kElevatorDeadband = 0.1;
  }
  
  public static class EndEffectorSubsystemConstants {
    public static final double kCoralDetectionHeight = 4; //in inches
    public static final double kInchesPerMillimeters = 1 / 25.4;
//    public static final int    kShooterMaxCurrent = 20;   // Amps
    public static final int    kShooterMaxCurrent = 40;   // Amps
  }

  public static class EndEffectorCommandConstants {
    public static final double kRightTriggerPressed = 0.1;
    public static final double kIntakeCoralPower = 0.5;
    public static final double kShootCoralPower = 0.9;    
    public static final double kIntakeKillDelay = 0.1; //in seconds    

    public static final double kGearboxRatio = 9.0;       // input to output gear reduction
    public static final int    kSprocketToothCount = 22;
    public static final double kChainPitch = 0.25;        // inches
    public static final double kSprocketCircumference = kChainPitch * kSprocketToothCount;  // inches
    public static final double kChainInchesPerMotorRotation = kSprocketCircumference / kGearboxRatio;
    public static final double kCarriageInchesPerMotorRotation = kChainInchesPerMotorRotation * 2;  // carriage moves twice the rate of the chain

    public static final int    kEndCount = 25;
  }

  public static class RobotConstants { 
    public static final double ROBOT_WIDTH = 0.5981; // 0.9114
    public static final double ROBOT_LENGTH = 0.5981;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = Units.feetToMeters(18.9); // 5.76 l3+ meters per second
    public static final double DT_SECONDS = 0.02; // 20ms per tick
    public static final double DISTANCE_PER_TICK = ROBOT_MAX_SPEED * DT_SECONDS; // 20ms per tick

    public static final int FRONT_RIGHT_DRIVE_MOTOR_CANID = 1;
    public static final int FRONT_RIGHT_ROTATION_MOTOR_CANID = 5;
    public static final int FRONT_RIGHT_CANCODER = 9;

    public static final String FRONT_RIGHT_ENCODER_OFFSET_KEY = "2025/frontRightEncoderOffset";

    public static final int FRONT_LEFT_DRIVE_MOTOR_CANID = 2;
    public static final int FRONT_LEFT_ROTATION_MOTOR_CANID = 6;
    public static final int FRONT_LEFT_CANCODER = 10;

    public static final String FRONT_LEFT_ENCODER_OFFSET_KEY = "2025/frontLeftEncoderOffset";

    public static final int BACK_LEFT_DRIVE_MOTOR_CANID = 3;
    public static final int BACK_LEFT_ROTATION_MOTOR_CANID = 7;
    public static final int BACK_LEFT_CANCODER = 11;

    public static final String BACK_LEFT_ENCODER_OFFSET_KEY = "2025/backLeftEncoderOffset";

    public static final int BACK_RIGHT_DRIVE_MOTOR_CANID = 4;
    public static final int BACK_RIGHT_ROTATION_MOTOR_CANID = 8;
    public static final int BACK_RIGHT_CANCODER = 12;

    public static final String BACK_RIGHT_ENCODER_OFFSET_KEY = "2025/backRightEncoderOffset";

    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double MANUAL_CONTROL_ANGLE_DEADBAND = 0.5;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;

    public static final int PIGEON_CANID = 13;
  }

  public class AutoConstants {

    // blue side
    public static final Pose2d SCORING_POSITION_1_RIGHT_BLUE = new Pose2d(3.2004, 3.8616, new Rotation2d(0));
    public static final Pose2d SCORING_POSITION_1_LEFT_BLUE = new Pose2d(3.2004, 4.1902, new Rotation2d(0));
    public static final Pose2d SCORING_POSITION_2_LEFT_BLUE = new Pose2d(3.7026, 2.9917, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_2_RIGHT_BLUE = new Pose2d(3.9872, 2.8274, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_3_LEFT_BLUE = new Pose2d(4.9917, 2.8274, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_3_RIGHT_BLUE = new Pose2d(5.2763, 2.9917, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_4_LEFT_BLUE = new Pose2d(5.7785, 3.8616, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_4_RIGHT_BLUE = new Pose2d(5.7785, 4.1902, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_5_LEFT_BLUE = new Pose2d(5.2763, 5.0601, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_5_RIGHT_BLUE = new Pose2d(4.9917, 5.2244, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_6_LEFT_BLUE = new Pose2d(3.9872, 5.2244, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_6_RIGHT_BLUE = new Pose2d(3.7026, 5.0601, new Rotation2d(Math.toRadians(300)));

    // red side
    public static final Pose2d SCORING_POSITION_1_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_1_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_1_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_1_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_1_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_1_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_2_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_2_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_2_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_2_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_2_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_2_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_3_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_3_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_3_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_3_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_3_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_3_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_4_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_4_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_4_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d SCORING_POSITION_4_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_4_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_4_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d SCORING_POSITION_5_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_5_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_5_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_5_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_5_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_5_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_6_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_6_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_6_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_6_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_6_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_6_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));

    // blue side
    public static final Pose2d L1_POSITION_1_LEFT_BLUE = new Pose2d(3.2004, 4.4061, new Rotation2d(0));
    public static final Pose2d L1_POSITION_1_RIGHT_BLUE = new Pose2d(3.2004, 3.6457, new Rotation2d(0));
    public static final Pose2d L1_POSITION_2_LEFT_BLUE = new Pose2d(3.5156, 3.0996, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d L1_POSITION_2_RIGHT_BLUE = new Pose2d(4.1742, 2.7195, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d L1_POSITION_3_LEFT_BLUE = new Pose2d(4.8047, 2.7195, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d L1_POSITION_3_RIGHT_BLUE = new Pose2d(5.4633, 3.0996, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d L1_POSITION_4_LEFT_BLUE = new Pose2d(5.7785, 3.6457, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d L1_POSITION_4_RIGHT_BLUE = new Pose2d(5.7785, 4.4061, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d L1_POSITION_5_LEFT_BLUE = new Pose2d(5.4633, 4.9522, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d L1_POSITION_5_RIGHT_BLUE = new Pose2d(4.8047, 5.3323, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d L1_POSITION_6_LEFT_BLUE = new Pose2d(4.1742, 5.3323, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d L1_POSITION_6_RIGHT_BLUE = new Pose2d(3.5156, 4.9522, new Rotation2d(Math.toRadians(300)));

    public static final Pose2d CLIMBER_POSITION_LEFT_BLUE = new Pose2d(7.769, 7.215, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d CLIMBER_POSITION_MIDDLE_BLUE = new Pose2d(7.769, 6.135, new Rotation2d(Math.toRadians(270)));
    public static final Pose2d CLIMBER_POSITION_RIGHT_BLUE = new Pose2d(7.769, 5.055, new Rotation2d(Math.toRadians(270)));


    // red side
    public static final Pose2d L1_POSITION_1_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_1_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_1_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d L1_POSITION_1_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_1_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_1_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d L1_POSITION_2_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_2_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_2_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));
    public static final Pose2d L1_POSITION_2_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_2_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_2_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));
    public static final Pose2d L1_POSITION_3_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_3_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_3_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d L1_POSITION_3_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_3_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_3_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d L1_POSITION_4_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_4_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_4_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d L1_POSITION_4_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_4_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_4_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d L1_POSITION_5_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_5_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_5_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d L1_POSITION_5_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_5_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_5_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d L1_POSITION_6_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_6_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_6_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));
    public static final Pose2d L1_POSITION_6_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - L1_POSITION_6_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - L1_POSITION_6_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));

    public static final Pose2d CLIMBER_POSITION_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - CLIMBER_POSITION_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - CLIMBER_POSITION_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(90)));
    public static final Pose2d CLIMBER_POSITION_MIDDLE_RED = new Pose2d(FieldLayout.FIELD_LENGTH - CLIMBER_POSITION_MIDDLE_BLUE.getX(), FieldLayout.FIELD_WIDTH - CLIMBER_POSITION_MIDDLE_BLUE.getY(), new Rotation2d(Math.toRadians(90)));
    public static final Pose2d CLIMBER_POSITION_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - CLIMBER_POSITION_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - CLIMBER_POSITION_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(90)));


    public static final Pose2d[] SETPOINTS_RIGHT = new Pose2d[12];

    static {
    //blue side
    SETPOINTS_RIGHT[0] = SCORING_POSITION_1_RIGHT_BLUE;
    SETPOINTS_RIGHT[1] = SCORING_POSITION_2_RIGHT_BLUE;
    SETPOINTS_RIGHT[2] = SCORING_POSITION_3_RIGHT_BLUE;
    SETPOINTS_RIGHT[3] = SCORING_POSITION_4_RIGHT_BLUE;
    SETPOINTS_RIGHT[4] = SCORING_POSITION_5_RIGHT_BLUE;
    SETPOINTS_RIGHT[5] = SCORING_POSITION_6_RIGHT_BLUE;

    //red
    SETPOINTS_RIGHT[6] = SCORING_POSITION_1_RIGHT_RED;
    SETPOINTS_RIGHT[7] = SCORING_POSITION_2_RIGHT_RED;
    SETPOINTS_RIGHT[8] = SCORING_POSITION_3_RIGHT_RED;
    SETPOINTS_RIGHT[9] = SCORING_POSITION_4_RIGHT_RED;
    SETPOINTS_RIGHT[10] = SCORING_POSITION_5_RIGHT_RED;
    SETPOINTS_RIGHT[11] = SCORING_POSITION_6_RIGHT_RED;
    }

    public static final Pose2d[] SETPOINTS_LEFT = new Pose2d[12];
    
    static {
      //blue
      SETPOINTS_LEFT[0] = SCORING_POSITION_1_LEFT_BLUE;
      SETPOINTS_LEFT[1] = SCORING_POSITION_2_LEFT_BLUE;
      SETPOINTS_LEFT[2] = SCORING_POSITION_3_LEFT_BLUE;
      SETPOINTS_LEFT[3] = SCORING_POSITION_4_LEFT_BLUE;
      SETPOINTS_LEFT[4] = SCORING_POSITION_5_LEFT_BLUE;
      SETPOINTS_LEFT[5] = SCORING_POSITION_6_LEFT_BLUE;

      //red
      SETPOINTS_LEFT[6] = SCORING_POSITION_1_LEFT_RED;
      SETPOINTS_LEFT[7] = SCORING_POSITION_2_LEFT_RED;
      SETPOINTS_LEFT[8] = SCORING_POSITION_3_LEFT_RED;
      SETPOINTS_LEFT[9] = SCORING_POSITION_4_LEFT_RED;
      SETPOINTS_LEFT[10] = SCORING_POSITION_5_LEFT_RED;
      SETPOINTS_LEFT[11] = SCORING_POSITION_6_LEFT_RED;
    }

    public static final Pose2d[] SETPOINTS_L1 = new Pose2d[30];
    
    static {
      //blue
      SETPOINTS_L1[0] = L1_POSITION_1_LEFT_BLUE;
      SETPOINTS_L1[1] = L1_POSITION_1_RIGHT_BLUE;
      SETPOINTS_L1[2] = L1_POSITION_2_LEFT_BLUE;
      SETPOINTS_L1[3] = L1_POSITION_2_RIGHT_BLUE;
      SETPOINTS_L1[4] = L1_POSITION_3_LEFT_BLUE;
      SETPOINTS_L1[5] = L1_POSITION_3_RIGHT_BLUE;
      SETPOINTS_L1[6] = L1_POSITION_4_LEFT_BLUE;
      SETPOINTS_L1[7] = L1_POSITION_4_RIGHT_BLUE;
      SETPOINTS_L1[8] = L1_POSITION_5_LEFT_BLUE;
      SETPOINTS_L1[9] = L1_POSITION_5_RIGHT_BLUE;
      SETPOINTS_L1[10] = L1_POSITION_6_LEFT_BLUE;
      SETPOINTS_L1[11] = L1_POSITION_6_RIGHT_BLUE;

      //red
      SETPOINTS_L1[12] =  L1_POSITION_1_LEFT_RED;
      SETPOINTS_L1[13] =  L1_POSITION_1_RIGHT_RED;
      SETPOINTS_L1[14] =  L1_POSITION_2_LEFT_RED;
      SETPOINTS_L1[15] =  L1_POSITION_2_RIGHT_RED;
      SETPOINTS_L1[16] =  L1_POSITION_3_LEFT_RED;
      SETPOINTS_L1[17] =  L1_POSITION_3_RIGHT_RED;
      SETPOINTS_L1[18] =  L1_POSITION_4_LEFT_RED;
      SETPOINTS_L1[19] =  L1_POSITION_4_RIGHT_RED;
      SETPOINTS_L1[20] = L1_POSITION_5_LEFT_RED;
      SETPOINTS_L1[21] = L1_POSITION_5_RIGHT_RED;
      SETPOINTS_L1[22] = L1_POSITION_6_LEFT_RED;
      SETPOINTS_L1[23] = L1_POSITION_6_RIGHT_RED;

      SETPOINTS_L1[24] = CLIMBER_POSITION_LEFT_BLUE;
      SETPOINTS_L1[25] = CLIMBER_POSITION_MIDDLE_BLUE;
      SETPOINTS_L1[26] = CLIMBER_POSITION_RIGHT_BLUE;
      SETPOINTS_L1[27] = CLIMBER_POSITION_LEFT_RED;
      SETPOINTS_L1[28] = CLIMBER_POSITION_MIDDLE_RED;
      SETPOINTS_L1[29] = CLIMBER_POSITION_RIGHT_RED;
    }

    public enum Objective {
      LAYINGEGGSBOTTOM("3L4Right"),
      LAYINGEGGSTOP("3L4Left"),
      TWOSCORERIGHT("2L4Left"),
      TWOSCORELEFT("2L4Right"),
      ONESCORECENTER("1L4Cent"),
      ONESCORELEFT("1L4Left"),
      ONESCORERIGHT("1L4Right"),
      LEAVE("LeaveCent"),
      CHOREOAUTOROUTINE("choreoAutoRoutine");

      public static final Objective Default = CHOREOAUTOROUTINE;

      private String m_dashboardDescript; // This is what will show on dashboard

      private Objective(String dashboardDescript) {
        m_dashboardDescript = dashboardDescript;
      }

      public String getDashboardDescript() {
        return m_dashboardDescript;
      }
    }
  }

  public static final class RobotContainerConstants {
    public static final int kElevatorMotorCANID = 20;
    public static final int kElevatorTopLimitChannel = 9;
    public static final int kElevatorBottomLimitChannel = 8;
    public static final int kLaserCanCANID = 29;
    public static final int kEndEffectorCANID = 30;
    public static final int kEndEffectorLimitChannel = 7;

    public static final int kClimberMotorCANID = 40;
    public static final int kClimberAbsoluteEncoderDIO = 6;
  }

 

  public static final class ClimberConstants {
    public static final double LOWER_PERCENTAGE_ABSOLUTE_ENCODER = 1.0/1024.0;
    public static final double HIGHER_PERCENTAGE_ABSOLUTE_ENCODER =  1023.0/1024.0;
    public static final double DEGREES_PER_ROTATION = 360;

    public static final double BOTTOM_LIMIT = 0.79;
    public static final double TOP_LIMIT = 0.3;

    public static final double CLIMBER_CONTROLLER_DEADBAND = 0.1;

    public static final double CLIMBER_UP_POSITION = 0.305;
    public static final double CLIMBER_DOWN_POSITION = 0.785;
    public static final double CLIMBER_CENTER_POSITION = 0.5;


    public static final double CLIMBER_POWER_LIMIT = 0.75;

    public static final double CLIMBER_ABSOLUTE_ENCODER_OFFSET = 0;
  }
  
    public static class LimelightConstants {

      public static final int BOTPOSE_X = 0;
      public static final int BOTPOSE_Y = 1;
      public static final int BOTPOSE_Z = 2;
      public static final int BOTPOSE_ROLL = 3;
      public static final int BOTPOSE_PITCH = 4;
      public static final int BOTPOSE_YAW = 5;
      public static final int BOTPOSE_LATENCY = 6;
      public static final int BOTPOSE_TOTAL_APRILTAGS_SEEN = 7;
      public static final int BOTPOSE_DISTANCE_BETWEEN_TAGS = 8;
      public static final int BOTPOSE_AVERAGE_DISTANCE_FROM_CAMERA = 9;
      public static final int BOTPOSE_AVERAGE_TAG_AREA = 10;
      public static final int BOTPOSE_TAG_ID = 11;
      public static final int BOTPOSE_TX_RAW_TARGET_ANGLE = 12;
      public static final int BOTPOSE_TY_RAW_TARGET_ANGLE = 13;
      public static final int BOTPOSE_TARGET_AREA = 14;
      public static final int BOTPOSE_DISTANCE_TO_CAMERA = 15;
      public static final int BOTPOSE_DISTANCE_TO_ROBOT = 16;
      public static final int BOTPOSE_AMBIGUITY = 17;

      public static final double LED_USE_PIPELINE = 0;
      public static final double LED_FORCE_OFF = 1;
      public static final double LED_FORCE_BLINK = 2;
      public static final double LED_FORCE_ON = 3;
    }

    public static class FieldLayout {
      public static double FIELD_LENGTH = Units.inchesToMeters(690.875);
      public static double FIELD_WIDTH = Units.inchesToMeters(317);
    }
}
