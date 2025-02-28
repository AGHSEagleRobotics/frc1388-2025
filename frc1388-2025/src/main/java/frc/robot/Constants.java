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
    public static final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192); // ask calvin about the math

    public static final double DIST_PER_MOTOR_ROTATION = 5.65 / 131.14; // calvins magic numbers

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
    //TODO create constants for endeffectorsubsystem and commands
  }

  public static class EndEffectorCommandConstants {
    public static final double kRightTriggerPressed = 0.1;
    public static final double kIntakeCoralPower = 0.4;
    public static final double kShootCoralPower = 0.6;    
    public static final double kIntakeKillDelay = 0.1; //in seconds    

    public static final double kGearboxRatio = 9.0;       // input to output gear reduction
    public static final int    kSprocketToothCount = 22;
    public static final double kChainPitch = 0.25;        // inches
    public static final double kSprocketCircumference = kChainPitch * kSprocketToothCount;  // inches
    public static final double kChainInchesPerMotorRotation = kSprocketCircumference / kGearboxRatio;
    public static final double kCarriageInchesPerMotorRotation = kChainInchesPerMotorRotation * 2;  // carriage moves twice the rate of the chain
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
  }

  public class AutoConstants {

    // blue side
    public static final Pose2d SCORING_POSITION_1_RIGHT_BLUE = new Pose2d(3.22, 3.859, new Rotation2d(0));
    public static final Pose2d SCORING_POSITION_1_LEFT_BLUE = new Pose2d(3.22, 4.172, new Rotation2d(0));
    public static final Pose2d SCORING_POSITION_2_LEFT_BLUE = new Pose2d(3.72, 3.006, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_2_RIGHT_BLUE = new Pose2d(4.006, 2.846, new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_3_LEFT_BLUE = new Pose2d(5.004, 2.86, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_3_RIGHT_BLUE = new Pose2d(5.28, 3.019, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_4_LEFT_BLUE = new Pose2d(5.746, 4.17, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_4_RIGHT_BLUE = new Pose2d(5.746, 3.86, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_5_LEFT_BLUE = new Pose2d(5.25, 5.04, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_5_RIGHT_BLUE = new Pose2d(4.97, 5.18, new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_6_LEFT_BLUE = new Pose2d(4.00, 5.20, new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_6_RIGHT_BLUE = new Pose2d(3.72, 5.04, new Rotation2d(Math.toRadians(240)));

    // red side
    public static final Pose2d SCORING_POSITION_1_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_1_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_1_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_1_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_1_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_1_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d SCORING_POSITION_2_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_2_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_2_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_2_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_2_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_2_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(120)));
    public static final Pose2d SCORING_POSITION_3_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_3_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_3_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_3_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_3_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_3_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(60)));
    public static final Pose2d SCORING_POSITION_4_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_4_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_4_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d SCORING_POSITION_4_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_4_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_4_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(0)));
    public static final Pose2d SCORING_POSITION_5_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_5_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_5_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_5_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_5_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_5_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(300)));
    public static final Pose2d SCORING_POSITION_6_LEFT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_6_LEFT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_6_LEFT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));
    public static final Pose2d SCORING_POSITION_6_RIGHT_RED = new Pose2d(FieldLayout.FIELD_LENGTH - SCORING_POSITION_6_RIGHT_BLUE.getX(), FieldLayout.FIELD_WIDTH - SCORING_POSITION_6_RIGHT_BLUE.getY(), new Rotation2d(Math.toRadians(240)));


    public static final Pose2d[] SETPOINTS = new Pose2d[24];

    static {
    //blue side
    SETPOINTS[0] = SCORING_POSITION_1_RIGHT_BLUE;
    SETPOINTS[1] = SCORING_POSITION_1_LEFT_BLUE;
    SETPOINTS[2] = SCORING_POSITION_2_LEFT_BLUE;
    SETPOINTS[3] = SCORING_POSITION_2_RIGHT_BLUE;
    SETPOINTS[4] = SCORING_POSITION_3_LEFT_BLUE;
    SETPOINTS[5] = SCORING_POSITION_3_RIGHT_BLUE;
    SETPOINTS[6] = SCORING_POSITION_4_LEFT_BLUE;
    SETPOINTS[7] = SCORING_POSITION_4_RIGHT_BLUE;
    SETPOINTS[8] = SCORING_POSITION_5_LEFT_BLUE;
    SETPOINTS[9] = SCORING_POSITION_5_RIGHT_BLUE;
    SETPOINTS[10] = SCORING_POSITION_6_LEFT_BLUE;
    SETPOINTS[11] = SCORING_POSITION_6_RIGHT_BLUE;

    //red
    SETPOINTS[12] = SCORING_POSITION_1_RIGHT_RED;
    SETPOINTS[13] = SCORING_POSITION_1_LEFT_RED;
    SETPOINTS[14] = SCORING_POSITION_2_LEFT_RED;
    SETPOINTS[15] = SCORING_POSITION_2_RIGHT_RED;
    SETPOINTS[16] = SCORING_POSITION_3_LEFT_RED;
    SETPOINTS[17] = SCORING_POSITION_3_RIGHT_RED;
    SETPOINTS[18] = SCORING_POSITION_4_LEFT_RED;
    SETPOINTS[19] = SCORING_POSITION_4_RIGHT_RED;
    SETPOINTS[20] = SCORING_POSITION_5_LEFT_RED;
    SETPOINTS[21] = SCORING_POSITION_5_RIGHT_RED;
    SETPOINTS[22] = SCORING_POSITION_6_LEFT_RED;
    SETPOINTS[23] = SCORING_POSITION_6_RIGHT_RED;
    }

    public enum Objective {
      LAYINGEGGSBOTTOM("3L4Bot"),
      LAYINGEGGSTOP("3L4Top"),
      ENDATTOP2("2L4Top"),
      ENDATBOT2("2L4Bot"),
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

    public static final double BOTTOM_LIMIT = 0.75;
    public static final double TOP_LIMIT = 0.3;

    public static final double CLIMBER_CONTROLLER_DEADBAND = 0.1;

    public static final double CLIMBER_UP_POSITION = 0.3;
    public static final double CLIMBER_DOWN_POSITION = 0.75;
    public static final double CLIMBER_CENTER_POSITION = 0.45;


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
