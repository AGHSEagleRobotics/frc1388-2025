// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.SwerveModule; 

public class DriveTrainSubsystem extends SubsystemBase {

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); 

  private ChassisSpeeds m_robotRelativeSpeeds = new ChassisSpeeds();

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  /** The distance in <strong>meters</strong> from the center of rotation of the front wheel to the center of rotation of the back wheel */
  private final double ROBOT_WHEEL_BASE = RobotConstants.ROBOT_LENGTH;
  /** The distance in <strong>meters</strong> from the center of rotation of the left wheel to the center of rotation of the right wheel */
  private final double ROBOT_TRACK_WIDTH = RobotConstants.ROBOT_WIDTH;

  private double m_gyroOffset = 0;

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  // these are the translations from the center of rotation of the robot to the center of rotation of each swerve module
  private final Translation2d m_frontRightTranslation = new Translation2d(ROBOT_WHEEL_BASE / 2, -ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(ROBOT_WHEEL_BASE / 2, ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-ROBOT_WHEEL_BASE / 2, ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(-ROBOT_WHEEL_BASE / 2, -ROBOT_TRACK_WIDTH / 2);

  /** Translating array for all the swerve modules */
  private final Translation2d[] m_swerveTranslation2d = {
    m_frontRightTranslation,
    m_frontLeftTranslation,
    m_backLeftTranslation,
    m_backRightTranslation
  };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  /** The odometry object keeps track of the robots position */
  private SwerveDrivePoseEstimator m_odometry;

  private final Pigeon2 m_pigeon2;


  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, Pigeon2 gyro) {

    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_pigeon2 = gyro;
    
    //gyro and odometry setup code I copied from a youtube video <br> https://www.youtube.com/watch?v=0Xi9yb1IMyA
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_pigeon2.reset();
        m_odometry = new SwerveDrivePoseEstimator(
            m_kinematics,
            getGyroHeading(),
            new SwerveModulePosition[] {
                m_frontRight.getPosition(),
                m_frontLeft.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            new Pose2d(0, 0, new Rotation2d()));
            
      } catch (Exception e) {
        // e.printStackTrace();
        System.out.println(e.toString());
      }
    }).start();

    DataLogManager.log("SwerveModuleOffsets: "
        + "  fr: " + Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)
        + "  fl: " + Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  bl: " + Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  br: " + Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0));

  } 

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /** the drive method takes in an x and y velocity in meters / second, and a rotation rate in radians / second */
  public void drive(double xVelocity, double yVelocity, double omega) {
    m_robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_robotRelativeSpeeds);

    // optimises wheel heading direction changes.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
    // do the divide by 3 speed here 

   
        }
    /**
     * Applies offset to ALL swerve modules.
     * <p>
     * Wheels MUST be pointed to 0 degrees relative to robot to use this method
     */
  public void setAllEncoderOffsets(){
    double frontRightOffset = m_frontRight.setEncoderOffset();
    double frontLeftOffset = m_frontLeft.setEncoderOffset();
    double backLeftOffset = m_backLeft.setEncoderOffset();
    double backRightOffset = m_backRight.setEncoderOffset();

    Preferences.setDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, frontRightOffset);
    Preferences.setDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, frontLeftOffset);
    Preferences.setDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, backLeftOffset);
    Preferences.setDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, backRightOffset);

    DataLogManager.log("SwerveModuleUpdatedOffsets: " + "setting offsets: "
        + "  fr: " + frontRightOffset
        + "  fl: " + frontLeftOffset
        + "  bl: " + backLeftOffset
        + "  br: " + backRightOffset);
  }

  public Rotation2d getGyroHeading() {
    if (m_pigeon2.isConnected()) {
      // return new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
      return new Rotation2d(Math.toRadians(getAngle()));
    }
    return new Rotation2d();
  }

  public void resetGyroHeading(double offset) {
    m_gyroOffset = offset;
    m_pigeon2.reset();
  }

  public void swerveOnlyResetPose(Pose2d pose) {
    m_odometry.resetPosition(getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  public Pose2d getPose() {
    if (m_odometry != null) {
      return m_odometry.getEstimatedPosition();
    }
    // return new Pose2d(123, 432, m_lastRotation2D);
    return new Pose2d(0, 0, getGyroHeading());
  }

  public void resetPose(Pose2d pose) {
    if (m_odometry != null) {
      m_odometry.resetPosition(getGyroHeading(), null, pose);
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_robotRelativeSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
    // speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
    m_robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroHeading());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
    //check desaturate constants
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);

    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
      // speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
      // speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
      SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
      //check desaturate constants
      m_frontRight.setSwerveModuleStates(states[0]);
      m_frontLeft.setSwerveModuleStates(states[1]);
      m_backLeft.setSwerveModuleStates(states[2]);
      m_backRight.setSwerveModuleStates(states[3]);
  
      }

    public void followTrajectory(SwerveSample trajectory) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            trajectory.vx + xController.calculate(pose.getX(), trajectory.x),
            trajectory.vy + yController.calculate(pose.getY(), trajectory.y),
            trajectory.omega + headingController.calculate(pose.getRotation().getRadians(), trajectory.heading)
        );

        // Apply the generated speeds
        driveFieldRelative(speeds);
    }


  public double getDistTraveled() {
    return m_frontRight.getPosition().distanceMeters;
  }

  public double getAngle() {
    if (m_pigeon2.isConnected()) {
      double angle = (getRawGyroAngle() + m_gyroOffset) % 360;
      if (angle < 0) {
        angle += 360;
      }
      return angle;
    }
    return 0;
  }

  public double getRawGyroAngle () {
    if (m_pigeon2.isConnected()) {
      return -m_pigeon2.getYaw().getValueAsDouble();
    }
    return 0;
  }

  public void setWheelAngle(double angle) {
    m_frontRight.setRotationPosition(angle);
    m_frontLeft.setRotationPosition(angle);
    m_backLeft.setRotationPosition(angle);
    m_backRight.setRotationPosition(angle);
  }

  public void differentialDrive(double speed) {
    m_frontRight.setRotationPosition(0);
    m_frontLeft.setRotationPosition(0);
    m_backLeft.setRotationPosition(0);
    m_backRight.setRotationPosition(0);

    m_frontRight.setDriveSpeed(speed);
    m_frontLeft.setDriveSpeed(speed);
    m_backLeft.setDriveSpeed(speed);
    m_backRight.setDriveSpeed(speed);
  }

  public void setBrakeMode(boolean brakeMode){
    m_frontRight.setBrakeMode(brakeMode);
    m_frontLeft.setBrakeMode(brakeMode);
    m_backLeft.setBrakeMode(brakeMode);
    m_backRight.setBrakeMode(brakeMode);
  }



  
  // public void resetPose(Pose2d pose) {
  //   if (m_limelight.getApriltagTargetFound()) {
  //     limelightResetMegaTag2();
  //   } else {
  //     swerveOnlyResetPose(pose);
  //   }
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_frontRight.periodic();
    m_frontLeft.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();

    if (m_odometry != null) {
      m_odometry.updateWithTime(
        Timer.getFPGATimestamp(),
        getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }
      );
    }
  }
}
