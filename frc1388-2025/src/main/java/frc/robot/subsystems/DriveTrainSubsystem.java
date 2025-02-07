// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionAcceptor; 

public class DriveTrainSubsystem extends SubsystemBase {

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); 

  private VisionAcceptor visionAcceptorGyro = new VisionAcceptor(false);
  private VisionAcceptor visionAcceptorMegaTag2 = new VisionAcceptor(true);
  private VisionAcceptor visionAcceptor = new VisionAcceptor(false);

  private ChassisSpeeds m_robotRelativeSpeeds = new ChassisSpeeds();

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  /** The distance in <strong>meters</strong> from the center of rotation of the front wheel to the center of rotation of the back wheel */
  private final double ROBOT_WHEEL_BASE = RobotConstants.ROBOT_LENGTH;
  /** The distance in <strong>meters</strong> from the center of rotation of the left wheel to the center of rotation of the right wheel */
  private final double ROBOT_TRACK_WIDTH = RobotConstants.ROBOT_WIDTH;

  private double m_gyroOffset = 0;

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

   private static final Vector<N3> stateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1),
      Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(Math.pow(0.02, 1), Math.pow(0.02, 1),
      Units.degreesToRadians(10));

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  /** The odometry object keeps track of the robots position */
  private SwerveDrivePoseEstimator m_odometry;

  private final Limelight m_limelight;
  private final Pigeon2 m_pigeon2;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, Pigeon2 gyro, Limelight limelight) {

    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_pigeon2 = gyro;
    m_limelight = limelight;
    
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
            new Pose2d(0, 0, new Rotation2d()),
            stateStdDevs, visionMeasurementStdDevs);
            
      m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
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
    // m_robotRelativeSpeeds = ChassisSpeeds.discretize(xVelocity, yVelocity, omega, DriveTrainConstants.DT_SECONDS);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_robotRelativeSpeeds);

    // optimises wheel heading direction changes.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
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

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_robotRelativeSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
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

  public double getDistTraveled() {
    return m_frontRight.getPosition().distanceMeters;
  }

  public void limelightResetGyro() {
    double[] botPose = m_limelight.getBotPose();
    m_gyroOffset = botPose[5] - getRawGyroAngle();
  }

  public double calculateAngle(Pose2d positionOfTarget) {
    double rX = getPose().getX();
    double rY = getPose().getY();

    return Math.toDegrees(Math.atan2(rY - positionOfTarget.getY(), rX - positionOfTarget.getX())) + 180;
  }


  public double calculateDistancePose(Pose2d distancePose) {
    double rX = getPose().getX();
    double tX = distancePose.getX();
    double tAngle = calculateAngle(distancePose);

    double adjacent = rX - tX;
    double distanceFromPose = - (adjacent / Math.cos(Math.toRadians(tAngle)));
    return distanceFromPose;
  }

  public Pose2d getClosestTargetPose() {

    Pose2d[] SETPOINTS = new Pose2d[2];

    SETPOINTS[0] = new Pose2d(15.15, 5.55, new Rotation2d(0));
    SETPOINTS[1] = new Pose2d(13.75, 7.25, new Rotation2d(45));
    
    double distance;
    double nextSetpointDistance;
    Pose2d[] setpoints = SETPOINTS;
    Pose2d closestPose = setpoints[0];
    for (int i = 0; i < setpoints.length - 1; i++) {
      distance = calculateDistancePose(setpoints[i]);
      nextSetpointDistance = calculateDistancePose(setpoints[i + 1]);
      if (distance < nextSetpointDistance) {
        closestPose = setpoints[i];
      }
      else {
        closestPose = setpoints[i + 1];
      }
    }
    return closestPose;
  }

  public double getXVelocityAuto(double xSetpoint, PIDController goToPointController, SlewRateLimiter xAccLimiter) {
    double m_lastXSpeed = 0;
    double xSpeed = goToPointController.calculate(getPose().getX(), xSetpoint);
    if (xSpeed > m_lastXSpeed) {
      xSpeed = xAccLimiter.calculate(xSpeed);
    }
    m_lastXSpeed = xSpeed;
    return xSpeed;
  }

  public double getYVelocityAuto(double ySetpoint, PIDController goToPointController, SlewRateLimiter xAccLimiter) {
    double m_lastYSpeed = 0;
    double ySpeed = goToPointController.calculate(getPose().getX(), ySetpoint);
    if (ySpeed > m_lastYSpeed) {
      ySpeed = xAccLimiter.calculate(ySpeed);
    }
    m_lastYSpeed = ySpeed;
    return ySpeed;
  }

  public double getOmegaVelocityAuto(PIDController turnPidController) {
    double rz = getAngle();
    rz = rz < 0 ? rz + 360 : rz;
    double speed = - (turnPidController.calculate(rz));
    return speed;
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

public boolean shouldResetPoseMegaTag2() {
    boolean acceptMegaTag2 = false;
    double[] odomTag2 = m_limelight.getMegaTag2();
    Twist2d robotSpeeds = new Twist2d(getRobotRelativeSpeeds().vxMetersPerSecond,
        getRobotRelativeSpeeds().vyMetersPerSecond, getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Pose2d megaTag2 = new Pose2d(odomTag2[0], odomTag2[1], getGyroHeading());
    if (m_limelight.getApriltagTargetFound()) {
      acceptMegaTag2 = visionAcceptorMegaTag2.shouldAccept(megaTag2, robotSpeeds);
    }
    return acceptMegaTag2;
  }

  public boolean shouldResetGyro() {
    boolean acceptPose = false;
    double[] botPose = m_limelight.getBotPose();
    Twist2d robotSpeeds = new Twist2d(getRobotRelativeSpeeds().vxMetersPerSecond,
        getRobotRelativeSpeeds().vyMetersPerSecond, getRobotRelativeSpeeds().omegaRadiansPerSecond);
    Pose2d megaTag1 = new Pose2d(botPose[0], botPose[1], getGyroHeading());
    if (m_limelight.getApriltagTargetFound()) {
      acceptPose = visionAcceptor.shouldAccept(megaTag1, robotSpeeds);
    }
    return acceptPose;
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

    boolean acceptPose = false;
    boolean acceptMegaTag2 = false;
    boolean acceptGyro = false;
    
    double[] botPose = m_limelight.getBotPose();

    double[] odomTag2 = m_limelight.getMegaTag2();
    
    Pose2d position1 = new Pose2d(botPose[0], botPose[1], getGyroHeading());

    Pose2d megaTag2 = new Pose2d(odomTag2[0], odomTag2[1], getGyroHeading());

    LimelightHelpers.SetRobotOrientation("limelight-shooter", getGyroHeading().getDegrees(), 0, 0, 0, 0, 0);

    Twist2d robotSpeeds = new Twist2d(getRobotRelativeSpeeds().vxMetersPerSecond,
    getRobotRelativeSpeeds().vyMetersPerSecond, getRobotRelativeSpeeds().omegaRadiansPerSecond);

    double timer = 1.5;
    if (Timer.getFPGATimestamp() > 1.5 ) {
      timer = Timer.getFPGATimestamp();
    }
    if (m_limelight.getApriltagTargetFound()) {
      acceptPose = visionAcceptorGyro.shouldAccept(position1, robotSpeeds);
      acceptMegaTag2 = visionAcceptorMegaTag2.shouldAccept(megaTag2, robotSpeeds);
      acceptGyro = visionAcceptorGyro.shouldResetGyro(robotSpeeds);
    }

    if (acceptGyro && acceptPose) {
      limelightResetGyro();
    }

    if (m_odometry != null) {
      if (acceptMegaTag2 && m_limelight.getApriltagTargetFound()) {
        m_odometry.addVisionMeasurement(megaTag2, timer);
      }

      m_odometry.updateWithTime(
          Timer.getFPGATimestamp(),
          getGyroHeading(),
          new SwerveModulePosition[] {
              m_frontRight.getPosition(),
              m_frontLeft.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });
    }
    SmartDashboard.putNumber("Limelight ODO X", odomTag2[0]);
    SmartDashboard.putNumber("Limelight ODO Y", odomTag2[1]);

    SmartDashboard.putNumber("drivetrain/odo x", getPose().getX());
    SmartDashboard.putNumber("drivetrain/odo y", getPose().getY());
    if (getRobotRelativeSpeeds() != null) {
    SmartDashboard.putString("drivetrain/robot relative speeds", getRobotRelativeSpeeds().toString());
    }

    SmartDashboard.putNumber("drivetrain/gyro angle", getAngle());
    SmartDashboard.putBoolean("VisionAcceptor/is Accepting Pose", acceptPose);
    SmartDashboard.putBoolean("VissionAcceptor/is Accepting megatag2", acceptMegaTag2);

    SmartDashboard.putNumber("drivetrain/closestPoseX", getClosestTargetPose().getX());
    SmartDashboard.putNumber("drivetrain/closestPoseY", getClosestTargetPose().getY());
    SmartDashboard.putNumber("drivetrain/closestPoseRotation", getClosestTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Angle Rotation2d", getGyroHeading().getRadians());

    SmartDashboard.putNumber("drivetrain/frontRight encoder angle", m_frontRight.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/frontLeft encoder angle", m_frontLeft.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/backLeft encoder angle", m_backLeft.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/backRight encoder angle", m_backRight.getRotationAngle());

    publisher.set(getPose());

  }
}
