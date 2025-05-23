// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.VisionAcceptor;
import frc.robot.vision.LimelightHelpers.PoseEstimate; 

public class DriveTrainSubsystem extends SubsystemBase {

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishPose1 = NetworkTableInstance.getDefault().getStructTopic("pose/pose1", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishPose2 = NetworkTableInstance.getDefault().getStructTopic("pose/pose2", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishPose3 = NetworkTableInstance.getDefault().getStructTopic("pose/pose3", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishMegaTag2FrontRight = NetworkTableInstance.getDefault().getStructTopic("pose/megatag2FrontRight", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishMegaTag2FrontLeft = NetworkTableInstance.getDefault().getStructTopic("pose/megatag2FrontLeft", Pose2d.struct).publish();
  StructPublisher<Pose2d> publishMegaTag2Back = NetworkTableInstance.getDefault().getStructTopic("pose/megatag2Back", Pose2d.struct).publish();

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); 

  private ChassisSpeeds m_robotRelativeSpeeds = new ChassisSpeeds();

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  /** The distance in <strong>meters</strong> from the center of rotation of the front wheel to the center of rotation of the back wheel */
  private final double ROBOT_WHEEL_BASE = RobotConstants.ROBOT_LENGTH;
  /** The distance in <strong>meters</strong> from the center of rotation of the left wheel to the center of rotation of the right wheel */
  private final double ROBOT_TRACK_WIDTH = RobotConstants.ROBOT_WIDTH;

  private double m_gyroOffset = 0;

  private final PIDController xController = new PIDController(0.1, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.1, 0.0, 0.0);
  private final PIDController headingController = new PIDController(0.1, 0.0, 0.0);

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

  VisionAcceptor visionAcceptorGyroFront = new VisionAcceptor(false);
  VisionAcceptor visionAcceptorGyroBack = new VisionAcceptor(false);
  VisionAcceptor visionAcceptorGyroFrontLeft = new VisionAcceptor(false); // ask alex
  VisionAcceptor visionAcceptorMegaTag2FrontLeft = new VisionAcceptor(true);
  VisionAcceptor visionAcceptorMegaTag2Front = new VisionAcceptor(true);
  VisionAcceptor visionAcceptorMegaTag2Back = new VisionAcceptor(true);
  VisionAcceptor visionAcceptor = new VisionAcceptor(false);

  
  Pose2d previousPosition1 = new Pose2d(0, 0, new Rotation2d(0));
  Pose2d previousPosition2 = new Pose2d(0, 0, new Rotation2d(0));
  Pose2d previousPosition3 = new Pose2d(0, 0, new Rotation2d(0));

  Pose2d previousMegaTag2Front = new Pose2d(0, 0, new Rotation2d(0));
  Pose2d previousMegaTag2Back = new Pose2d(0, 0, new Rotation2d(0));
  Pose2d previousMegaTagFrontLeft = new Pose2d(0, 0, new Rotation2d(0));

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  /** The odometry object keeps track of the robots position */
  private SwerveDrivePoseEstimator m_odometry;

  private final Limelight m_limelight;
  private final Pigeon2 m_pigeonGyro;
  // private final AHRS m_navxGyro;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, Pigeon2 gyro, Limelight limelight) {

    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_pigeonGyro = gyro;
    m_limelight = limelight;
    
    //gyro and odometry setup code I copied from a youtube video <br> https://www.youtube.com/watch?v=0Xi9yb1IMyA
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_pigeonGyro.reset();
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
    // m_robotRelativeSpeeds = ChassisSpeeds.discretize(xVelocity, yVelocity, omega,
    // DriveTrainConstants.DT_SECONDS);
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
    if (m_pigeonGyro.isConnected()) {
      // return new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
      return new Rotation2d(Math.toRadians(getAngle()));
    }
    return new Rotation2d();
  }

  public void resetGyroHeading(double offset) {
    m_gyroOffset = offset;
    m_pigeonGyro.reset();
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
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      m_frontRight.getPosition(),
      m_frontLeft.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
  };
    if(m_odometry != null) {
      m_odometry.resetPosition(getGyroHeading(), swerveModulePositions, pose);
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

  public void limelightResetGyroFront() {
    double[] botPose = m_limelight.getBotPoseFront();
    m_gyroOffset = botPose[5] - getRawGyroAngle();
  }

  public void limelightResetGyroBack() {
    double[] botPose = m_limelight.getBotPoseBack();
    m_gyroOffset = botPose[5] - getRawGyroAngle();
  }

  public void limelightResetGyroFrontLeft() {
    double[] botPose = m_limelight.getBotPoseLeft();
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

  public Pose2d getClosestTargetPoseRight() {

    Pose2d[] SETPOINTS = AutoConstants.SETPOINTS_RIGHT;
    
    // double distance;
    // double nextSetpointDistance;
    // Pose2d[] setpoints = SETPOINTS;
    // Pose2d closestPose = setpoints[0];
    // for (int i = 0; i < setpoints.length - 1; i++) {
    //   distance = calculateDistancePose(setpoints[i]);
    //   nextSetpointDistance = calculateDistancePose(setpoints[i + 1]);
    //   if (distance < nextSetpointDistance) {
    //     closestPose = setpoints[i];
    //   }
    //   else {
    //     closestPose = setpoints[i + 1];
    //   }
    // }
    
    double distance;
    Pose2d[] setpoints = SETPOINTS;
    double closestDistance = calculateDistancePose(setpoints[0]);
    Pose2d closestPose = setpoints[0];
    for (int i = 0; i < setpoints.length; i++) {
      distance = calculateDistancePose(setpoints[i]);
      if (distance < closestDistance) {
        closestPose = setpoints[i];
        closestDistance = distance;
      }
    }
    return closestPose;
  }

  public Pose2d getClosestTargetPoseLeft() {

    Pose2d[] SETPOINTS = AutoConstants.SETPOINTS_LEFT;
    
    // double distance;
    // double nextSetpointDistance;
    // Pose2d[] setpoints = SETPOINTS;
    // Pose2d closestPose = setpoints[0];
    // for (int i = 0; i < setpoints.length - 1; i++) {
    //   distance = calculateDistancePose(setpoints[i]);
    //   nextSetpointDistance = calculateDistancePose(setpoints[i + 1]);
    //   if (distance < nextSetpointDistance) {
    //     closestPose = setpoints[i];
    //   }
    //   else {
    //     closestPose = setpoints[i + 1];
    //   }
    // }
    
    double distance;
    Pose2d[] setpoints = SETPOINTS;
    double closestDistance = calculateDistancePose(setpoints[0]);
    Pose2d closestPose = setpoints[0];
    for (int i = 0; i < setpoints.length; i++) {
      distance = calculateDistancePose(setpoints[i]);
      if (distance < closestDistance) {
        closestPose = setpoints[i];
        closestDistance = distance;
      }
    }
    return closestPose;
  }

  public Pose2d getClosestTargetPoseL1() {

    Pose2d[] SETPOINTS = AutoConstants.SETPOINTS_L1;
    
    // double distance;
    // double nextSetpointDistance;
    // Pose2d[] setpoints = SETPOINTS;
    // Pose2d closestPose = setpoints[0];
    // for (int i = 0; i < setpoints.length - 1; i++) {
    //   distance = calculateDistancePose(setpoints[i]);
    //   nextSetpointDistance = calculateDistancePose(setpoints[i + 1]);
    //   if (distance < nextSetpointDistance) {
    //     closestPose = setpoints[i];
    //   }
    //   else {
    //     closestPose = setpoints[i + 1];
    //   }
    // }
    
    double distance;
    Pose2d[] setpoints = SETPOINTS;
    double closestDistance = calculateDistancePose(setpoints[0]);
    Pose2d closestPose = setpoints[0];
    for (int i = 0; i < setpoints.length; i++) {
      distance = calculateDistancePose(setpoints[i]);
      if (distance < closestDistance) {
        closestPose = setpoints[i];
        closestDistance = distance;
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
    if (m_pigeonGyro.isConnected()) {
      double angle = (getRawGyroAngle() + m_gyroOffset) % 360;
      if (angle < 0) {
        angle += 360;
      }
      return angle;
    }
    return 0;
  }

  public double getRawGyroAngle () {
    if (m_pigeonGyro.isConnected()) {
      return m_pigeonGyro.getYaw().getValueAsDouble();
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

  // public boolean shouldResetPoseMegaTag2Front() {
  //     boolean acceptMegaTag2 = false;
  //     double[] odomTag2 = m_limelight.getMegaTag2Front();

  //     Pose2d megaTag2 = new Pose2d(odomTag2[0], odomTag2[1], getGyroHeading());
  //     if (m_limelight.getApriltagTargetFound()) {
  //       acceptMegaTag2 = visionAcceptorMegaTag2Front.shouldAccept(megaTag2);
  //     }
  //     return acceptMegaTag2;
  //   }

  //   public boolean shouldResetPoseMegaTag2Back() {
  //     boolean acceptMegaTag2 = false;
  //     double[] odomTag2 = m_limelight.getMegaTag2Back();
  //     Pose2d megaTag2 = new Pose2d(odomTag2[0], odomTag2[1], getGyroHeading());
  //     if (m_limelight.getApriltagTargetFound()) {
  //       acceptMegaTag2 = visionAcceptorMegaTag2Back.shouldAccept(megaTag2);
  //     }
  //     return acceptMegaTag2;
  //   }

  // public boolean shouldResetGyroFront() {
  //   boolean acceptPose = false;
  //   double[] botPose = m_limelight.getBotPoseFront();
  //   Pose2d megaTag1 = new Pose2d(botPose[0], botPose[1], getGyroHeading());
  //   if (m_limelight.getApriltagTargetFound()) {
  //     acceptPose = visionAcceptor.shouldAccept(megaTag1);
  //   }
  //   return acceptPose;
  // }

  // public boolean shouldResetGyroBack() {
  //   boolean acceptPose = false;
  //   double[] botPose = m_limelight.getBotPoseBack();
  //   Pose2d megaTag1 = new Pose2d(botPose[0], botPose[1], getGyroHeading());
  //   if (m_limelight.getApriltagTargetFound()) {
  //     acceptPose = visionAcceptor.shouldAccept(megaTag1);
  //   }
  //   return acceptPose;
  // }


  
  // public void resetPose(Pose2d pose) {
  //   if (m_limelight.getApriltagTargetFound()) {
  //     limelightResetMegaTag2();
  //   } else {
  //     swerveOnlyResetPose(pose);
  //   }
  // }

  public void stopDriveTrain () {
     drive(0, 0, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_frontRight.periodic();
    m_frontLeft.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();

    // boolean acceptPoseFront = false;
    // boolean acceptPoseBack = false;
    // boolean acceptPoseFrontLeft = false;
    boolean acceptMegaTag2Front = false;
    boolean acceptMegaTag2Back = false;
    boolean acceptMegaTag2FrontLeft = false;  

    // Field2d fieldPose1 = new Field2d();

    LimelightHelpers.SetRobotOrientation("limelight-front", getAngle(), 0, 0, 0, 0, 0);
    // LimelightHelpers.SetRobotOrientation("limelight-back", getAngle(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", getAngle(), 0, 0, 0, 0, 0);

    // PoseEstimate botPoseFront = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
    // PoseEstimate botPoseBack = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");
    // PoseEstimate botPoseFrontLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

    PoseEstimate odomTag2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    // PoseEstimate odomTag2Back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
    PoseEstimate odomTag2FrontLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

    if (m_odometry != null && odomTag2Front != null && odomTag2FrontLeft != null) {
      if (m_robotRelativeSpeeds != null) {
          // acceptPoseFront = visionAcceptor.shouldAccept(botPoseFront.pose, previousPosition1, m_robotRelativeSpeeds);
          // previousPosition1 = botPoseFront.pose;

          acceptMegaTag2Front = visionAcceptorMegaTag2Front.shouldAccept(odomTag2Front.pose,
              previousMegaTag2Front, m_robotRelativeSpeeds);
          // acceptGyroFront = visionAcceptorGyroFront.shouldResetGyro();
          previousMegaTag2Front = odomTag2Front.pose;


          // acceptPoseBack = visionAcceptor.shouldAccept(botPoseBack.pose, previousPosition2, m_robotRelativeSpeeds);
          // previousPosition2 = botPoseBack.pose;

          // acceptMegaTag2Back = visionAcceptorMegaTag2Back.shouldAccept(odomTag2Back.pose, previousMegaTag2Back, m_robotRelativeSpeeds);
          // previousMegaTag2Back = odomTag2Back.pose;
          // acceptGyroBack = visionAcceptorGyroBack.shouldResetGyro();


          // acceptPoseFrontLeft = visionAcceptor.shouldAccept(botPoseFrontLeft.pose, previousPosition3, m_robotRelativeSpeeds);
          // previousPosition3 = botPoseFrontLeft.pose;
          acceptMegaTag2FrontLeft = visionAcceptorMegaTag2FrontLeft.shouldAccept(odomTag2FrontLeft.pose,
              previousMegaTagFrontLeft, m_robotRelativeSpeeds);
              previousMegaTagFrontLeft = odomTag2FrontLeft.pose;
          // acceptGyroFrontLeft = visionAcceptorGyroFrontLeft.shouldResetGyro();
        
          
        // if (acceptGyroFront && acceptPoseFront) {
        //   limelightResetGyroFront();
        // } else if (acceptGyroBack && acceptPoseBack) {
        //   limelightResetGyroBack();
        // } else if (acceptGyroFrontLeft && acceptPoseFrontLeft) {
        //   limelightResetGyroFrontLeft();
        // }

        if (acceptMegaTag2Front && LimelightHelpers.getTV("limelight-front")) {
          m_odometry.addVisionMeasurement(odomTag2Front.pose, odomTag2Front.timestampSeconds);
        }
        // if (acceptMegaTag2Back && LimelightHelpers.getTV("limelight-back")) {
        //   m_odometry.addVisionMeasurement(odomTag2Back.pose, odomTag2Back.timestampSeconds);
        // }
        if (acceptMegaTag2FrontLeft && LimelightHelpers.getTV("limelight-left")) {
          m_odometry.addVisionMeasurement(odomTag2FrontLeft.pose, odomTag2FrontLeft.timestampSeconds);
        }
      }
      // m_odometry.updateWithTime(
      //     Timer.getFPGATimestamp(),
      //     getGyroHeading(),
      //     new SwerveModulePosition[] {
      //         m_frontRight.getPosition(),
      //         m_frontLeft.getPosition(),
      //         m_backLeft.getPosition(),
      //         m_backRight.getPosition()
      //     });
      m_odometry.update(
          getGyroHeading(),
          new SwerveModulePosition[] {
              m_frontRight.getPosition(),
              m_frontLeft.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
          });
    }

    SmartDashboard.putNumber("drivetrain/odo x", getPose().getX());
    SmartDashboard.putNumber("drivetrain/odo y", getPose().getY());
    if (getRobotRelativeSpeeds() != null) {
    SmartDashboard.putString("drivetrain/robot relative speeds", getRobotRelativeSpeeds().toString());
    }

    SmartDashboard.putNumber("drivetrain/gyro angle", getAngle());
    // SmartDashboard.putBoolean("VisionAcceptor/is Accepting FrontRight", acceptPoseFront);
    SmartDashboard.putBoolean("VisionAcceptor/is Accepting megatag2FrontRight", acceptMegaTag2Front);
    // SmartDashboard.putBoolean("VisionAcceptor/is Accepting FrontLeft", acceptPoseFrontLeft);
    SmartDashboard.putBoolean("VisionAcceptor/is Accepting megatag2FrontLeft", acceptMegaTag2FrontLeft);
    // SmartDashboard.putBoolean("VisionAcceptor/is Accepting Pose Back", acceptPoseBack);
    SmartDashboard.putBoolean("VisionAcceptor/is Accepting megatag2Back", acceptMegaTag2Back);

    SmartDashboard.putNumber("drivetrain/closestPoseXRight", getClosestTargetPoseRight().getX());
    SmartDashboard.putNumber("drivetrain/closestPoseYRight", getClosestTargetPoseRight().getY());
    SmartDashboard.putNumber("drivetrain/closestPoseRotationRight", getClosestTargetPoseRight().getRotation().getDegrees());
    SmartDashboard.putNumber("Angle Rotation2d", getGyroHeading().getRadians());

    SmartDashboard.putNumber("drivetrain/closestPoseXLeft", getClosestTargetPoseLeft().getX());
    SmartDashboard.putNumber("drivetrain/closestPoseYLeft", getClosestTargetPoseLeft().getY());
    SmartDashboard.putNumber("drivetrain/closestPoseRotationLeft", getClosestTargetPoseLeft().getRotation().getDegrees());

    SmartDashboard.putNumber("drivetrain/frontRight encoder angle", m_frontRight.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/frontLeft encoder angle", m_frontLeft.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/backLeft encoder angle", m_backLeft.getRotationAngle());
    SmartDashboard.putNumber("drivetrain/backRight encoder angle", m_backRight.getRotationAngle());

    publisher.set(getPose());
    // publishPose1.set(botPoseFront.pose);
    // publishPose2.set(botPoseBack.pose);
    // publishPose3.set(botPoseFrontLeft.pose);
    if (odomTag2Front != null && odomTag2FrontLeft != null) {
    publishMegaTag2FrontRight.set(odomTag2Front.pose);
    publishMegaTag2FrontLeft.set(odomTag2FrontLeft.pose);
    // publishMegaTag2Back.set(odomTag2Back.pose);
    }
  }
}
