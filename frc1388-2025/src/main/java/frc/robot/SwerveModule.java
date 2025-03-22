package frc.robot;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    
  private final TalonFX m_driveMotor;
  private final TalonFXConfiguration m_driveMotorSettings;

  private final CANcoder m_cancoder;
  private double m_encoderOffset;

  private final TalonFX m_rotationMotor;
  private final TalonFXConfiguration m_rotationMotorSettings;
  private final PIDController m_rotationPID;

  public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, TalonFXConfiguration rotationMotorSettings, TalonFXConfiguration driveMotorSettings, CANcoder cancoder, double encoderOffset) {
      m_driveMotor = driveMotor;
      m_driveMotorSettings = driveMotorSettings;
      m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
      Slot0Configs driveConfig = new Slot0Configs();
      driveConfig.kP = Constants.SwerveModuleConstants.DRIVE_MOTOR_P;
      driveConfig.kI = Constants.SwerveModuleConstants.DRIVE_MOTOR_I;
      driveConfig.kD = Constants.SwerveModuleConstants.DRIVE_MOTOR_D;
      m_driveMotorSettings.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      m_driveMotor.getConfigurator().apply(m_driveMotorSettings);
      m_driveMotor.getConfigurator().apply(driveConfig);

      m_rotationMotor = rotationMotor;
      m_rotationMotorSettings = rotationMotorSettings;
      m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
      m_rotationMotorSettings.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      m_rotationMotor.getConfigurator().apply(m_rotationMotorSettings);

      m_encoderOffset = encoderOffset;

      m_rotationPID = new PIDController(
          Constants.SwerveModuleConstants.ROTATION_MOTOR_P,
          Constants.SwerveModuleConstants.ROTATION_MOTOR_I,
          Constants.SwerveModuleConstants.ROTATION_MOTOR_D);
      m_rotationPID.setTolerance(Constants.SwerveModuleConstants.ROTATION_TOLERANCE);
      m_rotationPID.enableContinuousInput(0, 360);

      m_cancoder = cancoder;
      MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs();
      cancoderConfig.AbsoluteSensorDiscontinuityPoint = 1;
      cancoderConfig.MagnetOffset = 0;
      cancoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      m_cancoder.getConfigurator().apply(cancoderConfig);
  }
  
  public void setSwerveModuleStates(SwerveModuleState inputState) {
    Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
    inputState.optimize(rotation);
    setDriveSpeed(inputState.speedMetersPerSecond);
    setRotationPosition(inputState.angle.getDegrees());

  }

  public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          m_driveMotor.getPosition().getValueAsDouble() * Constants.SwerveModuleConstants.DIST_PER_MOTOR_ROTATION,
          new Rotation2d(Math.toRadians(getRotationAngle()))
      );
  }
  public void setBrakeMode(boolean brakeMode){
      if(brakeMode) {
          m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
          m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);
      } else {
          m_driveMotor.setNeutralMode(NeutralModeValue.Coast);
          m_rotationMotor.setNeutralMode(NeutralModeValue.Coast);
      }
  }

  /**
   * @brief set the robot's driving speed
   * @param inputSpeed meters per second
   */
  public void setDriveSpeed(double inputSpeed) { 
      // dividing the speed by the robot's max speed results in a power [-1, 1] that we can set the motor to 
      m_driveMotor.set(inputSpeed / Constants.DriveTrainConstants.ROBOT_MAX_SPEED); // probably should be done outside of swerve module
  }

  public void setRotationPosition(double angle) {
      m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), angle));
  }

  /**
   * Determines the encoder offset from the swerve module.
   *<p>
   * Applies this offset to the swerve module.
   * <p>
   * Wheels MUST be pointed to 0 degrees relative to robot to use this method
   * @return new encoder offset
   */
  public double setEncoderOffset(){ 
     double rotationAngle = m_cancoder.getAbsolutePosition().getValueAsDouble() * 360; 
     m_encoderOffset = rotationAngle;
     return m_encoderOffset;
  }

  public double getRotationAngle() {
      double rotationAngle;
      rotationAngle = m_cancoder.getAbsolutePosition().getValueAsDouble() * 360 - m_encoderOffset;
      rotationAngle = rotationAngle % 360;
      if(rotationAngle < 0){
          rotationAngle += 360;
      }
      return rotationAngle;        
  }

  public void periodic() {
      // if the DriveTrain subsystem periodic is calling this, this method acts as a periodic
  }
}