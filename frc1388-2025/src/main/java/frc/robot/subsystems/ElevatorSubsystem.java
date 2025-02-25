// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import au.grapplerobotics.LaserCan;


public class ElevatorSubsystem extends SubsystemBase {

  SparkFlex m_motor;
  DigitalInput m_topLimitSwitch;
  DigitalInput m_bottomLimitSwitch;
  private double m_targetPosition = 0;
  private RelativeEncoder m_elevatorEncoder;
  private boolean m_isInitialized = false;
  private boolean m_autoMode = false;
  private double m_manualPower = 0;

  // private final PIDController m_elevatorController = new PIDController(ElevatorSubsystemConstants.kElevatorPIDP, ElevatorSubsystemConstants.kElevatorPIDI, 0.0015);
  private final PIDController m_elevatorController = 
   new PIDController(
   ElevatorSubsystemConstants.kElevatorPIDP,
   ElevatorSubsystemConstants.kElevatorPIDI, 
   ElevatorSubsystemConstants.kElevatorPIDD);

  public enum ElevatorSetPoints {
    // LEVEL1(18),
    // LEVEL2(31.875);  //highest height is 37 inches on elevator
    // LEVEL3(47.625),
    // LEVEL4(72);

    //values are in inches
    LEVEL1(0),
    LEVEL2(10),
    LEVEL3(25),
    LEVEL4(50.5);

    private double setpoint;

    private ElevatorSetPoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getSetPoint() {
      return this.setpoint;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(SparkFlex motor, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
    m_motor = motor;
    m_topLimitSwitch = topLimitSwitch;
    m_bottomLimitSwitch = bottomLimitSwitch;
    m_elevatorEncoder = motor.getEncoder();
  
    m_elevatorController.setTolerance(ElevatorSubsystemConstants.kElevatorTolerance);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    boolean isInverted = false;
    motorConfig.inverted(isInverted);
    motorConfig.encoder.positionConversionFactor(ElevatorSubsystemConstants.kCarriageInchesPerMotorRotation);
    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveElevator(double power) {
    if ((isAtTopLimit() && power > 0)) {
      power = 0;
    } else if (isAtBottomLimit() && power < 0) {
      power = 0;
    } else {
      double powerLimit;
      double height = getElevatorHeight();
      if ((height < ElevatorSubsystemConstants.kElevatorBottomEndRange && power < 0) || //if the elevator is near the bottom & going down, clamp power
          (height > ElevatorSubsystemConstants.kElevatorTopEndRange && power > 0)) {    //if the elevator is near the top & going up, clamp power
        powerLimit = ElevatorSubsystemConstants.kElevatorEndRangePowerLimit;
      } else if (power < 0) {
        powerLimit = ElevatorSubsystemConstants.kElevatorPowerLimitDown;
      }
      else {
        powerLimit = ElevatorSubsystemConstants.kElevatorPowerLimit;
      }
      power = MathUtil.clamp(power,
        -(powerLimit),
          powerLimit);
    }
      SmartDashboard.putNumber("elevator/moveElevatorPower", power);
      //driving the motors
      m_motor.set(power);
      // System.out.println("power = " + power);
  }

  public void setManualPower(double power) {
    m_manualPower = power;
    m_autoMode = false;
  }

  public void setTargetPosition(double position) {
    m_targetPosition = position;
    m_elevatorController.setSetpoint(position);
  }

  public void setSetpointToCurrentPosition() {
    double scaleFactor = MathUtil.clamp(ElevatorSubsystemConstants.kElevatorOffsetAccountSpeed * getMotorEncoderVelocity(), -1, 1);
    setTargetPosition(getElevatorHeight() + scaleFactor);
    m_autoMode = true;
  }

  public double getTargetPosition() {
    return m_targetPosition;
  }

  public void setSetpoint(ElevatorSetPoints setpoint) {
    setTargetPosition(setpoint.getSetPoint());
    m_autoMode = true;
  }

  /**
   * returns height of elevator
   * @return height in inches
   */
  public double getElevatorHeight() {
    // return getLaserCanHeight();
    return getMotorEncoderHeight();
  }
  
  private double getMotorEncoderHeight() {
    return m_elevatorEncoder.getPosition();
  }

  public double getMotorEncoderVelocity() {
    return m_elevatorEncoder.getVelocity() * ElevatorSubsystemConstants.kDistancePerVelocityScale;
  }

  // public double getLaserCanHeight() {
  //   return (m_laserCan.getMeasurement().distance_mm) * ElevatorSubsystemConstants.kInchesPerMillimeters;
  // }
  
  private void resetEncoder() {
    m_elevatorEncoder.setPosition(ElevatorSubsystemConstants.kElevatorLimitSwitchZero);
  }

  private boolean isAtTopLimit() {
    return m_topLimitSwitch.get();
  }
  
  private boolean isAtBottomLimit() {
    return m_bottomLimitSwitch.get();
  }
  
  public void resetElevatorSubsystem() {
    m_isInitialized = false;
    m_autoMode = false;
  }

  public boolean isAtSetpoint() {
    return m_elevatorController.atSetpoint();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double speed = 0;
    if (m_isInitialized) {
      if (m_autoMode) {
        speed = m_elevatorController.calculate(getElevatorHeight());
        if (m_elevatorController.atSetpoint()) {
          speed = 0;
        }
      } else {
        speed = m_manualPower;
      }
      moveElevator(speed);
    } else {
      moveElevator(-ElevatorSubsystemConstants.kElevatorEndRangePowerLimit);
      if (isAtBottomLimit()) {
        moveElevator(0);
        resetEncoder();
        m_targetPosition = 0;
        m_manualPower = 0;
        setSetpointToCurrentPosition();
        m_isInitialized = true;
      }
    }
    if (isAtBottomLimit()) {
      resetEncoder();
    }
//TODO: top and bottom limit only work after motor isnt running anymore/if the motor is running the limits dont work (fix this)
    SmartDashboard.putNumber("elevator/encoder", m_elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("elevator/bottomlimit", isAtBottomLimit());
    SmartDashboard.putBoolean("elevator/toplimit", isAtTopLimit());
    SmartDashboard.putNumber("elevator/speed", speed);
    SmartDashboard.putNumber("elevator/elevatorVelocity", getMotorEncoderVelocity());
    // System.out.println("speed =" + speed +
    //     " height =" + getElevatorHeight() + " setpoint =" + m_targetPosition + " error"
    //     + (m_targetPosition - getElevatorHeight()));
  }
}

//TODO slow down elevator more when its moving downwards