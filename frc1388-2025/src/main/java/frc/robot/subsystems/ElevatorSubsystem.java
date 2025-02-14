// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorSubsystemConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import au.grapplerobotics.LaserCan;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  // Bottom sparkmax canid: 8 on left side looking at the motor
  // Top sparkmax canid: 7 on the right side
  //positive goes up negative goes down
  // bottom limit switch is DIO 8 top is 9
  SparkMax m_rightMotor;
  SparkMax m_leftMotor;
  DigitalInput m_topLimitSwitch;
  DigitalInput m_bottomLimitSwitch;
  LaserCan m_laserCan;
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

    LEVEL1(5),
    LEVEL2(15),
    LEVEL3(25),
    LEVEL4(35);

    private double setpoint;

    private ElevatorSetPoints(double setpoint) {
      this.setpoint = setpoint;
    }

    public double getSetPoint() {
      return this.setpoint;
    }
  }

  public ElevatorSubsystem(SparkMax rightMotor, SparkMax leftMotor, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch, LaserCan laserCan) {
    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;
    m_topLimitSwitch = topLimitSwitch;
    m_bottomLimitSwitch = bottomLimitSwitch;
    m_laserCan = laserCan;
    m_elevatorEncoder = rightMotor.getEncoder();
    m_elevatorEncoder = leftMotor.getEncoder();
  
    m_elevatorController.setTolerance(ElevatorSubsystemConstants.kElevatorTolerance);
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
      } else {
        powerLimit = ElevatorSubsystemConstants.kElevatorPowerLimit;
      }
      power = MathUtil.clamp(power,
        -(powerLimit),
          powerLimit);
    }
      SmartDashboard.putNumber("elevator/moveElevatorPower", power);
      //driving the motors
      m_leftMotor.set(power);
      m_rightMotor.set(power);
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
    setTargetPosition(getElevatorHeight());
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
    double height = m_elevatorEncoder.getPosition();
    //  / ElevatorSubsystemConstants.kTicksPerInch;
    return height;
  }

  // public double getMotorEncoderHeight() {
    
  // }

  public double getLaserCanHeight() {
    return (m_laserCan.getMeasurement().distance_mm) * ElevatorSubsystemConstants.kInchesPerMillimeters;
  }
  
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
    // System.out.println("speed =" + speed +
    //     " height =" + getElevatorHeight() + " setpoint =" + m_targetPosition + " error"
    //     + (m_targetPosition - getElevatorHeight()));
  }
}