// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoints;

public class ClimberSubsystem extends SubsystemBase {

  SparkFlex m_climbMotor;
  DutyCycleEncoder m_absoluteClimbEncoder;

  private final PIDController m_climberPidController = 
   new PIDController(10, 0.0, 0.0);

   private double m_targetPosition = 0;
   private boolean m_autoMode = false;
   private double m_manualPower = 0;


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(SparkFlex climbMotor, DutyCycleEncoder absoluteClimbEncoder) {
    m_climbMotor = climbMotor;
    m_absoluteClimbEncoder = absoluteClimbEncoder;

    m_absoluteClimbEncoder.setDutyCycleRange(ClimberConstants.LOWER_PERCENTAGE_ABSOLUTE_ENCODER, ClimberConstants.HIGHER_PERCENTAGE_ABSOLUTE_ENCODER);
  }

  public void moveClimber(double power) {
    double powerLimit = ClimberConstants.CLIMBER_POWER_LIMIT;
    if ((isAtTopLimit() && power < 0)) {
      power = 0;
    } else if (isAtBottomLimit() && power > 0) {
      power = 0;
    }
    power = MathUtil.clamp(power,
        -(powerLimit),
          powerLimit);
    m_climbMotor.set(-power);
  }

  public double getPosition() {
    return m_absoluteClimbEncoder.get() + ClimberConstants.CLIMBER_ABSOLUTE_ENCODER_OFFSET;
  }

  public void setTargetPosition(double position) {
    m_targetPosition = position;
    m_climberPidController.setSetpoint(position);
  }

  public void setSetpoint(double setpoint) {
    setTargetPosition(setpoint);
    m_autoMode = true;
  }

  public void setManualPower(double power) {
    m_manualPower = power;
    m_autoMode = false;
  }

  public boolean isAtTopLimit() {
    if (getPosition() <= ClimberConstants.TOP_LIMIT) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAtBottomLimit() {
    if (getPosition() >= ClimberConstants.BOTTOM_LIMIT) {
      return true;
    } else {
      return false;
    }
  }

  public void setSetpointToCurrentPosition() {
    setTargetPosition(getPosition());
    m_autoMode = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double speed = 0;
      if (m_autoMode) {
        speed = m_climberPidController.calculate(getPosition());
        if (m_climberPidController.atSetpoint()) {
          speed = 0;
        }
      } else {
        speed = m_manualPower;
      }
      moveClimber(speed);
      SmartDashboard.putNumber("climber/Climber Position", getPosition());
      SmartDashboard.putNumber("climber/Power Output", speed);
      SmartDashboard.putBoolean("climber/isAtTopLimit", isAtTopLimit());
      SmartDashboard.putBoolean("climber/isAtBottomLimit", isAtBottomLimit());
  }
  
}
