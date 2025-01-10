// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;


public class ElevatorSubsytem extends SubsystemBase {
  /** Creates a new ElevatorSubsytem. */

  // Bottom sparkmax canid: 8 on left side looking at the motor
  // Top sparkmax canid: 7 on the right side
  //positive goes up negative goes down
  // bottom limit switch is DIO 8 top is 9
  SparkMax m_rightMotor;
  SparkMax m_leftMotor;
  DigitalInput m_topLimitSwitch;
  DigitalInput m_bottomLimitSwitch;
  LaserCan m_laserCan;

  public ElevatorSubsystem(SparkMax rightMotor, SparkMax leftMotor, LaserCan laserCan, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch) {
    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;
    m_topLimitSwitch = topLimitSwitch;
    m_bottomLimitSwitch = bottomLimitSwitch;
    m_laserCan = laserCan;
  }
  public void moveElevator(double power) {
    if((m_topLimitSwitch.get() == true) || (m_bottomLimitSwitch.get() == true)) {
      m_rightMotor.set(0);
      m_leftMotor.set(0);
    } else {
      m_leftMotor.set(-power); //invert power so negative goes up and positive goes down
      m_rightMotor.set(-power); //same
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
