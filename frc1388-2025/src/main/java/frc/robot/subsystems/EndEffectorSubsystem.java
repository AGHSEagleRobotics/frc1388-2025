// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorSubsystem extends SubsystemBase {
  SparkMax m_endEffectorMotor; 
  DigitalInput m_endEffectorLimitSwitch;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(SparkMax endEffectorMotor, DigitalInput endEffectorLimitSwitch) {
    m_endEffectorMotor = endEffectorMotor;
    m_endEffectorLimitSwitch = endEffectorLimitSwitch;
  }

public void RunEndEffector(double power) {
  m_endEffectorMotor.set(power);

  if (isAtEffectorLimit()) {
    m_endEffectorMotor.set(0);
  }
}

private boolean isAtEffectorLimit() {
  return m_endEffectorLimitSwitch.get();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
