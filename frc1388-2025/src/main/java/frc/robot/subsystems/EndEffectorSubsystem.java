// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorSubsystemConstants;

import com.revrobotics.spark.SparkMax;
import au.grapplerobotics.LaserCan;


public class EndEffectorSubsystem extends SubsystemBase {
  SparkMax m_endEffectorMotor; 
  LaserCan m_laserCAN;
  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(SparkMax endEffectorMotor, LaserCan laserCAN) {
    m_endEffectorMotor = endEffectorMotor;
    m_laserCAN = laserCAN;
  }

private void RunEndEffector(double power) {
  m_endEffectorMotor.set(power);
}

public void IntakeCoral(double power) {
  if (isCoralDetected() == true) {
    RunEndEffector(0);
  } else {
    RunEndEffector(power);
  }
}

//ignores lasercan limiting
public void ShootCoral(double power) {
  RunEndEffector(power);
}


public double getCoralHeight() {
 return (m_laserCAN.getMeasurement().distance_mm * EndEffectorSubsystemConstants.kInchesPerMillimeters); //converting to inches
}

public boolean isCoralDetected() {
  if (getCoralHeight() > EndEffectorSubsystemConstants.kCoralDetectionHeight) {
    return false;
  } else {
    return true;
  }
}

// public boolean isAtEffectorLimit() {
//   return isCoralDetected();
// }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("endeffector/coralheight", getCoralHeight());

    // This method will be called once per scheduler run
    // if (isAtEffectorLimit()) {
    //   RunEndEffector(0);
    // }
  }
}
