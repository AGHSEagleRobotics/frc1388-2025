// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModuleIO {

public default void setSwerveModuleStates(SwerveModuleState inputState) {}
  
public default void setBrakeMode(boolean brakeMode) {}

public default void setDriveSpeed(double inputSpeed) {}

public default void setRotationPosition(double angle) {}




}