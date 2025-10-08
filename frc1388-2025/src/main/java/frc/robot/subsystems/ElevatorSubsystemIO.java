// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoints;

/** Add your docs here. */
public interface ElevatorSubsystemIO {
    public default void moveElevator(double power) {}

    public default void setManualPower(double power) {}

    public default void setTargetPosition(double position) {}

    public default void getTargetPosition() {}

    public default void setSetpointToCurrentPosition() {}

    public default void setSetpoint(ElevatorSetPoints setpoint) {}

    public default void getElevatorHeight() {}

    public default void getMotorEncoderHeight() {}

    public default void getMotorEncoderVelocity() {}

    public default void resetEncoder() {}

    public default void isAtTopLimit() {}

    public default void isAtBottomLimit() {}

    public default void resetElevatorSubsystem() {}

    public default void isAtSetpoint() {}

}
