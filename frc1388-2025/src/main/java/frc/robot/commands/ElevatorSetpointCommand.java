// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoints;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetpointCommand extends Command {
  private ElevatorSubsystem m_elevatorSubsystem;
  private boolean m_isLevel4;
  /** Creates a new ElevatorSetpointCommand. */
  public ElevatorSetpointCommand(ElevatorSubsystem elevatorSubsystem, boolean isLevel4) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_isLevel4 = isLevel4;

    addRequirements(m_elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_isLevel4) {
    m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL4);
    }
    else {
      m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.isAtSetpoint();
  }
}
