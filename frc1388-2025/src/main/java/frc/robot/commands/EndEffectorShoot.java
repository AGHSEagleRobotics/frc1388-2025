// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorCommandConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/**
 * @ brief  EndEffectorShoot command is intended for autonomous use
 */
public class EndEffectorShoot extends Command {
  private EndEffectorSubsystem m_endEffectorSubsystem;

  private int m_endCounter;

  /** Creates a new EndEffectorShoot. */
  public EndEffectorShoot(EndEffectorSubsystem endEffectorSubsystem) {
    m_endEffectorSubsystem = endEffectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endEffectorSubsystem.ShootCoral(EndEffectorCommandConstants.kShootCoralPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffectorSubsystem.ShootCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_endEffectorSubsystem.isCoralDetected()) {
      m_endCounter = 0;
    }
    else {
      m_endCounter++;
    }

    return (m_endCounter >= EndEffectorCommandConstants.kEndCount);
  }
}
