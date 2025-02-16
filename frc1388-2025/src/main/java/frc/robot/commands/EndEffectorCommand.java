// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  private final EndEffectorSubsystem m_endEffectorSubsystem;
  // private final Supplier<Double> m_leftTrigger;
  // private final Supplier<Boolean> m_leftBumper;
  private final Supplier<Double> m_rightTrigger;
  private final Supplier<Boolean> m_rightBumper;
  private final Timer m_endEffectorTimer = new Timer();
  /** Creates a new EndEffectorCommand. */
  // Supplier<Double> leftTrigger, Supplier<Boolean> leftBumper,
  public EndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem, Supplier<Double> rightTrigger, Supplier<Boolean> rightBumper) {
  m_endEffectorSubsystem = endEffectorSubsystem;
    // m_leftTrigger = leftTrigger;
    // m_leftBumper = leftBumper;
    m_rightTrigger = rightTrigger;
    m_rightBumper = rightBumper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffectorSubsystem);
  }
//TODO all left trigger and left bumper stuff is for algae intaking
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_endEffectorSubsystem.isAtEffectorLimit() == true) {
      m_endEffectorTimer.restart();;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO change timer values once programming gets robot
    double rightTrigger = m_rightTrigger.get();
    if (m_endEffectorTimer.get() > 1) {
      m_endEffectorSubsystem.RunEndEffector(0.25);
      // m_endEffectorSubsystem.RunEndEffector(0.25);
      // m_endEffectorSubsystem.isAtEffectorLimit();
    } 
    else if (m_rightBumper.get()) {
      m_endEffectorSubsystem.RunEndEffectorIgnoreLimit(0.25);  
    }
    
        
      

    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
