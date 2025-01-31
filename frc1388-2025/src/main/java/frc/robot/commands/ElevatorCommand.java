// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorCommandConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetPoints;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Supplier<Double> m_leftY; 
    private final Supplier<Boolean> m_a;
    private final Supplier<Boolean> m_b;
    private final Supplier<Boolean> m_x;
    private final Supplier<Boolean> m_y;
    private boolean m_manualMode;
    private boolean m_autoMode;
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> leftY, Supplier<Boolean> a, Supplier<Boolean> b, Supplier<Boolean> x, Supplier<Boolean> y) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_leftY = leftY;
    m_a = a;
    m_b = b;
    m_x = x;
    m_y = y;
    m_manualMode = false;
    m_autoMode = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevatorSubsystem.setPosition(m_elevatorSubsystem.getLaserCanPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = MathUtil.applyDeadband(-m_leftY.get(), ElevatorCommandConstants.kElevatorDeadband);
    double position = m_elevatorSubsystem.getElevatorHeight() + (leftY * 20); //TODO: tune later
    if (m_a.get()) {
      m_manualMode = false;
      m_autoMode = true;
      m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL1);
    }
    else if (m_b.get()) {
      m_manualMode = false;
      m_autoMode = true;
      m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL2);
    }
    else if (m_x.get()) {
      m_manualMode = false;
      m_autoMode = true;
      m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL3);
    }
    else if (m_y.get()) {
      m_manualMode = false;
      m_autoMode = true;
      m_elevatorSubsystem.setSetpoint(ElevatorSetPoints.LEVEL4);
    }
    else if (leftY > 0 || leftY < 0) {
      m_elevatorSubsystem.setTargetPosition(position);
    } 

    // m_elevatorSubsystem.moveElevator(leftY);
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
