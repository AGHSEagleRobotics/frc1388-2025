// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final Supplier<Double> m_rightY; 
  private final Supplier<Boolean> m_DPadUp;
  private final Supplier<Boolean> m_DPadDown;
  private final Supplier<Boolean> m_DPadRight;
  private boolean m_autoMode = false;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem climberSubsystem, Supplier<Double> rightY, Supplier<Boolean> DPadUp, Supplier<Boolean> DPadDown, Supplier<Boolean> DPadRight) {
    m_climberSubsystem = climberSubsystem;
    m_rightY = rightY;
    m_DPadUp = DPadUp;
    m_DPadDown = DPadDown;
    m_DPadRight = DPadRight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//TODO: call setSetpointToCurrentPosition() so climber doesn't move when enabling the robot
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightY = MathUtil.applyDeadband(m_rightY.get(), ClimberConstants.CLIMBER_CONTROLLER_DEADBAND);
    if(m_DPadDown.get()) {
      m_autoMode = true;
      m_climberSubsystem.setSetpoint(ClimberConstants.CLIMBER_DOWN_POSITION);
    }
    else if(m_DPadUp.get()) {
      m_autoMode = true;
      m_climberSubsystem.setSetpoint(ClimberConstants.CLIMBER_UP_POSITION);
    }
    else if(m_DPadRight.get()) {
      m_autoMode = true;
      m_climberSubsystem.setSetpoint(ClimberConstants.CLIMBER_CENTER_POSITION);
    }
    else if (rightY > 0 || rightY < 0) {
      m_autoMode = false;
      m_climberSubsystem.setManualPower(rightY);
    } 
    else if (!m_autoMode && rightY == 0) {
        m_autoMode = true;
        // m_elevatorSubsystem.setManualPower(0);
        m_climberSubsystem.setSetpointToCurrentPosition();
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
