// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoTurn extends Command {

  private final double m_turnAngled;
  private final DriveTrainSubsystem m_driveTrain;
  private PIDController m_rotationController = new PIDController(0.03, 0, 0);


  /** Creates a new AutoTurbn. */
  public AutoTurn(double turnAngle, DriveTrainSubsystem driveTrainSubsystem) {
    m_turnAngled = turnAngle;
    m_driveTrain = driveTrainSubsystem;
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.drive(0, 0, m_rotationController.calculate(m_driveTrain.getAngle() - 360 + m_turnAngled));
    // m_driveTrain.drive(0, 0, 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getAngle() - 360 + m_turnAngled) < 5;
  }
}
