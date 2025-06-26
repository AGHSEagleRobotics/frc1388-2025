// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraight extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final double m_distanceInMeters;
  private double m_start;

  private final PIDController m_driveController = new PIDController(20, 0.015, 0);

  /** Creates a new AutoDrive. */
  public DriveStraight(double distanceInMeters, DriveTrainSubsystem driveTrainSubsystem) {
    m_driveTrain = driveTrainSubsystem;
    m_distanceInMeters = distanceInMeters;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start = m_driveTrain.getDistTraveled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double pidInput = -m_setPoint  + (m_driveTrain.getDistTraveled() - m_start);
    // SmartDashboard.putNumber("drive pid input", pidInput);
    // SmartDashboard.putString("auto drive info", "setpoint" + m_setPoint + "dt.getdist" + m_driveTrain.getDistTraveled() + "start" + m_start);
    double pidOutput = m_driveController.calculate(m_driveTrain.getDistTraveled() - m_start, m_distanceInMeters);
    pidOutput = MathUtil.clamp(pidOutput, -1, 1);
    m_driveTrain.differentialDrive(pidOutput);
    SmartDashboard.putNumber("AutoDrive/dist traveled", m_driveTrain.getDistTraveled() - m_start);
    SmartDashboard.putNumber("AutoDrive/total dist traveled", m_driveTrain.getDistTraveled());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getDistTraveled() - m_start) > Math.abs(m_distanceInMeters) - 0.01;
  }
}
