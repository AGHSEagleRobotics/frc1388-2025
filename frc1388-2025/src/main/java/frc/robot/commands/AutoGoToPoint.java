// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/** this command moves the robot to an x, y location */
public class AutoGoToPoint extends Command {

  private final DriveTrainSubsystem m_driveTrain;

  private final double X_SETPOINT;
  private final double Y_SETPOINT;
  private final double ROTATION_SETPOINT;

  // i was 0.015
  private final PIDController m_xController = new PIDController(1.8, 0, 0);
  private double m_lastXSpeed = 0;
  private final SlewRateLimiter m_xAccLimiter = new SlewRateLimiter(0.2);

  private final PIDController m_yController = new PIDController(1.8, 0, 0);
  private double m_lastYSpeed = 0;
  private final SlewRateLimiter m_yAccLimiter = new SlewRateLimiter(0.2);

  private PIDController m_rotationController = new PIDController(0.002, 0, 0);


  /** Creates a new AutoMove. */
  public AutoGoToPoint(double xSetpoint, double ySetpoint, double rotationSetpoint, DriveTrainSubsystem drivetrain) {

    m_driveTrain =  drivetrain;

    X_SETPOINT = xSetpoint;
    Y_SETPOINT = ySetpoint;
    ROTATION_SETPOINT = rotationSetpoint;

    m_xController.setTolerance(0.05);
    m_yController.setTolerance(0.05);
    
    m_rotationController.setTolerance(3);
    m_rotationController.enableContinuousInput(0, 360);

    
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = m_xController.calculate(m_driveTrain.getPose().getX(), X_SETPOINT);
    if (xSpeed > m_lastXSpeed) {
      xSpeed = m_xAccLimiter.calculate(xSpeed);
    }

    double ySpeed = m_yController.calculate(m_driveTrain.getPose().getY(), Y_SETPOINT);
    if (ySpeed > m_lastYSpeed) {
      ySpeed = m_yAccLimiter.calculate(ySpeed);
    }

    SmartDashboard.putNumber("AutoGoToPoint/rot pid in", m_driveTrain.getAngle());
    SmartDashboard.putBoolean("AutoGoToPoint/is at rot sp", m_rotationController.atSetpoint());
    m_driveTrain.drive(xSpeed, ySpeed, m_rotationController.calculate(m_driveTrain.getAngle(), ROTATION_SETPOINT));
    m_lastXSpeed = xSpeed;
    m_lastYSpeed = ySpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atSetpoint() && m_yController.atSetpoint() && m_rotationController.atSetpoint();
    // return false;
  }
}
