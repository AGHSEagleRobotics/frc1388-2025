// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
  private final Supplier<Boolean> m_start;

  private final PIDController m_xController = new PIDController(1.8, 0, 0);
  private final PIDController m_yController = new PIDController(1.8, 0, 0);
  
  private PIDController m_turnPidController = new PIDController(0.14, 0.000012, 0.000012);
  private final Supplier<Boolean> m_a;

  private final SlewRateLimiter m_xAccLimiter = new SlewRateLimiter(0.2);
  private final SlewRateLimiter m_yAccLimiter = new SlewRateLimiter(0.2);
  private boolean m_lastAButtonPressed = false; // used for edge detection 
  private boolean m_lineUp = false;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem driveTrain, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> a, Supplier<Boolean> start) {
    m_driveTrain = driveTrain;
    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;
    m_a =  a;
    m_start = start;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnPidController.setTolerance(2.5);
    m_turnPidController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftX = MathUtil.applyDeadband(m_leftX.get(), DriveTrainConstants.CONTROLLER_DEADBAND);
    double leftY = MathUtil.applyDeadband(m_leftY.get(), DriveTrainConstants.CONTROLLER_DEADBAND);
    double rightX = -MathUtil.applyDeadband(m_rightX.get(), DriveTrainConstants.CONTROLLER_DEADBAND);

    if(m_start.get()) {
      m_driveTrain.limelightResetGyroFront();
      m_driveTrain.limelightResetGyroBack();
    }

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      leftX = -leftX;
      leftY = -leftY;
    }
    
    // velocities from controller inputs
    double xVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftY, DriveTrainConstants.LEFT_STICK_SCALE);
    double yVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftX, DriveTrainConstants.LEFT_STICK_SCALE);
    /** angular velocity */
    double omega = 2 * Math.PI * scale(rightX, 2.5);

    boolean a = m_a.get();

    if (a && !m_lastAButtonPressed) {
      m_lineUp = !m_lineUp;
    }
    m_lastAButtonPressed = a;

    // if (rightX != 0) { // default turning with stick

      // m_lineUp = false;
    // }

    // else if (m_lineUp) {
    //   xVelocity = m_driveTrain.getXVelocityAuto(m_driveTrain.getClosestTargetPose().getX(), m_xController,
    //       m_xAccLimiter) / 10;
    //   yVelocity = m_driveTrain.getYVelocityAuto(m_driveTrain.getClosestTargetPose().getY(), m_yController,
    //       m_yAccLimiter) / 10;
    //   omega = m_driveTrain.getOmegaVelocityAuto(m_turnPidController);
    // }

    m_driveTrain.drive(xVelocity, yVelocity, omega);

  }

   /** This method uses the tangent function to scale the input
   * <p>
   * Example: <a>https://www.desmos.com/calculator/kntlmrgprn</a>
   * 
   * @param in the raw input
   * @param scale the scaling number (represented as a in the example)
   * @return the scaled output
   */
  private double scale(double in, double scale) {
    return Math.tan(in * Math.atan(scale)) / scale;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
