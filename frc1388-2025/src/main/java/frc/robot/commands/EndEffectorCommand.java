// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.EndEffectorCommandConstants;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  private final EndEffectorSubsystem m_endEffectorSubsystem;
  // private final Supplier<Double> m_leftTrigger;
  // private final Supplier<Boolean> m_leftBumper;
  private final Supplier<Double> m_rightTrigger;
  private final Supplier<Boolean> m_rightBumper;
  private final Supplier<Double> m_rightTriggerOperator;
  private CommandXboxController m_operatorController = null;
  private final Timer m_endEffectorTimer = new Timer();
  private boolean m_coralIsDetected = false;
  private boolean m_rightTriggerWasPressed = false;
  private boolean m_rightTriggerWasPressedOperator = false;

  /** Creates a new EndEffectorCommand. */
  // Supplier<Double> leftTrigger, Supplier<Boolean> leftBumper,
  public EndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem, Supplier<Double> rightTrigger,
      Supplier<Boolean> rightBumper, Supplier<Double> rightTriggerOperator) {
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_rightTrigger = rightTrigger;
    m_rightBumper = rightBumper;
    m_rightTriggerOperator = rightTriggerOperator;
    addRequirements(m_endEffectorSubsystem);
    // m_leftTrigger = leftTrigger;
    // m_leftBumper = leftBumper;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  //this constructor used for controller rumble
  public EndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem, Supplier<Double> rightTrigger,
  Supplier<Boolean> rightBumper, Supplier<Double> rightTriggerOperator, CommandXboxController operatorController) {
    this(endEffectorSubsystem,
         rightTrigger, 
         rightBumper, 
         rightTriggerOperator);
    m_operatorController = operatorController;
  }




//TODO all left trigger and left bumper stuff is for algae intaking
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_endEffectorSubsystem.isCoralDetected() == true) {
      //only run this 1st time coral is detected
      if (m_coralIsDetected == false) {
        m_endEffectorTimer.reset();
        m_coralIsDetected = true;
        System.out.println("Coral Detected First Time");
      }
    } else { //coral is not detected 
      //reset coral detected flag when coral no longer detected
      if (m_coralIsDetected == true) {
        m_coralIsDetected = false;
        System.out.println("Coral Not Detected Reset");
      }
    }

    if (m_coralIsDetected == true) {
      // //TODO change timer values once programming gets robot
      // // if(m_endEffectorTimer.get() > EndEffectorCommandConstants.kIntakeKillDelay) {
      //   SmartDashboard.putNumber("Timer", m_endEffectorTimer.get());
      //   // System.out.println("Intake Kill Delay");
      // }
      m_endEffectorSubsystem.ShootCoral(0);
      if (m_endEffectorTimer.get() < EndEffectorCommandConstants.kIntakeKillDelay) {
        m_operatorController.setRumble(RumbleType.kBothRumble, 1);
      } else {
        m_operatorController.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    double rightTrigger = m_rightTrigger.get();
    boolean rightBumper = m_rightBumper.get();
    double rightTriggerOperator = m_rightTriggerOperator.get();
    if (rightTrigger > EndEffectorCommandConstants.kRightTriggerPressed) {
      m_endEffectorSubsystem.ShootCoral(EndEffectorCommandConstants.kShootCoralPower);
      m_rightTriggerWasPressed = true;
      System.out.println("Right Trigger Pressed");
    } 
    else if (rightTriggerOperator > EndEffectorCommandConstants.kRightTriggerPressed) {
      m_endEffectorSubsystem.ShootCoralReverse(EndEffectorCommandConstants.kShootCoralPower);
      m_rightTriggerWasPressedOperator = true;
    }
    else if (m_rightTriggerWasPressed == true) {
      System.out.println("Right Trigger Released");
      m_endEffectorSubsystem.ShootCoral(0);
      m_rightTriggerWasPressed = false;
    }
    else if (m_rightTriggerWasPressedOperator == true) {
      m_endEffectorSubsystem.ShootCoral(0);
      m_rightTriggerWasPressedOperator = false;
    }
    else if (rightBumper == true) {
      //endeffectortimer\
      System.out.println("Right Bumper Pressed");
      m_endEffectorSubsystem.IntakeCoral(EndEffectorCommandConstants.kIntakeCoralPower);  
    }
    
  }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffectorSubsystem.ShootCoral(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
