
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import au.grapplerobotics.CanBridge;
import choreo.auto.AutoRoutine;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private int m_userButtonCounter = 0;

  private final RobotContainer m_robotContainer;
  private AutoRoutine m_autonomousCommand;

  private Timer m_neutralModeTimer = new Timer();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // CanBridge.runTCP(); //for grapplehook/lasercan
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
   @Override
   public void robotInit() {
     DataLogManager.start();
     DataLogManager.log("####### RobotInit");
     DataLogManager.log("Git version: " + BuildInfo.GIT_VERSION + " (branch: " + BuildInfo.GIT_BRANCH + " "
         + BuildInfo.GIT_STATUS + ")");
     DataLogManager.log("      Built: " + BuildInfo.BUILD_DATE + "  " + BuildInfo.BUILD_TIME);

     // logs everytime a command starts / stops
     CommandScheduler.getInstance()
         .onCommandInitialize(command -> DataLogManager.log("++ " + command.getName() + " Initialized"));
     CommandScheduler.getInstance()
         .onCommandInterrupt(command -> DataLogManager.log("-- " + command.getName() + " Interrupted"));
     CommandScheduler.getInstance()
         .onCommandFinish(command -> DataLogManager.log("-- " + command.getName() + " Finished"));
   }
 
 
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Actions to perform when user button on RoboRio is pressed
    if (RobotController.getUserButton()) {
      // User button is pressed
      m_userButtonCounter += 1;

      if (m_userButtonCounter == 1) {
        DataLogManager.log("### UserButtonPressed");
        // RobotController.setRadioLEDState(RadioLEDState.kGreen);
      } else if (m_userButtonCounter == 100) { // when held for 2 seconds (100 tics)
        DataLogManager.log("### UserButtonHeld");
        RobotController.setRadioLEDState(RadioLEDState.kOrange);
        m_robotContainer.setAllEncoderOffsets();
      }
    } else {
      // User button is not pressed
      if (m_userButtonCounter > 0) {
        // button has just been released
        m_userButtonCounter = 0;
        RobotController.setRadioLEDState(RadioLEDState.kOff);
      }
    }
  }
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_neutralModeTimer.restart();
    m_robotContainer.resetSubsystemsAndCommands();
  }

  @Override
  public void disabledPeriodic() {
    if (m_neutralModeTimer.hasElapsed(5)) {
      m_robotContainer.setBrakeMode(false);
      m_neutralModeTimer.reset();
      m_neutralModeTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    DataLogManager.log("####### Autonomous Init");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    DataLogManager.log("setting neutral mode");
    m_robotContainer.setBrakeMode(true);
    DataLogManager.log("done setting neutral mode");

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("starting auto command");
    }


    // Get match info from FMS
    if (DriverStation.isFMSAttached()) {
      String fmsInfo = "FMS info: ";
      fmsInfo += " " + DriverStation.getEventName();
      fmsInfo += " " + DriverStation.getMatchType();
      fmsInfo += " match " + DriverStation.getMatchNumber();
      fmsInfo += " replay " + DriverStation.getReplayNumber();
      fmsInfo += ";  " + DriverStation.getAlliance() + " alliance";
      fmsInfo += ",  Driver Station " + DriverStation.getLocation();
      DataLogManager.log(fmsInfo);
    } else {
      DataLogManager.log("FMS not connected");

      DataLogManager.log("Match type:\t" + DriverStation.getMatchType());
      DataLogManager.log("Event name:\t" + DriverStation.getEventName());
      DataLogManager.log("Alliance:\t" + DriverStation.getAlliance());
      DataLogManager.log("Match number:\t" + DriverStation.getMatchNumber());
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }

    m_robotContainer.setBrakeMode(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
