// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoAllign;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoMethod extends SubsystemBase {
  private final AutoFactory m_autoFactory;
  private final AutoChooser m_autoChooser;

  /** Creates a new AutoMethod. */
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;
  private final Command m_choreoAuto;
  private final AutoRoutine m_choreoAutoRoutine;
  
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;

    m_autoFactory = new AutoFactory(
            m_driveTrainSubsystem::getPose, // A function that returns the current robot pose
            m_driveTrainSubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
            m_driveTrainSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            m_driveTrainSubsystem // The drive subsystem
    );

    m_choreoAuto = ChoreoAuto();
    m_choreoAutoRoutine = ChoreoAutoRoutine();

    m_autoChooser = new AutoChooser();

        // Add options to the chooser
        m_autoChooser.addRoutine("Choreo Auto Routine", this::ChoreoAutoRoutine);
        m_autoChooser.addCmd("Choreo Auto", this::ChoreoAuto);

        // Put the auto chooser on the dashboard
        SmartDashboard.putData(m_autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
  }

  public Command SitStillLookPretty(){
    return null;
  }

  public Command ChoreoAuto() {
    return m_autoFactory.trajectoryCmd("choreoAuto");
  }

  public AutoRoutine ChoreoAutoRoutine() {
    AutoRoutine routine = m_autoFactory.newRoutine("choreoAutoRoutine");
    
    AutoTrajectory drive = routine.trajectory("drive");

    routine.active().onTrue(
      Commands.sequence(
        drive.resetOdometry(),
        drive.cmd()
      )
    );
    return routine;
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoConstants.Objective objective = m_dashboard.getObjective();
        DataLogManager.log("####### objective:" + objective);
    
        if (objective == null) {
          return null;
        }
      
        switch (objective) {
    
        case SITSTILL:
            return SitStillLookPretty();
      
        case CHOREOAUTO:
            return m_choreoAuto;
        }
      return null;
    }
}
