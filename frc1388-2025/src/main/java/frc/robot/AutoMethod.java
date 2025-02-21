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
  private final AutoAllign m_allign;
  
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, AutoAllign allign) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_allign = allign;
    
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
        m_autoChooser.addRoutine("LayingEggsTop", this::LayingEggsTop);
        m_autoChooser.addRoutine("LayingEggsBottom", this::LayingEggsBottom);
        m_autoChooser.addRoutine("OneScoreCenter", this::OneScoreCenter);
        m_autoChooser.addRoutine("EndAt2Top", this::EndAt2Top);
        m_autoChooser.addRoutine("EndAt2Bottom", this::EndAt2Bottom);

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
  

  public AutoRoutine LayingEggsTop() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsTop");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsTop",0); 
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsTop",1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsTop",2);
    AutoTrajectory score2ToPickup = routine.trajectory("LayingEggsTop",3);
    AutoTrajectory pickup2ToScore = routine.trajectory("LayingEggsTop",4);


    routine.active().onTrue(
      Commands.sequence(
        startToScore.resetOdometry(),
        startToScore.cmd()
      )
    );
      startToScore.done().onTrue(m_allign.andThen(score1ToPickup.cmd()));
      score1ToPickup.done().onTrue(pickup1ToScore.cmd());
      pickup1ToScore.done().onTrue(m_allign.andThen(score2ToPickup.cmd()));
      score2ToPickup.done().onTrue(pickup2ToScore.cmd());
      pickup2ToScore.done().onTrue(m_allign);
    return routine;
  }

  public AutoRoutine LayingEggsBottom() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsBottom");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsBottom",0); 
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsBottom",1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsBottom",2);
    AutoTrajectory score2ToPickup = routine.trajectory("LayingEggsBottom",3);
    AutoTrajectory pickup2ToScore = routine.trajectory("LayingEggsBottom",4);

    routine.active().onTrue(
      Commands.sequence(
        startToScore.resetOdometry(),
        startToScore.cmd()
      )
    );
      startToScore.done().onTrue(m_allign.andThen(score1ToPickup.cmd()));
      score1ToPickup.done().onTrue(pickup1ToScore.cmd());
      pickup1ToScore.done().onTrue(m_allign.andThen(score2ToPickup.cmd()));
      score2ToPickup.done().onTrue(pickup2ToScore.cmd());
      pickup2ToScore.done().onTrue(m_allign);
      return routine;
  }

  public AutoRoutine EndAt2Top() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsTop");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsTop",0); 
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsTop",1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsTop",2);

    routine.active().onTrue(
      Commands.sequence(
        startToScore.resetOdometry(),
        startToScore.cmd()
      )
    );
      startToScore.done().onTrue(m_allign.andThen(score1ToPickup.cmd()));
      score1ToPickup.done().onTrue(pickup1ToScore.cmd());
      pickup1ToScore.done().onTrue(m_allign);
      return routine;
  }

  public AutoRoutine EndAt2Bottom() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsBottom");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsBottom",0); 
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsBottom",1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsBottom",2);

    routine.active().onTrue(
      Commands.sequence(
        startToScore.resetOdometry(),
        startToScore.cmd()
      )
    );
      startToScore.done().onTrue(m_allign.andThen(score1ToPickup.cmd()));
      score1ToPickup.done().onTrue(pickup1ToScore.cmd());
      pickup1ToScore.done().onTrue(m_allign);
      return routine;
  }

  public AutoRoutine OneScoreCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("OneScoreCenter");

    AutoTrajectory startToScore = routine.trajectory("OneScoreCenter",0); 

    routine.active().onTrue(
      Commands.sequence(
        startToScore.resetOdometry(),
        startToScore.cmd()
      )
    );
      startToScore.done().onTrue(m_allign);
      return routine;
  }


  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   AutoConstants.Objective objective = m_dashboard.getObjective();
  //       DataLogManager.log("####### objective:" + objective);
    
  //       if (objective == null) {
  //         return null;
  //       }
      
  //       switch (objective) {
    
  //       case SITSTILL:
  //           return SitStillLookPretty();
      
  //       case CHOREOAUTO:
  //           return m_choreoAuto;
  //       }
  //     return null;
  //   }
}
