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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldLayout;
import frc.robot.commands.AutoAllign;
import frc.robot.commands.AutoGoToPoint;
import frc.robot.commands.ElevatorSetpointCommand;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.commands.EndEffectorIntake;
import frc.robot.commands.EndEffectorShoot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AutoMethod extends SubsystemBase {
  private final AutoFactory m_autoFactory;
  private final AutoChooser m_autoChooser;

  /** Creates a new AutoMethod. */
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final EndEffectorSubsystem m_endEffectorSubsystem;
  private final Dashboard m_dashboard;
  private final Command m_choreoAuto;
  private final AutoRoutine m_choreoAutoRoutine;
  
  
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, Dashboard dashboard) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
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
    
    // m_autoChooser.addRoutine("OneScoreCenter", this::OneScoreCenter);
    // SmartDashboard.putData(m_autoChooser);

        // Schedule the selected auto during the autonomous period
        // RobotModeTriggers.autonomous().whileTrue(OneScoreCenter());
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

    AutoTrajectory startToScore = routine.trajectory("LayingEggsTop", 0);
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsTop", 1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsTop", 2);
    AutoTrajectory score2ToPickup = routine.trajectory("LayingEggsTop", 3);
    AutoTrajectory pickup2ToScore = routine.trajectory("LayingEggsTop", 4);

    routine.active().onTrue(
        Commands.sequence(
            startToScore.resetOdometry(),
            startToScore.cmd()));
    startToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen(score1ToPickup.cmd()));
    score1ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup1ToScore.cmd()));
    pickup1ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen((score2ToPickup.cmd())));
    score2ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup2ToScore.cmd()));
    pickup2ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)));

    return routine;
  }

  public AutoRoutine LayingEggsBottom() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsBottom");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsBottom", 0);
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsBottom", 1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsBottom", 2);
    AutoTrajectory score2ToPickup = routine.trajectory("LayingEggsBottom", 3);
    AutoTrajectory pickup2ToScore = routine.trajectory("LayingEggsBottom", 4);

    routine.active().onTrue(
        Commands.sequence(
            startToScore.resetOdometry(),
            startToScore.cmd()));
    routine.active().onTrue(
        Commands.sequence(
            startToScore.resetOdometry(),
            startToScore.cmd()));
    startToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen(score1ToPickup.cmd()));
    score1ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup1ToScore.cmd()));
    pickup1ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen((score2ToPickup.cmd())));
    score2ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup2ToScore.cmd()));
    pickup2ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)));

    return routine;
  }

  public AutoRoutine EndAt2Top() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsTop");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsTop", 0);
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsTop", 1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsTop", 2);

    routine.active().onTrue(
        Commands.sequence(
            startToScore.resetOdometry(),
            startToScore.cmd()));
    startToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen(score1ToPickup.cmd()));
    score1ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup1ToScore.cmd()));
    pickup1ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)));

    return routine;
  }

  public AutoRoutine EndAt2Bottom() {
    AutoRoutine routine = m_autoFactory.newRoutine("LayingEggsBottom");

    AutoTrajectory startToScore = routine.trajectory("LayingEggsBottom", 0);
    AutoTrajectory score1ToPickup = routine.trajectory("LayingEggsBottom", 1);
    AutoTrajectory pickup1ToScore = routine.trajectory("LayingEggsBottom", 2);

    routine.active().onTrue(
        Commands.sequence(
            startToScore.resetOdometry(),
            startToScore.cmd()));
    startToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).andThen(score1ToPickup.cmd()));
   score1ToPickup.done()
        .onTrue(new EndEffectorIntake(m_endEffectorSubsystem).withTimeout(1.5).andThen(pickup1ToScore.cmd()));
    pickup1ToScore.done()
        .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
            .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
            .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)));
    return routine;
  }

  // public AutoRoutine OneScoreCenter() {
  //   AutoRoutine routine = m_autoFactory.newRoutine("OneScoreCenter");

  //   AutoTrajectory startToScore = routine.trajectory("OneScoreCenter", 0);

  //   routine.active().onTrue(
  //       Commands.sequence(
  //           startToScore.resetOdometry(),
  //           startToScore.cmd()));
  //   // startToScore.done()
  //   //     .onTrue(new AutoAllign(m_driveTrainSubsystem).alongWith(new ElevatorSetpointCommand(m_elevatorSubsystem, true))
  //   //         .andThen(new EndEffectorShoot(m_endEffectorSubsystem))
  //   //         .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)));
  //   return routine;
  // }

  public Command OneScoreCenter() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(5.77, 4.19, 180, m_driveTrainSubsystem).withTimeout(4).andThen(
          new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 5.77, FieldLayout.FIELD_WIDTH - 4.19, 0, m_driveTrainSubsystem)
          .withTimeout(2.5).andThen(
              new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
                  .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    }
  }

  public Command OneScoreLeft() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(4.97, 5.23, 240, m_driveTrainSubsystem).withTimeout(4).andThen(
          new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 4.97, FieldLayout.FIELD_WIDTH - 5.23, 60, m_driveTrainSubsystem)
          .withTimeout(4).andThen(
              new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
                  .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    }
  }

  public Command OneScoreRight() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(5.28, 3.02, 120, m_driveTrainSubsystem).withTimeout(4).andThen(
          new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 5.28, FieldLayout.FIELD_WIDTH - 3.02, 60, m_driveTrainSubsystem)
          .withTimeout(4).andThen(
              new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1))
                  .andThen(new EndEffectorShoot(m_endEffectorSubsystem).withTimeout(1));
    }
  }

  public Command TwoScoreRight() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(5.28, 3.02, 120, m_driveTrainSubsystem).withTimeout(4).andThen(
          new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout(1)
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1)
              .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).withTimeout(1).andThen(
                  new AutoGoToPoint(1.60, 0.583, 54.6, m_driveTrainSubsystem))
              .alongWith(new EndEffectorIntake(m_endEffectorSubsystem)).withTimeout(4)
              .andThen(new AutoGoToPoint(4.01, 2.82, 60, m_driveTrainSubsystem).withTimeout(4))
              .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout(1)
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1);
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 5.28, FieldLayout.FIELD_WIDTH - 3.02, 300, m_driveTrainSubsystem)
          .withTimeout(4).andThen(
              new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout(1)
                  .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1).andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).withTimeout(1).andThen(
                    new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 1.60, FieldLayout.FIELD_WIDTH - 0.583, 245.4, m_driveTrainSubsystem))
                .alongWith(new EndEffectorIntake(m_endEffectorSubsystem)).withTimeout(4)
                .andThen(new AutoGoToPoint(4.01, 2.82, 240, m_driveTrainSubsystem).withTimeout(4))
                .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout((1))
                .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1);
    }
  }

  public Command TwoScoreLeft() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(4.97, 5.23, 240, m_driveTrainSubsystem).withTimeout(4).andThen(
          new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1)
              .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1))
          .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).withTimeout(1).andThen(
              new AutoGoToPoint(1.51, 7.39, 306.4, m_driveTrainSubsystem))
          .alongWith(new EndEffectorIntake(m_endEffectorSubsystem)).withTimeout(4)
          .andThen(new AutoGoToPoint(3.96, 5.23, 300, m_driveTrainSubsystem).withTimeout(4))
          .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout((1))
          .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1);
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 4.97, FieldLayout.FIELD_WIDTH - 5.23, 60,
          m_driveTrainSubsystem)
          .withTimeout(4).andThen(
              new ElevatorSetpointCommand(m_elevatorSubsystem, true).withTimeout(1)
                  .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1))
          .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, false)).withTimeout(1).andThen(
              new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 1.51, FieldLayout.FIELD_WIDTH - 7.39, 125.4, m_driveTrainSubsystem))
          .alongWith(new EndEffectorIntake(m_endEffectorSubsystem)).withTimeout(4)
          .andThen(new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 3.96, FieldLayout.FIELD_WIDTH - 5.23, 120, m_driveTrainSubsystem).withTimeout(4))
          .andThen(new ElevatorSetpointCommand(m_elevatorSubsystem, true)).withTimeout((1))
          .andThen(new EndEffectorShoot(m_endEffectorSubsystem)).withTimeout(1);
    }
  }


  public Command Leave() {
    if (Alliance.Blue == DriverStation.getAlliance().get()) {
      return new AutoGoToPoint(5.77, 4.19, 180, m_driveTrainSubsystem);
    } else {
      return new AutoGoToPoint(FieldLayout.FIELD_LENGTH - 5.77, FieldLayout.FIELD_WIDTH - 4.19, 0, m_driveTrainSubsystem);
    }
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoConstants.Objective objective = m_dashboard.getObjective();
    DataLogManager.log("####### objective:" + objective);

    if (objective == null) {
      return null;
    }

    switch (objective) {

      // case LAYINGEGGSBOTTOM:
      //   return LayingEggsBottom();

      // case LAYINGEGGSTOP:
      //   return LayingEggsTop();

      case TWOSCORELEFT:
        return TwoScoreLeft();

      case TWOSCORERIGHT:
        return TwoScoreRight();

      case LEAVE:
        return Leave();

      case ONESCORECENTER:
        return OneScoreCenter();
      
      case ONESCORELEFT:
        return OneScoreLeft();
      
      case ONESCORERIGHT:
        return OneScoreRight();

      // case CHOREOAUTOROUTINE:
      //   return ChoreoAutoRoutine();
    }
    return null;
  }
}
