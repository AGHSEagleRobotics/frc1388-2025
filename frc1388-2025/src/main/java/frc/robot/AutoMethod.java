// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;;

public class AutoMethod extends SubsystemBase {
  
  /** Creates a new AutoMethod. */
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;
  
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;


  }

  public Command SitStillLookPretty(){
    return null;
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
      }
      return null;
    }

    public Command makeSwerveAutoCommand(String pathString) {
      return null;
}
}
