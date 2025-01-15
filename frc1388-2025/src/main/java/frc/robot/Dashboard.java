// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Objective;

public class Dashboard extends SubsystemBase {
  private final ShuffleboardTab m_shuffleboardTab;
    private final static String SHUFFLEBOARD_TAB_NAME = "Competition";

    private final ComplexWidget m_complexWidgetObjective;
    private static SendableChooser<Objective> m_autoObjective = new SendableChooser<>();

    /** Creates a new Dashboard. */
    public Dashboard() {
      m_shuffleboardTab = Shuffleboard.getTab(SHUFFLEBOARD_TAB_NAME);
      Shuffleboard.selectTab(SHUFFLEBOARD_TAB_NAME);



      for (AutoConstants.Objective o : Objective.values()) {
        m_autoObjective.addOption(o.getDashboardDescript(), o);
      }
      m_autoObjective.setDefaultOption(Objective.Default.getDashboardDescript(), Objective.Default);
      m_complexWidgetObjective = m_shuffleboardTab.add("AutoObjective", m_autoObjective)
          .withWidget(BuiltInWidgets.kComboBoxChooser)
          .withSize(8, 4)
          .withPosition(20, 4);
    }

  public Objective getObjective() {
    return m_autoObjective.getSelected();
  }
}
