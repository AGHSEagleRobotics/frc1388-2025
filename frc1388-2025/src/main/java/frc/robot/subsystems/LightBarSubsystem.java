// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightBarSubsystem extends SubsystemBase {
  PowerDistribution m_PDH;
  /** Creates a new LightBarSubsystem. */
  public LightBarSubsystem(PowerDistribution PDH) {
    m_PDH = PDH;

    PDH.setSwitchableChannel(false);
  }

  public void SetLightBar(boolean on) {
    m_PDH.setSwitchableChannel(on);
  }

  public boolean GetLightBar() {
    return m_PDH.getSwitchableChannel();
  }

  public void ToggleLightBar() {
    m_PDH.setSwitchableChannel(!m_PDH.getSwitchableChannel());
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
