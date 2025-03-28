// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

/** Add your docs here. */
public class Limelight {
    private static NetworkTable m_placerSideTable;
    private static NetworkTable m_intakeSideTable;
    private static NetworkTable m_placerSideTableLeft;
    private static double[] botPose0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    public Limelight(String placerSideName, String intakeSideName, String placerSideLeftName) {
    m_placerSideTable = NetworkTableInstance.getDefault().getTable(placerSideName);
    m_intakeSideTable = NetworkTableInstance.getDefault().getTable(intakeSideName);
    m_placerSideTableLeft = NetworkTableInstance.getDefault().getTable(placerSideLeftName);
    setShooterPipeline(0);
  }

  public void setShooterPipeline(double pipelineNumber) {
    NetworkTableEntry shooterPipeline = m_placerSideTable.getEntry("pipeline");
    shooterPipeline.setNumber(pipelineNumber);
  }

  public void setIntakePipeline(double pipelineNumber) {
    NetworkTableEntry intakePipeline = m_intakeSideTable.getEntry("pipeline");
    intakePipeline.setNumber(pipelineNumber);
  }

  public void setIMUMode(int number) {
    m_placerSideTable.getEntry("imumode_set").setNumber(number);
  }

   public double getAprilTagID() {
    NetworkTableEntry tid = m_placerSideTable.getEntry("tid");
    double m_tid = tid.getDouble(0);
    return m_tid;
   }

   public boolean getApriltagTargetFoundFront() {
    NetworkTableEntry tv = m_placerSideTable.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public boolean getApriltagTargetFoundBack() {
    NetworkTableEntry tv = m_intakeSideTable.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public boolean getApriltagTargetFoundFrontLeft() {
    NetworkTableEntry tv = m_placerSideTableLeft.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public boolean getIsNoteFound() {
    NetworkTableEntry tv = m_intakeSideTable.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public double getAprilTagTy() {
    NetworkTableEntry ty = m_placerSideTable.getEntry("ty");
    double m_ty = ty.getDouble(0.0);
    return m_ty;
  }

  public double getNoteTy() {
    NetworkTableEntry ty = m_intakeSideTable.getEntry("ty");
    double m_ty = ty.getDouble(0.0);
    return m_ty;
  }

  public double getAprilTagTx() {
    NetworkTableEntry tx = m_placerSideTable.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
    return m_tx;
  }

  public double getNoteTx() {
    NetworkTableEntry tx = m_intakeSideTable.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
    return m_tx;
  }

  public double[] getBotPoseFront() {
    double[] botPose;
      botPose = m_placerSideTable.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 18) {
        return botPose;
        }
        return botPose0;
  }

  public double[] getBotPoseBack() {
    double[] botPose;
      botPose = m_intakeSideTable.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 18) {
        return botPose;
        }
        return botPose0;
  }

  public double[] getBotPoseLeft() {
    double[] botPose;
      botPose = m_placerSideTableLeft.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 18) {
        return botPose;
        }
        return botPose0;
  }

  public double getBotPoseValue(double[] botPose, int index) {
    if (index < botPose.length) {
      return botPose[index];
    } else {
      return 0.0;
    }
  }

  public double[] getMegaTag2Front() {
    double[] botPose;
      botPose = m_placerSideTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 3) {
        return botPose;
        }
        return botPose0;
  }

  public double[] getMegaTag2Back() {
    double[] botPose;
      botPose = m_intakeSideTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 3) {
        return botPose;
        }
        return botPose0;
  }

  public double[] getMegaTag2Left() {
    double[] botPose;
      botPose = m_placerSideTableLeft.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 3) {
        return botPose;
        }
        return botPose0;
  }


  public void setLimelightLEDsOn(Boolean on) {
    if (on) {
      m_placerSideTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_BLINK);
      m_intakeSideTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_BLINK);
    } else {
      m_placerSideTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_OFF);
      m_intakeSideTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_OFF);
    }
  }



}
