// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.AutoAllign;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Limelight m_limeLight = new Limelight("limelight-shooter", "limelight-intake");
  
    private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
          new SwerveModule(
              new TalonFX(DriveTrainConstants.FRONT_RIGHT_DRIVE_MOTOR_CANID),
              new TalonFX(DriveTrainConstants.FRONT_RIGHT_ROTATION_MOTOR_CANID),
              new TalonFXConfiguration(), 
              new CANcoder(DriveTrainConstants.FRONT_RIGHT_CANCODER),
              Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)),
          new SwerveModule(
              new TalonFX(DriveTrainConstants.FRONT_LEFT_DRIVE_MOTOR_CANID),
              new TalonFX(DriveTrainConstants.FRONT_LEFT_ROTATION_MOTOR_CANID),
              new TalonFXConfiguration(), 
              new CANcoder(DriveTrainConstants.FRONT_LEFT_CANCODER),
                            Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)),
          new SwerveModule(
              new TalonFX(DriveTrainConstants.BACK_LEFT_DRIVE_MOTOR_CANID),
              new TalonFX(DriveTrainConstants.BACK_LEFT_ROTATION_MOTOR_CANID),
              new TalonFXConfiguration(),
              new CANcoder(DriveTrainConstants.BACK_LEFT_CANCODER),
                            Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)),
          new SwerveModule(
              new TalonFX(DriveTrainConstants.BACK_RIGHT_DRIVE_MOTOR_CANID),
              new TalonFX(DriveTrainConstants.BACK_RIGHT_ROTATION_MOTOR_CANID),
              new TalonFXConfiguration(),
              new CANcoder(DriveTrainConstants.BACK_RIGHT_CANCODER),
              Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0)),
          new Pigeon2(20), m_limeLight
      );

      private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

      private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DriveCommand m_driveCommand = new DriveCommand(
        m_driveTrain,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getHID().getAButton());
      
    m_driveTrain.setDefaultCommand(m_driveCommand);


    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    double closestPoseX = m_driveTrain.getClosestTargetPose().getX();
    double closestPoseY = m_driveTrain.getClosestTargetPose().getY();
    double closestPoseRotation = m_driveTrain.getClosestTargetPose().getRotation().getDegrees();
    SmartDashboard.putNumber("XPose", closestPoseX);
    SmartDashboard.putNumber("YPose", closestPoseY);
    SmartDashboard.putNumber("RotationPose", closestPoseRotation);
    m_driverController.b().whileTrue(new AutoAllign(closestPoseX, closestPoseY, closestPoseRotation, m_driveTrain));
  }

  public void setAllEncoderOffsets() {
    m_driveTrain.setAllEncoderOffsets();
  }

  public void setBrakeMode(boolean brakeMode) {
    m_driveTrain.setBrakeMode(brakeMode);
  }

}
