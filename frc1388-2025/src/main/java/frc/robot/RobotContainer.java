// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.AutoAllignRight;
import frc.robot.commands.AutoAllignL1;
import frc.robot.commands.AutoAllignLeft;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import com.revrobotics.spark.SparkFlex;
import frc.robot.vision.Limelight;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final boolean robot2025 = true;

  private final Dashboard m_dashboard = new Dashboard();

  private final Limelight m_limeLight = new Limelight("limelight-front", "limelight-back", "limelight-left");

  // Subsystems
  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
      new SwerveModule(
          new TalonFX(DriveTrainConstants.FRONT_RIGHT_DRIVE_MOTOR_CANID),
          new TalonFX(DriveTrainConstants.FRONT_RIGHT_ROTATION_MOTOR_CANID),
          new TalonFXConfiguration(), new TalonFXConfiguration(),
          new CANcoder(DriveTrainConstants.FRONT_RIGHT_CANCODER),
          Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(DriveTrainConstants.FRONT_LEFT_DRIVE_MOTOR_CANID),
          new TalonFX(DriveTrainConstants.FRONT_LEFT_ROTATION_MOTOR_CANID),
          new TalonFXConfiguration(), new TalonFXConfiguration(),
          new CANcoder(DriveTrainConstants.FRONT_LEFT_CANCODER),
          Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(DriveTrainConstants.BACK_LEFT_DRIVE_MOTOR_CANID),
          new TalonFX(DriveTrainConstants.BACK_LEFT_ROTATION_MOTOR_CANID),
          new TalonFXConfiguration(), new TalonFXConfiguration(),
          new CANcoder(DriveTrainConstants.BACK_LEFT_CANCODER),
          Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(DriveTrainConstants.BACK_RIGHT_DRIVE_MOTOR_CANID),
          new TalonFX(DriveTrainConstants.BACK_RIGHT_ROTATION_MOTOR_CANID),
          new TalonFXConfiguration(), new TalonFXConfiguration(),
          new CANcoder(DriveTrainConstants.BACK_RIGHT_CANCODER),
          Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0)),

      new Pigeon2(DriveTrainConstants.PIGEON_CANID), m_limeLight);

  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(
      new SparkFlex(Constants.RobotContainerConstants.kElevatorMotorCANID, MotorType.kBrushless), // motor
      new DigitalInput(Constants.RobotContainerConstants.kElevatorTopLimitChannel), // toplimitswitch
      new DigitalInput(Constants.RobotContainerConstants.kElevatorBottomLimitChannel) // bottomlimitswitch
  );

  ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(
      new SparkFlex(Constants.RobotContainerConstants.kClimberMotorCANID, MotorType.kBrushless),
      new DutyCycleEncoder(Constants.RobotContainerConstants.kClimberAbsoluteEncoderDIO));

  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem(
      new SparkMax(RobotContainerConstants.kEndEffectorCANID, MotorType.kBrushless),
      new LaserCan(RobotContainerConstants.kLaserCanCANID));

  DriveCommand m_driveCommand;
  ElevatorCommand m_elevatorCommand;
  ClimberCommand m_climberCommand;
  EndEffectorCommand m_endEffectorCommand;

  private final AutoMethod m_autoMethod;

  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */ 
  public RobotContainer() {

    // Commands
    m_autoMethod = new AutoMethod(m_driveTrain, m_elevatorSubsystem, m_endEffectorSubsystem, m_dashboard);

    DriveCommand m_driveCommand = new DriveCommand(
        m_driveTrain,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getHID().getAButton());
    m_driveTrain.setDefaultCommand(m_driveCommand);

    m_elevatorCommand = new ElevatorCommand(
        m_elevatorSubsystem,
        () -> m_operatorController.getLeftY(),
        () -> m_operatorController.getHID().getAButton(),
        () -> m_operatorController.getHID().getBButton(),
        () -> m_operatorController.getHID().getXButton(),
        () -> m_operatorController.getHID().getYButton(),
        () -> m_operatorController.getHID().getLeftBumperButton(),
        () -> m_operatorController.getHID().getRightBumperButton());
    m_elevatorSubsystem.setDefaultCommand(m_elevatorCommand);

    m_climberCommand = new ClimberCommand(
       m_climberSubsystem,
        () -> m_operatorController.getRightY(),
        () -> m_operatorController.getHID().getPOV() == 0,
        () -> m_operatorController.getHID().getPOV() == 180,
        () -> m_operatorController.getHID().getPOV() == 90);
    m_climberSubsystem.setDefaultCommand(m_climberCommand);

    m_endEffectorCommand = new EndEffectorCommand(
        m_endEffectorSubsystem,
        // () -> m_driverController.getHID().getLeftTriggerAxis(),
        // () -> m_driverController.getHID().getLeftBumperButton(), 
        () -> m_driverController.getHID().getRightTriggerAxis(),
        () -> m_driverController.getHID().getRightBumperButton());
    m_endEffectorSubsystem.setDefaultCommand(m_endEffectorCommand);

setBrakeMode(true);

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
    // Silence warnings if controllers aren't plugged in
    DriverStation.silenceJoystickConnectionWarning(true);

    m_driverController.leftBumper().whileTrue(new AutoAllignRight(m_driveTrain));
    m_driverController.leftTrigger().whileTrue(new AutoAllignLeft(m_driveTrain));
    m_driverController.a().whileTrue(new AutoAllignL1(m_driveTrain));
  }



  public void setAllEncoderOffsets() {
    m_driveTrain.setAllEncoderOffsets();
  }

  public void setBrakeMode(boolean brakeMode) {
    m_driveTrain.setBrakeMode(brakeMode);
  }

  public void resetSubsystemsAndCommands() {
    m_elevatorSubsystem.resetElevatorSubsystem();
    m_elevatorCommand.resetElevatorCommand();

  // if (robot2025) {
  //   m_driverController.rightTrigger().whileTrue(m_endEffectorCommand);
  // }
  }

  public void resetGyro() {
    if (m_limeLight.getApriltagTargetFoundFront()) {
      m_driveTrain.limelightResetGyroFront();
    }
  }

  public Command getAutonomousCommand() {
    return m_autoMethod.getAutonomousCommand();
  }
}
