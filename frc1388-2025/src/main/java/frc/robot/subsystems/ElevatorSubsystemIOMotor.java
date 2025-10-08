// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorSubsystemIOMotor implements ElevatorSubsystemIO {
    private final ElevatorSim elevatorSim = new ElevatorSim(0, 0, 0, 0, 0, 0, false, 0, null
        
    )


    // private final SparkFlex m_elevatorMotor;

    // public ElevatorSubsystemIOMotor() {
    //     m_elevatorMotor = new SparkFlex(Constants.RobotContainerConstants.kElevatorMotorCANID, MotorType.kBrushless); 

    // }
    
}
