// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.random.RandomGenerator.JumpableGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

/** VisionAcceptor is a class that accepts vision data from the robot's camera.
 * It checks if the current position of the robot is valid based on the last known position,
 * the robot's velocity, and the angle between the robot's direction and the camera's position change.
 * It also checks if the robot is within the field boundaries and if the robot is moving too fast for the camera to update.
 * The class provides methods to determine if the vision data should be accepted and if the gyro should be reset.
 * It also calculates the norm of the robot's velocity to determine if the robot is moving.
 * This class is used to ensure that the robot's position is accurate and reliable before using it for navigation or other tasks.
 * It is designed to work with the MegaTag2 vision system, but can be adapted for other systems as well.
 * It is important to note that this class is not responsible for processing the vision data itself,
 * but rather for validating the data before it is used by other parts of the robot's software.
 */
public class VisionAcceptor {
    
    // Constants for the robot's margin of error when checking position
    public static final double robotMargin = 0.5;
    
    // Variables to store the last known position and velocity of the robot
    ChassisSpeeds m_robotVelocity = new ChassisSpeeds(0, 0, 0);
    int m_jumpCount = 0;
    int m_jumpCountMax = 0;
    double m_angle = 0;
    boolean m_isMegaTag2;


    public VisionAcceptor(boolean isMegaTag2) {
        m_isMegaTag2 = isMegaTag2;
    }

    // This method checks if the current position of the robot is valid based on the last known position,
    public boolean shouldAccept(Pose2d currentPosition, Pose2d lastPosition, ChassisSpeeds robotVelocity) {

        // first checks seeing if the robot velocity and current position are not null (otherwise the code will crash)
        if(m_robotVelocity == null || currentPosition == null) {
             System.out.println("null check");
            return false;
        }
        m_robotVelocity = robotVelocity;

        // checks if the current position is at the origin (0,0), which is not a valid position for the robot (means that limelight is not connected)
        if(currentPosition.getX() == 0 && currentPosition.getY() == 0) {
            return false;
        }

        // if this is the first ever check, then initialize class variable and trust the position
        if(lastPosition == null) {
            lastPosition = currentPosition;
            // System.out.println("first check");
            return true;
        }

        SmartDashboard.putNumber("difference of x", Math.abs(currentPosition.getX() - lastPosition.getX()));
        SmartDashboard.putNumber("difference of y", Math.abs(currentPosition.getY() - lastPosition.getY()));

        // checks if the current position is within the robot's margin of error from the last known position
        double velocityPerTick =  DriveTrainConstants.DISTANCE_PER_TICK;

        double velocityPerTickClamped = MathUtil.clamp(velocityPerTick, 0.05, velocityPerTick);

        double clamedJumpCount = MathUtil.clamp(m_jumpCount, 0, 1000);

        double velocityTimesJumpCount = velocityPerTickClamped * (clamedJumpCount + 1);

        SmartDashboard.putNumber("normalizedVelocity", norm());
        SmartDashboard.putNumber("velocityPerTick", velocityPerTick);
        SmartDashboard.putNumber("velocityPerTickClamped", velocityPerTickClamped);
        SmartDashboard.putNumber("jumpCountVelocity", velocityTimesJumpCount);

        // check if the current position compared to the last position is greater than the velocity per tick of the robot
        
        // if(Math.abs(currentPosition.getX() - m_lastPosition.getX()) > velocityTimesJumpCount
        // || Math.abs(currentPosition.getY() - m_lastPosition.getY()) > velocityTimesJumpCount) {
        //     m_jumpCount++;
        //     if(m_jumpCount > m_jumpCountMax) {
        //         m_jumpCountMax = m_jumpCount;
        //     }
        //     if(m_jumpCount > 4) {
        //         // m_lastPosition = currentPosition;
        //     }
        // }
        // else {
        //     m_jumpCount = 0;
        // }

        // If the current position is too far from the last position, return false (could adjust these values based on robot max speed)
        if(currentPosition.getTranslation().getDistance(lastPosition.getTranslation()) > DriveTrainConstants.ROBOT_MAX_SPEED * DriveTrainConstants.DT_SECONDS) {
            return false;
        }

        //checks if robot is outside of field
        if(currentPosition.getX() < -robotMargin 
          || currentPosition.getX() > Constants.FieldLayout.FIELD_LENGTH + robotMargin
          || currentPosition.getY() < -robotMargin
          || currentPosition.getY() > Constants.FieldLayout.FIELD_WIDTH + robotMargin) {
            return false;
        }

        // checks if the angle between the robot's direction and the camera's position change is within a certain threshold
        if (norm() > 0) {
            double differenceOfPositionX = currentPosition.getX() - lastPosition.getX();
            double differenceOfPositionY = currentPosition.getY() - lastPosition.getY();
            
            Translation2d positionChange = new Translation2d(differenceOfPositionX, differenceOfPositionY);
            Translation2d robotDirection = new Translation2d(m_robotVelocity.vxMetersPerSecond, m_robotVelocity.vyMetersPerSecond);

            Translation2d robotDirectionNormalized = robotDirection.div(robotDirection.getNorm());
            Translation2d positionChangeNormalized = positionChange.div(positionChange.getNorm());

            // dot product of the robot direction and the position change (physics calculation dot product formula)
            double dotProduct = (robotDirectionNormalized.getX() * positionChangeNormalized.getX()) + (robotDirectionNormalized.getY() * positionChangeNormalized.getY());

            m_angle = Math.acos(dotProduct);

            SmartDashboard.putNumber("VisionAcceptor/angleBetweenRobotAndCamera", Math.toDegrees(m_angle));

            double allignmentThreshold = Math.toRadians(15);

            if (m_angle < allignmentThreshold) {
                return false;
            }
        }
        else {
           m_angle = 0;
        }

        // checks if robot is moving too fast for camera to update

        // TODO - this is a temporary fix, need to find a better way to determine if the robot is moving too fast for the camera to update 
        // TODO - might already be fixed and might be not needed, but keeping it here for now
        if (norm() > 2.5) { //changed from 4
            return false;
        }
        // m_lastPosition = currentPosition;
        
        return true;      
    }

    public boolean shouldResetGyro() {
        if(norm() == 0.0) {
        return true;
        }
    return false;
    }

    public double norm() {
        double dx = m_robotVelocity.vxMetersPerSecond;
        double dy = m_robotVelocity.vyMetersPerSecond;
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }
}