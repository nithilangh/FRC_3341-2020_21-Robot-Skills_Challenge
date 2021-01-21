// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final SpeedController m_leftMotorSpeedController = 
      new SpeedControllerGroup(
            new PWMTalonSRX(Constants.DrivePorts.leftFrontTalonPort), 
            new PWMTalonSRX(Constants.DrivePorts.leftBackTalonPort)
      );

  private final SpeedController m_rightMotorSpeedController = 
      new SpeedControllerGroup(
        new PWMTalonSRX(Constants.DrivePorts.rightFrontTalonPort), 
        new PWMTalonSRX(Constants.DrivePorts.rightBackTalonPort)
      );

  private final DifferentialDrive m_diffDrive = 
      new DifferentialDrive(m_leftMotorSpeedController, m_rightMotorSpeedController);
      
  

  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void arcadeDrive(double speed, double turn) {
    m_diffDrive.arcadeDrive(speed, turn);
  }


  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }


  public void stop() {
    m_diffDrive.stopMotor();
  }
}
