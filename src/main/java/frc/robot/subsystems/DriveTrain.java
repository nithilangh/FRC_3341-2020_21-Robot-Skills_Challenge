// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  WPI_TalonSRX _leftDriveTalonMain = new WPI_TalonSRX(Constants.DrivePorts.leftDriveTalonMainPort);
  WPI_TalonSRX _leftDriveTalonFollower = new WPI_TalonSRX(Constants.DrivePorts.leftDriveTalonFollowerPort);
  WPI_TalonSRX _rightDriveTalonMain = new WPI_TalonSRX(Constants.DrivePorts.rightDriveTalonMainPort);
  WPI_TalonSRX _rightDriveTalonFollower = new WPI_TalonSRX(Constants.DrivePorts.rightDriveTalonFollowerPort);

  private final DifferentialDrive _diffDrive = 
      new DifferentialDrive(_leftDriveTalonMain, _rightDriveTalonMain);
    
  

  public DriveTrain() {
    _leftDriveTalonMain.configFactoryDefault();
    _leftDriveTalonFollower.configFactoryDefault();
    _rightDriveTalonMain.configFactoryDefault();
    _rightDriveTalonFollower.configFactoryDefault();

    _leftDriveTalonFollower.follow(_leftDriveTalonMain);
    _rightDriveTalonFollower.follow(_rightDriveTalonMain);

    _leftDriveTalonMain.setInverted(false);
    _rightDriveTalonMain.setInverted(true);
    _leftDriveTalonFollower.setInverted(InvertType.FollowMaster);
    _rightDriveTalonFollower.setInverted(InvertType.FollowMaster);

    _leftDriveTalonMain.setSensorPhase(true);
    _rightDriveTalonMain.setSensorPhase(true);
    
    _diffDrive.setRightSideInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /* public static void initTalon(WPI_TalonSRX talon) {
    talon.setNeutralMode(NeutralMode.Coast);
    talon.neutralOutput();
    talon.setSensorPhase(false);
    talon.config
  } */

  
  public void arcadeDrive(double speed, double turn) {
    _diffDrive.arcadeDrive(speed, turn);
  }


  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }


  public void stop() {
    _diffDrive.stopMotor();
  }
}
