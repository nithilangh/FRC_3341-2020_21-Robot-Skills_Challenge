// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;



public class DriveTrain extends SubsystemBase {  
  /** Creates a new DriveTrain. */
  WPI_TalonSRX _leftDriveTalonMain = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonMainPort);
  WPI_TalonSRX _rightDriveTalonMain = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonMainPort);

  AHRS _ahrs = new AHRS(SPI.Port.kMXP);

  /* enable these lines for follower talons
  WPI_TalonSRX _leftDriveTalonFollower = new WPI_TalonSRX(Constants.DrivePorts.LeftDriveTalonFollowerPort);
  WPI_TalonSRX _rightDriveTalonFollower = new WPI_TalonSRX(Constants.DrivePorts.RightDriveTalonFollowerPort);
  */

  private final DifferentialDrive _diffDrive = 
      new DifferentialDrive(_leftDriveTalonMain, _rightDriveTalonMain);

    
  public DriveTrain() {
    _leftDriveTalonMain.configFactoryDefault();
    _rightDriveTalonMain.configFactoryDefault();
    
    _leftDriveTalonMain.set(ControlMode.PercentOutput, 0);
    _rightDriveTalonMain.set(ControlMode.PercentOutput, 0);

    _leftDriveTalonMain.setNeutralMode(NeutralMode.Brake);
    _rightDriveTalonMain.setNeutralMode(NeutralMode.Brake);

    
    _leftDriveTalonMain.setInverted(false);
    _rightDriveTalonMain.setInverted(true);

    _leftDriveTalonMain.setSensorPhase(true);
    _rightDriveTalonMain.setSensorPhase(true);

    /* Enable these two lines later during testing and see if there is any change in performance
    _leftDriveTalonMain.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
    _rightDriveTalonMain.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
     */
    _leftDriveTalonMain.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _rightDriveTalonMain.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);


    /* enable these lines for follower talons
    _leftDriveTalonFollower.configFactoryDefault();
    _rightDriveTalonFollower.configFactoryDefault();

    _leftDriveTalonFollower.follow(_leftDriveTalonMain);
    _rightDriveTalonFollower.follow(_rightDriveTalonMain);

    _leftDriveTalonFollower.setInverted(InvertType.FollowMaster);
    _rightDriveTalonFollower.setInverted(InvertType.FollowMaster);
    */
    
    
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

  public Rotation2d getRotation2d() {
    return _ahrs.getRotation2d();
  }

  public double getLeftDistance() {
    return  (_leftDriveTalonMain.getSelectedSensorPosition() * Constants.Chassis.Wheelcircumference_M
    / Constants.Encoder.PulsesPerRev);
  }

  public double getRightDistance() {
    return (_rightDriveTalonMain.getSelectedSensorPosition() * Constants.Chassis.Wheelcircumference_M
           / Constants.Encoder.PulsesPerRev);
  }


  public void stop() {
    _diffDrive.stopMotor();
  }
}
