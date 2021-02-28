// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;



public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

 // TalonSRXConfiguration

  WPI_TalonSRX _leftDriveTalonMain = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonMainPort);
  WPI_TalonSRX _rightDriveTalonMain = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonMainPort);

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

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


    _leftDriveTalonMain.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                                      Constants.PIDIndex.PIDPrimary,
                                                      Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configRemoteFeedbackFilter(_leftDriveTalonMain.getDeviceID(),
                                                    RemoteSensorSource.TalonSRX_SelectedSensor,
                                                    Constants.Ordinals.OrdinalOne,
                                                    Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSensorTerm(SensorTerm.Sum0,
                                          FeedbackDevice.RemoteSensor1,
                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSensorTerm(SensorTerm.Sum1,
                                          FeedbackDevice.CTRE_MagEncoder_Relative,
                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSensorTerm(SensorTerm.Diff1,
                                          FeedbackDevice.RemoteSensor1,
                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSensorTerm(SensorTerm.Diff0,
                                          FeedbackDevice.CTRE_MagEncoder_Relative,
                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSelectedFeedbackSensor(FeedbackDevice.SensorSum,
                                                      Constants.PIDIndex.PIDPrimary,
                                                      Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSelectedFeedbackCoefficient(0.5,
                                                          Constants.PIDIndex.PIDPrimary,
                                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,
                                                      Constants.PIDIndex.PIDAux1,
                                                      Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configSelectedFeedbackCoefficient(1,
                                                          Constants.PIDIndex.PIDAux1,
                                                          Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,
                                              20,
                                              Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
                                              20,
                                              Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,
                                              20,
                                              Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,
                                              5,
                                              Constants.Time.SensorConfigTimeout);

    _leftDriveTalonMain.configNeutralDeadband(Constants.Joystick.Deadband,
                                                Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configNeutralDeadband(Constants.Joystick.Deadband,
                                                Constants.Time.SensorConfigTimeout);

    _leftDriveTalonMain.configPeakOutputForward(+1.0, Constants.Time.SensorConfigTimeout);
    _leftDriveTalonMain.configPeakOutputReverse(-1.0, Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.configPeakOutputForward(+1.0, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.configPeakOutputReverse(-1.0, Constants.Time.SensorConfigTimeout);

    _rightDriveTalonMain.config_kP(Constants.PIDSlots.Distance, Constants.DrivePID.Proportional, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kI(Constants.PIDSlots.Distance, Constants.DrivePID.Integral, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kD(Constants.PIDSlots.Distance, Constants.DrivePID.Derivative, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kF(Constants.PIDSlots.Distance, Constants.DrivePID.FeedForward, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_IntegralZone(Constants.PIDSlots.Distance, Constants.DrivePID.IntegralZone, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.configClosedLoopPeakOutput(Constants.PIDSlots.Distance, Constants.DrivePID.PeakOutput, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.configAllowableClosedloopError(Constants.PIDSlots.Distance, 0, Constants.Time.SensorConfigTimeout);

    
    _rightDriveTalonMain.config_kP(Constants.PIDSlots.Turn, Constants.DrivePID.Proportional, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kI(Constants.PIDSlots.Turn, Constants.DrivePID.Integral, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kD(Constants.PIDSlots.Turn, Constants.DrivePID.Derivative, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_kF(Constants.PIDSlots.Turn, Constants.DrivePID.FeedForward, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.config_IntegralZone(Constants.PIDSlots.Turn, Constants.DrivePID.IntegralZone, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.configClosedLoopPeakOutput(Constants.PIDSlots.Turn, Constants.DrivePID.PeakOutput, Constants.Time.SensorConfigTimeout);
    _rightDriveTalonMain.configAllowableClosedloopError(Constants.PIDSlots.Turn, 0, Constants.Time.SensorConfigTimeout);

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

  
  public void resetDriveTrain() {
    
  }

  
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
