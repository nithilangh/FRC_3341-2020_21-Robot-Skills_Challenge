// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class MotionProfile_WPILib extends CommandBase {
  /** Creates a new FRC2021_MotionProfile_WPILib. */
  private final DriveTrain _driveTrain;
  private final DifferentialDriveKinematics _diffDriveKinematics;
  private final DifferentialDriveOdometry _diffDriveOdometry;


  public MotionProfile_WPILib(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _diffDriveKinematics =
        new DifferentialDriveKinematics(Constants.Chassis.TrackWidth_M);
    _diffDriveOdometry = 
        new DifferentialDriveOdometry(_driveTrain.getRotation2d());

    addRequirements(_driveTrain);

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _diffDriveOdometry.update(_driveTrain.getRotation2d(),
        _driveTrain.getLeftDistance(),
        _driveTrain.getRightDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
