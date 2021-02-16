// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */

  private final DriveTrain _driveTrain;
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;


  public TankDrive(DriveTrain dt, Joystick lj, Joystick rj) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _leftJoystick = lj;
    _rightJoystick = rj;

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    _driveTrain.tankDrive(-1 * _leftJoystick.getRawAxis(Constants.JoystickAxis.YAxis),
                          -1 * _rightJoystick.getRawAxis(Constants.JoystickAxis.YAxis));
  
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
