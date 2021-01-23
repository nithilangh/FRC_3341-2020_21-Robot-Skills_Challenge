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

  private final DriveTrain m_driveTrain;
  private final Joystick m_joystick;


  public TankDrive(DriveTrain dt, Joystick js) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    m_joystick = js;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.tankDrive(-m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickYAxis),
                          -m_joystick.getRawAxis(Constants.JoystickAxis.RightStickYAxis));
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
