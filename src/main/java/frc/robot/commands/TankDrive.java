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

  private double leftVal;
  private double rightVal;


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

    leftVal = (-1 * m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickYAxis)) 
              + m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickXAxis);

    rightVal = (-1 * m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickYAxis)) 
               - m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickXAxis);

    if(leftVal > 1) leftVal = 1;
    if(leftVal < -1) leftVal = -1;
    if(rightVal > 1) rightVal = 1;
    if(rightVal < -1) rightVal = -1;

    m_driveTrain.tankDrive(leftVal, rightVal);

    /* This line is expecting two joystick inputs.
       It is replaced with the single joystick drive.
    m_driveTrain.tankDrive(-m_joystick.getRawAxis(Constants.JoystickAxis.LeftStickYAxis),
                          -m_joystick.getRawAxis(Constants.JoystickAxis.RightStickYAxis));
     */
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
