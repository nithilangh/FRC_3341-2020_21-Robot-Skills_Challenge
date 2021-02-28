// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.profiles.Test_MotionProfileArc_Simple;
import frc.robot.subsystems.DriveTrain;

public class TestDrive01 extends CommandBase {
  /** Creates a new TestDrive01. */

  private final DriveTrain _driveTrain;
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;
  private boolean[] _jsButtons = new boolean[Constants.Joystick.NumButtonsPlusOne];
  private boolean[] _jsButtonsOldState = new boolean[Constants.Joystick.NumButtonsPlusOne];

  private BufferedTrajectoryPointStream _bufferedStream = new BufferedTrajectoryPointStream();

  public TestDrive01(DriveTrain dt, Joystick lj, Joystick rj) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _leftJoystick = lj;
    _rightJoystick = rj;
    
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initBufferStream(Test_MotionProfileArc_Simple.Points, 
                    Test_MotionProfileArc_Simple.NumPoints, 
                    90.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.tankDrive(-1 * _leftJoystick.getRawAxis(Constants.Joystick.JoystickAxis.YAxis),
                           -1 *_rightJoystick.getRawAxis(Constants.Joystick.JoystickAxis.YAxis));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  private void initBufferStream(double[][] profile, int totalCount, double finalTurnDeg) {
    boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
    // since you can use negative numbers in profile).

    TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                              // automatically, you can alloc just one

    /* clear the buffer, in case it was used elsewhere */
    _bufferedStream.Clear();

    /* Insert every point into buffer, no limit on size */
    for (int i = 0; i < totalCount; ++i) {

      double direction = forward ? +1 : -1;
      /* use the generated profile to figure out the forward arc path (translation)*/
      double positionRot = profile[i][0];
      double velocityRPM = profile[i][1];
      int durationMilliseconds = (int) profile[i][2];

      /* to get the turn target, lets just scale from 0 deg to caller's final deg linearizly */
      double targetTurnDeg = finalTurnDeg * (i + 1) / totalCount;

      /* for each point, fill our structure and pass it to API */
      point.timeDur = durationMilliseconds;

      /* drive part */
      point.position = direction * positionRot * Constants.Encoder.MagEncoderUnitsPerRotation; // Rotations => sensor units
      point.velocity = direction * velocityRPM * Constants.Encoder.MagEncoderUnitsPerRotation / 600.0; // RPM => units per 100ms
      point.arbFeedFwd = 0; // good place for kS, kV, kA, etc...

      /* turn part */
      // TODO:
      point.auxiliaryPos = targetTurnDeg * Constants.Chassis.EncoderUnitsPerDegree; // Convert deg to remote sensor units
      point.auxiliaryVel = 0; // advanced teams can also provide the target velocity
      point.auxiliaryArbFeedFwd = 0; // good place for kS, kV, kA, etc...

      point.profileSlotSelect0 = Constants.PIDSlots.Distance; /* which set of gains would you like to use [0,3]? */
      point.profileSlotSelect1 = Constants.PIDSlots.Turn; /* auxiliary PID [0,1], leave zero */
      point.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
      point.isLastPoint = ((i + 1) == totalCount); /* set this to true on the last point */
      point.useAuxPID = true; /* tell MPB that we are using both pids */

      _bufferedStream.Write(point);
    }

  }

}
