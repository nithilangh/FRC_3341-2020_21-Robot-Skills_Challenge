// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class DriveTrainPorts {
        public static int LeftDriveTalonMainPort = 0;
        public static int LeftDriveTalonFollowerPort = 1;
        public static int RightDriveTalonMainPort = 2;
        public static int RightDriveTalonFollowerPort = 3;
    }

    public static final class Time{
        public static final int SensorConfigTimeout = 30;
    }

    public static final class Ordinals {
        public static final int OrdinalZero = 0;
        public static final int OrdinalOne = 1;
    }

    public static final class DriverStation {

        public static final class USBOrder {
            public static final int Zero = 0;
            public static final int One = 1;
            public static final int Two = 2;
            public static final int Three = 3;
            public static final int Four = 4;
            public static final int Five = 5;
        }
 
    }

    public static final class Joystick {

        public static final class JoystickAxis {
            public static final int XAxis = 0;
            public static final int YAxis = 1;
            public static final int ZRotate = 2;
            public static final int Slider = 3;
        }

        public static final int NumButtonsPlusOne = 7;
        public static final double Deadband = 0.05;
    }

    public static final class Sensor {

    }

    public static final class Encoder {
        public static final int MagEncoderUnitsPerRotation = 4096;
    }

    public static final class DrivePID{
        public static final double FeedForward = 1.0;
        public static final double Proportional = 1.0;
        public static final double Integral  = 1.0;
        public static final double Derivative = 1.0;
        public static final double IntegralZone = 1.0;
        public static final double PeakOutput = 1.0;
    }

    public static final class TurnPID{
        public static final double FeedForward = 1.0;
        public static final double Proportional = 1.0;
        public static final double Integral  = 1.0;
        public static final double Derivative = 1.0;
        public static final double IntegralZone = 1.0;
        public static final double PeakOutput = 1.0;
    }

    public static final class PIDIndex {
        public static final int PIDPrimary = 0;
        public static final int PIDAux1 = 1;
    }

    public static final class PIDSlots {
        public static final int Distance = 0;
        public static final int Turn = 1;
        public static final int Velocity = 2;
        public static final int MotionProfile = 3;
    }

    public static final class Chassis {
        public static final double ChassisWidth = 23.5;
        public static final double RobotRotationCircumference = ChassisWidth * Math.PI;

        public static final double WheelDiameter = 6;
        public static final double WheelCircumference = WheelDiameter * Math.PI;

        public static final double RobotCircumferenceTicks = (RobotRotationCircumference / WheelCircumference) * Encoder.MagEncoderUnitsPerRotation;
        public static final double EncoderUnitsPerDegree = RobotCircumferenceTicks / 360;
    }


}
