// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Chassis {
        public static final double WheelDiameter_In = 6;
        public static final double WheelCircumference_In = WheelDiameter_In * Math.PI;
        public static final double Wheelcircumference_M = Units.inchesToMeters(WheelCircumference_In);
        public static final double TrackWidth_In = 23.0;
        public static final double TrackWidth_M = Units.inchesToMeters(TrackWidth_In);
        public static final double MaxVelocity_MPS = 1.0;
        public static final double MaxAcceleration_MPSS = 1.0;
    }

    public static final class Encoder {
        public static final double PulsesPerRev = 4096; //Pulses Per Revolution
    }

    public static final class Ramsete {
        public static final double B = 2.0;
        public static final double Zeta = 0.7;
    }

	public static final class DriveTrainPorts {
        public static int LeftDriveTalonMainPort = 1;
        public static int LeftDriveTalonFollowerPort = 2;
        public static int RightDriveTalonMainPort = 11;
        public static int RightDriveTalonFollowerPort = 12;
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

    public static final class JoystickAxis {
        public static final int XAxis = 0;
        public static final int YAxis = 1;
        public static final int ZRotate = 2;
        public static final int Slider = 3;
    }

    public static final class FeedforwardCoeff {
        public static final class Voltage {
            public static final double StaticFriction = 0.662;
            public static final double CruiseVelocity_SPM = 3.15;   // Seconds Per Meter
            public static final double Acceleration_SSPM = 0.609;   // Seconds Square Per Meter
        }     
    }

    public static final class FeedbackCoeff {
        public static final class PID {
            public static final double P = 0.11;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
    }

    public static final class CTRE_PIDIndex {
        public static final int PIDPrimary = 0;
        public static final int PIDAux1 = 1;
    }


}
