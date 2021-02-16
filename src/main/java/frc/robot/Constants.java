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

    public static final class PIDIndex {
        public static final int PIDPrimary = 0;
        public static final int PIDAux1 = 1;
    }


}
