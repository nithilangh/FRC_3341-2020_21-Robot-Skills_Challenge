// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.profiles;

/** Add your docs here. */
public class Test_MotionProfileArc_Simple {
    public static final int NumPoints =133;
    		
	// Position (rotations)	Velocity (RPM)	Duration (ms)
	public static double [][]Points = new double[][] {		
        {0,	0	,25},
        {0.000142857142857143,	0.685714286	,25},
        {0.000642857142857143,	1.714285714	,25},
        {0.00164285714285714,	3.085714286	,25},
        {0.00328571428571429,	4.8	,25},
        {0.00571428571428571,	6.857142857	,25},
        {0.00907142857142857,	9.257142857	,25},
        {0.0135,	12	,25},
        {0.0191428571428571,	15.08571429	,25},
        {0.0261428571428571,	18.51428571	,25},
        {0.0346428571428571,	22.28571429	,25},
        {0.0447857142857143,	26.4	,25},
        {0.0567142857142857,	30.85714286	,25},
        {0.0705714285714285,	35.65714286	,25},
        {0.0865,	40.8	,25},
        {0.104642857142857,	46.28571429	,25},
        {0.125142857142857,	52.11428571	,25},
        {0.148142857142857,	58.28571429	,25},
        {0.173785714285714,	64.8	,25},
        {0.202214285714286,	71.65714286	,25},
        {0.233571428571428,	78.85714286	,25},
        {0.268,	86.4	,25},
        {0.305642857142857,	94.28571429	,25},
        {0.346642857142857,	102.5142857	,25},
        {0.391142857142857,	111.0857143	,25},
        {0.439214285714286,	119.6571429	,25},
        {0.490857142857143,	128.2285714	,25},
        {0.546071428571428,	136.8	,25},
        {0.604857142857143,	145.3714286	,25},
        {0.667071428571428,	153.2571429	,25},
        {0.7325,	160.8	,25},
        {0.801,	168	,25},
        {0.872428571428571,	174.8571429	,25},
        {0.946642857142857,	181.3714286	,25},
        {1.0235,	187.5428571	,25},
        {1.10285714285714,	193.3714286	,25},
        {1.18457142857143,	198.8571429	,25},
        {1.2685,	204	,25},
        {1.3545,	208.8	,25},
        {1.44242857142857,	213.2571429	,25},
        {1.53214285714286,	217.3714286	,25},
        {1.6235,	221.1428571	,25},
        {1.71635714285714,	224.5714286	,25},
        {1.81057142857143,	227.6571429	,25},
        {1.906,	230.4	,25},
        {2.0025,	232.8	,25},
        {2.09992857142857,	234.8571429	,25},
        {2.19814285714286,	236.5714286	,25},
        {2.297,	237.9428571	,25},
        {2.39635714285714,	238.9714286	,25},
        {2.49607142857143,	239.6571429	,25},
        {2.596,	240	,25},
        {2.696,	240	,25},
        {2.796,	240	,25},
        {2.896,	240	,25},
        {2.996,	240	,25},
        {3.096,	240	,25},
        {3.196,	240	,25},
        {3.296,	240	,25},
        {3.396,	240	,25},
        {3.496,	240	,25},
        {3.596,	240	,25},
        {3.696,	240	,25},
        {3.796,	240	,25},
        {3.896,	240	,25},
        {3.996,	240	,25},
        {4.096,	240	,25},
        {4.196,	240	,25},
        {4.296,	240	,25},
        {4.396,	240	,25},
        {4.496,	240	,25},
        {4.596,	240	,25},
        {4.696,	240	,25},
        {4.796,	240	,25},
        {4.896,	240	,25},
        {4.996,	240	,25},
        {5.096,	240	,25},
        {5.196,	240	,25},
        {5.296,	240	,25},
        {5.396,	240	,25},
        {5.496,	240	,25},
        {5.59585714285714,	239.3142857	,25},
        {5.69535714285714,	238.2857143	,25},
        {5.79435714285714,	236.9142857	,25},
        {5.89271428571428,	235.2	,25},
        {5.99028571428571,	233.1428571	,25},
        {6.08692857142857,	230.7428571	,25},
        {6.1825,	228	,25},
        {6.27685714285714,	224.9142857	,25},
        {6.36985714285714,	221.4857143	,25},
        {6.46135714285714,	217.7142857	,25},
        {6.55121428571428,	213.6	,25},
        {6.63928571428571,	209.1428571	,25},
        {6.72542857142857,	204.3428571	,25},
        {6.80949999999999,	199.2	,25},
        {6.89135714285714,	193.7142857	,25},
        {6.97085714285714,	187.8857143	,25},
        {7.04785714285714,	181.7142857	,25},
        {7.12221428571428,	175.2	,25},
        {7.19378571428571,	168.3428571	,25},
        {7.26242857142857,	161.1428571	,25},
        {7.32799999999999,	153.6	,25},
        {7.39035714285714,	145.7142857	,25},
        {7.44935714285714,	137.4857143	,25},
        {7.50485714285714,	128.9142857	,25},
        {7.55678571428571,	120.3428571	,25},
        {7.60514285714285,	111.7714286	,25},
        {7.64992857142857,	103.2	,25},
        {7.69114285714285,	94.62857143	,25},
        {7.72892857142857,	86.74285714	,25},
        {7.7635,	79.2	,25},
        {7.795,	72	,25},
        {7.82357142857142,	65.14285714	,25},
        {7.84935714285714,	58.62857143	,25},
        {7.8725,	52.45714286	,25},
        {7.89314285714285,	46.62857143	,25},
        {7.91142857142857,	41.14285714	,25},
        {7.9275,	36	,25},
        {7.9415,	31.2	,25},
        {7.95357142857142,	26.74285714	,25},
        {7.96385714285714,	22.62857143	,25},
        {7.9725,	18.85714286	,25},
        {7.97964285714285,	15.42857143	,25},
        {7.98542857142857,	12.34285714	,25},
        {7.99,	9.6	,25},
        {7.9935,	7.2	,25},
        {7.99607142857143,	5.142857143	,25},
        {7.99785714285714,	3.428571429	,25},
        {7.999,	2.057142857	,25},
        {7.99964285714285,	1.028571429	,25},
        {7.99992857142857,	0.342857143	,25},
        {8,	3.73035E-15	,25},
        {8,	0	,25}
    };
}			
