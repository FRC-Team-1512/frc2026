package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.Constants;
//import frc.robot.lib.InterpolatingDouble;
//import frc.robot.lib.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterCalc {

    //Distance : Meters (from robot center to target center, horizontal distance)
    //RPS : Rotations Per Second of motor rotor

    // LocalHoodAngle : Rotations of the hood motor rotor

    //===============================================================================================

    static final double V_COEFF = 1.74; // Coefficient to adjust the calculated velocity for real-world conditions (e.g., air resistance, friction)
    static final double H_MAX_FROM_GROUND = 3.0;

    //===============================================================================================

    static final double hopper_height = 1.8288; //meters
    static final double robot_height = 0.505; //meters
    static final double robot_half_width = 0.27305; //meters

    static final double H = hopper_height - robot_height; // Height difference between target and shooter (meters)
    static final double H_MAX = H_MAX_FROM_GROUND - robot_height; // Height at the peak of the trajectory (meters)
    static final double G = 9.81; // Gravitatinal acceleration (m/s^2)

    static final double ALPHA = H_MAX + Math.sqrt(H_MAX * (H_MAX - H)); // Auxiliary variable for calculations

    public static final double T_ETA = Math.sqrt(2.0 / G) * (Math.sqrt(H_MAX) + Math.sqrt(H_MAX - H)); // Time of flight (seconds)

    static public Rotation2d calculateGlobalHoodAngle (double distance) {
        distance -= robot_half_width;
        Rotation2d universalAngle = Rotation2d.fromRadians(Math.atan((2 * ALPHA) / distance));
        return Rotation2d.fromDegrees(90).minus(universalAngle);
    }

    static public double calculateRotorHoodAngleRotation(double distance) {
        Rotation2d globalHoodAngle = calculateGlobalHoodAngle(distance);
        return getRotorHoodAngleRotation(globalHoodAngle);
    }

    static public double calculateGlobalRPS(double distance) {
        distance -= robot_half_width;
        double base = Math.sqrt(distance * distance + 4 * ALPHA * ALPHA) / (2.0 * ALPHA);
        double vel = base * Math.sqrt(2.0 * G * H_MAX) * V_COEFF;
        SmartDashboard.putNumber("Shooter: calc glob vel", vel);
        return vel;
    }

    static public double calculateRotorRPS(double distance) {
        double globalRPS = calculateGlobalRPS(distance);
        return getRotorRPS(globalRPS);
    }

    static public double getRotorRPS(double mechanismRPS) {
        return ((mechanismRPS * 25.0) / 24.0) / (0.319);
    }

    static public double getRotorHoodAngleRotation(Rotation2d globalHoodAngle) {
        Rotation2d X = globalHoodAngle.minus(Rotation2d.fromDegrees(3.0));
        return 0.34 - X.getRotations() * 25.4545;
    }

    /* 
    ===============================================================
    Legacy interpolation

    public static final Rotation2d HOOD_FAR = Rotation2d.fromRotations(-1.95);
    public static final Rotation2d HOOD_MID = Rotation2d.fromRotations(-0.7);
    public static final Rotation2d HOOD_NEAR = Rotation2d.fromRotations(-0.35);

    private static double[][] _kRPSValuesNear = {
        // {meters, RPS} make a lot of values. 
        {1.3589, 45},
        {1.7526, 45},
    };
    private static double[][] _kRPSValuesFar = {
        // {meters, RPS} make a lot of values.   
        {2.1336, 45},
        {2.54, 50.25},
        {3.2258, 56},
    };
    private static double[][] _kRPSValuesMiddle = {
        // {meters, RPS} make a lot of values. 
        {3.7846, 50},
        {4.2926, 54},
        {4.8768, 57.5},
    };

    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _kRPSMapNear = new InterpolatingTreeMap<>();
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _kRPSMapFar = new InterpolatingTreeMap<>();
    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _kRPSMapMiddle = new InterpolatingTreeMap<>();

    static {
        for (double[] pair : _kRPSValuesNear) {
            _kRPSMapNear.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        for (double[] pair : _kRPSValuesFar) {
            _kRPSMapFar.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        for (double[] pair : _kRPSValuesMiddle) {
            _kRPSMapMiddle.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }
    
    public static double getRPSFromDistance(double distance) {
        if (distance >= Constants.Shooter.FAR_DISTANCE) {
            return _kRPSMapFar.isEmpty() ? 0.0 : _kRPSMapFar.getInterpolated(new InterpolatingDouble(distance)).value;
        } else if (Constants.Shooter.NEAR_DISTANCE < distance && distance < Constants.Shooter.FAR_DISTANCE) {
            return _kRPSMapMiddle.isEmpty() ? 0.0 : _kRPSMapMiddle.getInterpolated(new InterpolatingDouble(distance)).value;
        } else {
            return _kRPSMapNear.isEmpty() ? 0.0 : _kRPSMapNear.getInterpolated(new InterpolatingDouble(distance)).value;
        }
    }

    public static Rotation2d getLocalHoodAngleFromDistance(double distance){
         if (distance >= Constants.Shooter.FAR_DISTANCE){
            return HOOD_FAR;
        } else if (Constants.Shooter.NEAR_DISTANCE < distance && distance < Constants.Shooter.FAR_DISTANCE){
            return HOOD_MID;
        } else {
            return HOOD_NEAR;
        }
    }

    */
}
