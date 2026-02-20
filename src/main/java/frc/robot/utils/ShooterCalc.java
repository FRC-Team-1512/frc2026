package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.InterpolatingTreeMap;

public class ShooterCalc {

    //Distance : Meters (from robot center to target center, horizontal distance)
    //RPS : Rotations Per Second of motor rotor

    // LocalHoodAngle : Rotations of the hood motor rotor

    private static double[][] _kRPSValuesNear = {
        // {meters, RPS} make a lot of values. 
    };
    private static double[][] _kRPSValuesFar = {
        // {meters, RPS} make a lot of values.   
    };
    private static double[][] _kRPSValuesMiddle = {
        // {meters, RPS} make a lot of values. 
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
    
    public static double getRPSFromDistance(double distance){
        if (distance >= Constants.Shooter.FAR_DISTANCE){
            return _kRPSMapFar.getInterpolated(new InterpolatingDouble(distance)).value;
        } else if (Constants.Shooter.NEAR_DISTANCE < distance && distance < Constants.Shooter.FAR_DISTANCE){
            return _kRPSMapMiddle.getInterpolated(new InterpolatingDouble(distance)).value;
        } else {
            return _kRPSMapNear.getInterpolated(new InterpolatingDouble(distance)).value;
        }
    }

    public static Rotation2d getLocalHoodAngleFromDistance(double distance){
         if (distance >= Constants.Shooter.FAR_DISTANCE){
            return Constants.Shooter.HOOD_FAR;
        } else if (Constants.Shooter.NEAR_DISTANCE < distance && distance < Constants.Shooter.FAR_DISTANCE){
            return Constants.Shooter.HOOD_MID;
        } else {
            return Constants.Shooter.HOOD_NEAR;
        }
    }
}
