package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vector2d {
    private final Translation2d t;

    public Vector2d() {
        this.t = new Translation2d();
    }

    public Vector2d(double x, double y) {
        this.t = new Translation2d(x, y);
    }

    public Vector2d(Translation2d t) {
        this.t = t;
    }

    public double x() { return t.getX(); }
    public double y() { return t.getY(); }

    public double norm() { return t.getNorm(); }
    public Rotation2d angle() { return t.getAngle(); }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(t.plus(other.t));
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(t.minus(other.t));
    }

    public Vector2d times(double scalar) {
        return new Vector2d(t.times(scalar));
    }

    public Vector2d rotateBy(Rotation2d r) {
        return new Vector2d(t.rotateBy(r));
    }

    public Translation2d toTranslation2d() {
        return t;
    }

    public double dot(Vector2d other) {
        return x() * other.x() + y() * other.y();
    }
    
    public double cross(Vector2d other) {
        return x() * other.y() - y() * other.x();
    }
    
    public Vector2d normalized() {
        double n = norm();
        return (n < 1e-9) ? new Vector2d(0,0) : new Vector2d(x()/n, y()/n);
    }

    @Override
    public String toString() {
        return "Vector2d(" + x() + ", " + y() + ")";
    }
}
