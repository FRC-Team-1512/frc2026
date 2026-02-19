package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTest extends Command {

    private Shooter _shooter;

    public ShooterTest(Shooter shooter){
        _shooter = shooter;
        addRequirements(_shooter);
    }

    @Override
    public void execute() {
        double v = -applyDeadband(RobotContainer.operator.getLeftY(), 0.15);
        //_shooter.setShooterVelocity(v * 50.0);

        double angle = -applyDeadband(RobotContainer.operator.getRightY(), 0.15);
        angle *= 1.35;
        angle -= 0.95;
        //_shooter.setHoodAngle(Rotation2d.fromRotations(angle));
    }

    private static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }else {
            return input;
        }
    }
}