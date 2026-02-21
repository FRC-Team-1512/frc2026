package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
    Drivetrain _drivetrain;

    public Drive(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
        addRequirements(_drivetrain);
    }
    
    @Override
    public void execute() {
        boolean isSlowMode = RobotContainer.driver.leftBumper().getAsBoolean();

        double vx = -applyDeadband(RobotContainer.driver.getLeftY(), 0.15);
        double vy = -applyDeadband(RobotContainer.driver.getLeftX(), 0.15);

        double rot = -applyDeadband(RobotContainer.driver.getRightX(), 0.15);

        if(!isSlowMode) {
            double speed = 8.0;
            vx *= speed;
            vy *= speed;
            rot *= speed;
        }

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("rot", rot);

        _drivetrain.setFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, rot));
    }

    private static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }else {
            return input;
        }
    }
}
