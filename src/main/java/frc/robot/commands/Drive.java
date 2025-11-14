package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        double vx = -applyDeadband(RobotContainer.driver.getLeftY(), 0.05);
        double vy = -applyDeadband(RobotContainer.driver.getLeftX(), 0.05);

        double rot = -applyDeadband(RobotContainer.driver.getRightX(), 0.05);

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
