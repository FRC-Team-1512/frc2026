package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;

public class SingleSwerveTest extends Command {
    SwerveModule _module;

    public SingleSwerveTest(SwerveModule module) {
        _module = module;
        addRequirements(_module);
    }
    
    @Override
    public void execute() {
        double vy = RobotContainer.driver.getLeftY();
        double vx = RobotContainer.driver.getLeftX();

        double speed = Math.sqrt(vx * vx + vy * vy) * 3.0; // Scale to max speed
        double angle = Math.atan2(vy, vx); // Radians

        _module.setState(speed, new Rotation2d(angle));
    }
}
