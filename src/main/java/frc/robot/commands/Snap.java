package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Snap extends Command {

    private Drivetrain _drivetrain;
    private Rotation2d _heading;

    public Snap(Drivetrain drivetrain, Rotation2d heading){
        _drivetrain = drivetrain;
        _heading = heading;
    }

    @Override
    public void execute() {
        _drivetrain.setHeadingTarget(_heading);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}