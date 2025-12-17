package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SetIMU extends Command {
    Drivetrain _drivetrain;
    Rotation2d _rot;

    public SetIMU(Drivetrain drivetrain, Rotation2d rot) {
        _drivetrain = drivetrain;
        _rot = rot;
    }

    @Override
    public void execute() {
        _drivetrain.setIMU(_rot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
