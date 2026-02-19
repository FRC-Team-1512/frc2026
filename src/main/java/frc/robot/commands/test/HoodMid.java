package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HoodMid extends Command {

    private Shooter _shooter;

    public HoodMid(Shooter shooter){
        _shooter = shooter;
        addRequirements(_shooter);
    }

    @Override
    public void execute() {
        _shooter.setHoodAngle(Rotation2d.fromRotations(-1.1));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
