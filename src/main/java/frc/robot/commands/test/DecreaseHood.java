package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class DecreaseHood extends Command {

    private Shooter _shooter;

    public DecreaseHood(Shooter shooter){
        _shooter = shooter;
    }

    @Override
    public void execute() {
        double angle = _shooter.getDesiredAngle();
        _shooter.setHoodAngle(Rotation2d.fromRotations(angle - 0.05));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}