package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IncreaseHood extends Command {

    private Shooter _shooter;

    public IncreaseHood(Shooter shooter){
        _shooter = shooter;
    }

    @Override
    public void execute() {
        double angle = _shooter.getDesiredAngle();
        _shooter.setHoodAngleRotations(angle + 0.05);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}