package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HoodLow extends Command {

    private Shooter _shooter;

    public HoodLow(Shooter shooter){
        _shooter = shooter;
        addRequirements(_shooter);
    }

    @Override
    public void execute() {
        _shooter.setHoodAngleRotations(0.4);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
