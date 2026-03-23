package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class HoodHigh extends Command {

    private Shooter _shooter;

    public HoodHigh(Shooter shooter){
        _shooter = shooter;
        addRequirements(_shooter);
    }

    @Override
    public void execute() {
        _shooter.setHoodAngleRotations(-2.2);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
