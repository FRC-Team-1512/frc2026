package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class IncreaseFlyWheel extends Command {

    private Shooter _shooter;

    public IncreaseFlyWheel(Shooter shooter){
        _shooter = shooter;
    }

    @Override
    public void execute() {
        _shooter.getDesiredVelocity();
        _shooter.setShooterVelocity(_shooter.getDesiredVelocity() + 5.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}