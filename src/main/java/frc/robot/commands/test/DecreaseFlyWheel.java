package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class DecreaseFlyWheel extends Command {

    private Shooter _shooter;

    public DecreaseFlyWheel(Shooter shooter){
        _shooter = shooter;
    }

    @Override
    public void execute() {
        _shooter.getDesiredVelocity();
        _shooter.setShooterVelocity(_shooter.getDesiredVelocity() - 5.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}