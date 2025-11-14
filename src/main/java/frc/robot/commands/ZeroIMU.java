package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ZeroIMU extends Command {
    Drivetrain _drivetrain;

    public ZeroIMU(Drivetrain drivetrain) {
        _drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        _drivetrain.zeroIMU();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
