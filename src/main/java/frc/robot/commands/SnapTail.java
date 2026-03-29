package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SnapTail extends Command {

    private Drivetrain _drivetrain;

    public SnapTail(Drivetrain drivetrain){
        _drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();
        Rotation2d target = Rotation2d.fromDegrees(180);
        if (_isRedAlliance) {
            target = Rotation2d.fromDegrees(0);
        }
        _drivetrain.setHeadingTarget(target);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}