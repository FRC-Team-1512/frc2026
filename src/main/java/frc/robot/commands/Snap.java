package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Snap extends Command {

    private Drivetrain _drivetrain;
    

    public Snap(Drivetrain drivetrain, double heading){

        _drivetrain = drivetrain;



    }

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        _drivetrain.setHeadingTargetDegrees(0);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    
    
}