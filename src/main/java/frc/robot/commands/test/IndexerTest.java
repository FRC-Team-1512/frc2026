package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class IndexerTest extends Command {

    private Indexer _indexer;

    public IndexerTest(Indexer indexer){
        _indexer = indexer;
        addRequirements(_indexer);
    }

    @Override
    public void execute() {
        double v = -applyDeadband(RobotContainer.operator.getRightY(), 0.15);
        _indexer.setIndexer(v * 1.0);
    }

    private static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }else {
            return input;
        }
    }
}