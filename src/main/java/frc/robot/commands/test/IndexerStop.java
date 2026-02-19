package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class IndexerStop extends Command {

    private Indexer _indexer;

    public IndexerStop(Indexer indexer){
        _indexer = indexer;
        addRequirements(_indexer);
    }

    @Override
    public void execute() {
        _indexer.setIndexer(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}