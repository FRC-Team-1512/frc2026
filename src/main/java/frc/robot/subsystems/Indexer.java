package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;

public class Indexer extends SubsystemBase {

    private final TalonFX _indexerLeftMotor;
    private final TalonFX _indexerRightMotor;
    private final Follower _indexerFollower;

    private double _motorPower;

    private final DoublePublisher _motorPowerPublisher;
    private final DoublePublisher _leftCurrentPublisher;
    private final DoublePublisher _rightCurrentPublisher;

    public Indexer() {
        _indexerRightMotor = new TalonFX(RobotMap.CAN.RIGHT_INDEXER);
        _indexerLeftMotor = new TalonFX(RobotMap.CAN.LEFT_INDEXER);

        TalonFXConfiguration _indexerMotorConfig = new TalonFXConfiguration();

        _indexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _indexerMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Indexer.MotorConfig.INDEXER_STATOR_CURRENT_LIMIT;
        _indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.MotorConfig.INDEXER_SUPPLY_CURRENT_LIMIT;
        _indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _indexerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        StatusCode rightIndexerStatus = ApplyConfig.applyConfigWithRetry("Indexer Right", getName(), () -> _indexerRightMotor.getConfigurator().apply(_indexerMotorConfig));
        if (!rightIndexerStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for right indexer motor: " + rightIndexerStatus, false);
        }
        StatusCode leftIndexerStatus = ApplyConfig.applyConfigWithRetry("Indexer Left", getName(), () -> _indexerLeftMotor.getConfigurator().apply(_indexerMotorConfig));
        if (!leftIndexerStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for left indexer motor: " + leftIndexerStatus, false);
        }

        _motorPower = 0.0;
        _indexerFollower = new Follower(RobotMap.CAN.LEFT_INDEXER, MotorAlignmentValue.Opposed);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Indexer");
        _motorPowerPublisher = table.getDoubleTopic("MotorPower").publish();
        _leftCurrentPublisher = table.getDoubleTopic("LeftCurrent").publish();
        _rightCurrentPublisher = table.getDoubleTopic("RightCurrent").publish();
    }
    
    @Override
    public void periodic() {
        updateState();

        _motorPowerPublisher.set(_motorPower);
        _leftCurrentPublisher.set(_indexerLeftMotor.getStatorCurrent().getValueAsDouble());
        _rightCurrentPublisher.set(_indexerRightMotor.getStatorCurrent().getValueAsDouble());
    }

    public void setIndexer(double power) {
        _motorPower = power;
    }

    private void updateState() {
        _indexerLeftMotor.set(_motorPower);
        _indexerRightMotor.setControl(_indexerFollower);
    }
}
