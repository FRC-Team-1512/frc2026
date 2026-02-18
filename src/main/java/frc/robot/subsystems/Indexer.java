package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;

public class Indexer extends SubsystemBase {

    private final TalonFX _indexerLeftMotor;
    private final TalonFX _indexerRightMotor;

    private double _motorPower;

    public Indexer() {
        _indexerRightMotor = new TalonFX(RobotMap.CAN.RIGHT_INDEXER);
        _indexerLeftMotor = new TalonFX(RobotMap.CAN.LEFT_INDEXER);

        TalonFXConfiguration _indexerMotorConfig = new TalonFXConfiguration();

        _indexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //_indexerMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Indexer.MotorConfig.INDEXER_STATOR_CURRENT_LIMIT;
        _indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //_indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.MotorConfig.INDEXER_SUPPLY_CURRENT_LIMIT;
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
    }
    
    @Override
    public void periodic() {
        updateState();
    }

    public void setIndexer(double power) {
        _motorPower = power;
    }

    private void updateState() {
        _indexerLeftMotor.set(_motorPower);
        _indexerRightMotor.setControl(new Follower(RobotMap.CAN.LEFT_INDEXER, MotorAlignmentValue.Opposed));
    }
}
