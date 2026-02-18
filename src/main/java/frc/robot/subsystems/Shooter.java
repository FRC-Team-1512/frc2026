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

public class Shooter extends SubsystemBase {

    private final TalonFX _shooterRightMotor;
    private final TalonFX _shooterLeftMotor;
    private final TalonFX _hoodMotor;

    private final VelocityVoltage _shooterVelocityVoltage;

    private double _targetVelocity;
    private Rotation2d _targetHoodAngle;

    public Shooter() {
        _shooterRightMotor = new TalonFX(RobotMap.CAN.RIGHT_SHOOTER);
        _shooterLeftMotor = new TalonFX(RobotMap.CAN.LEFT_SHOOTER);
        _hoodMotor = new TalonFX(RobotMap.CAN.HOOD);

        TalonFXConfiguration _shooterMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration _hoodMotorConfig = new TalonFXConfiguration();

        _shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _shooterMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.MotorConfig.SHOOTER_STATOR_CURRENT_LIMIT;
        _shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.MotorConfig.SHOOTER_SUPPLY_CURRENT_LIMIT;
        _shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        _shooterMotorConfig.Slot0.kP = Constants.Shooter.SHOOTER_KP;
        _shooterMotorConfig.Slot0.kI = Constants.Shooter.SHOOTER_KI;
        _shooterMotorConfig.Slot0.kD = Constants.Shooter.SHOOTER_KD;
        _shooterMotorConfig.Slot0.kV = Constants.Shooter.SHOOTER_KV;
        
        StatusCode rightShooterStatus = ApplyConfig.applyConfigWithRetry("Shooter Right", getName(), () -> _shooterRightMotor.getConfigurator().apply(_shooterMotorConfig));
        if (!rightShooterStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for right shooter motor: " + rightShooterStatus, false);
        }
        StatusCode leftShooterStatus = ApplyConfig.applyConfigWithRetry("Shooter Left", getName(), () -> _shooterLeftMotor.getConfigurator().apply(_shooterMotorConfig));
        if (!leftShooterStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for left shooter motor: " + leftShooterStatus, false);
        }
        StatusCode hoodStatus = ApplyConfig.applyConfigWithRetry("Hood", getName(), () -> _hoodMotor.getConfigurator().apply(_hoodMotorConfig));
        if (!hoodStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for hood motor: " + hoodStatus, false);
        }

        _shooterVelocityVoltage = new VelocityVoltage(0.0).withSlot(0);
        
        _targetVelocity = 0.0;
        _targetHoodAngle = new Rotation2d();
    }
    
    @Override
    public void periodic() {
        updateState();
    }

    public void setShooterVelocity(double velocity) {
        _targetVelocity = velocity;
    }

    private void updateState() {
        _shooterLeftMotor.setControl(_shooterVelocityVoltage.withVelocity(_targetVelocity));
        _shooterRightMotor.setControl(new Follower(RobotMap.CAN.LEFT_SHOOTER, MotorAlignmentValue.Opposed));
    }
}
