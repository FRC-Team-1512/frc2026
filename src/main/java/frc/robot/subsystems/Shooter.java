package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;
import frc.robot.utils.ShooterCalc;


public class Shooter extends SubsystemBase {

    private final TalonFX _shooterRightMotor;
    private final TalonFX _shooterLeftMotor;
    private final TalonFX _hoodMotor;

    private final VelocityVoltage _shooterVelocityVoltage;
    private final PositionDutyCycle _hoodPositionDutyCycle;

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

        _hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        _hoodMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        _hoodMotorConfig.Slot0.kP = Constants.Shooter.HOOD_KP;
        _hoodMotorConfig.Slot0.kI = Constants.Shooter.HOOD_KI;
        _hoodMotorConfig.Slot0.kD = Constants.Shooter.HOOD_KD;
        _hoodMotorConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.Hardware.HOOD_SENSOR_TO_MECHANISM_RATIO;
        _hoodMotorConfig.Feedback.RotorToSensorRatio = Constants.Shooter.Hardware.HOOD_ROTOR_TO_SENSOR_RATIO;
        _hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        
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
        _hoodPositionDutyCycle = new PositionDutyCycle(0.0).withSlot(0);
        
        _targetVelocity = 0.0;
        _targetHoodAngle = Rotation2d.fromRotations(Constants.Shooter.Hardware.HOOD_MIN);
    }
    
    @Override
    public void periodic() {
        updateState();
        SmartDashboard.putNumber("HOOD TARGET", _targetHoodAngle.getRotations());
        SmartDashboard.putNumber("FLYWHEEL VEL TARGET", _targetVelocity);
    }

    public void setShooterFromDistance(double distance){
        _targetVelocity = ShooterCalc.getRPSFromDistance(distance);
        _targetHoodAngle = ShooterCalc.getLocalHoodAngleFromDistance(distance);
    }

    public void setShooterVelocity(double velocity) {
        _targetVelocity = velocity;
    }

    public void setHoodAngle(Rotation2d angle){
        double clampedAngle = MathUtil.clamp(angle.getRotations(), Constants.Shooter.Hardware.HOOD_MIN, Constants.Shooter.Hardware.HOOD_MAX);
        _targetHoodAngle = Rotation2d.fromRotations(clampedAngle);
    }

    public double getHoodAngle(){
        return _hoodMotor.getPosition().getValueAsDouble();
    }

    public double getShooterVelocity(){
        return _shooterRightMotor.getVelocity().getValueAsDouble();
    }
    
    public double getDesiredVelocity(){
        return _targetVelocity;
    }

    public double getDesiredAngle() {
        return _targetHoodAngle.getRotations();
    }

    private void updateState() {
        _shooterLeftMotor.setControl(_shooterVelocityVoltage.withVelocity(_targetVelocity));
        _shooterRightMotor.setControl(new Follower(RobotMap.CAN.LEFT_SHOOTER, MotorAlignmentValue.Opposed));
        _hoodMotor.setControl(_hoodPositionDutyCycle.withPosition(_targetHoodAngle.getRotations()));
    }
}
