package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;

public class Intake extends SubsystemBase {

    private final TalonFX _intakeWheelMotor;
    private final TalonFX _intakeArmMotor;

    private double _wheelPower;
    private Rotation2d _targetArmPosition;

    private PositionDutyCycle _intakeArmPositionDutyCycle;

    public Intake() {
        _intakeWheelMotor = new TalonFX(RobotMap.CAN.INTAKE_WHEEL);
        _intakeArmMotor = new TalonFX(RobotMap.CAN.INTAKE_ARM);

        TalonFXConfiguration _intakeWheelMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration _intakeArmMotorConfig = new TalonFXConfiguration();

        _intakeWheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _intakeWheelMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.MotorConfig.INTAKE_WHEEL_STATOR_CURRENT_LIMIT;
        _intakeWheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _intakeWheelMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.MotorConfig.INTAKE_WHEEL_SUPPLY_CURRENT_LIMIT;
        _intakeWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _intakeWheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        _intakeArmMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _intakeArmMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.MotorConfig.INTAKE_ARM_STATOR_CURRENT_LIMIT;
        _intakeArmMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _intakeArmMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.MotorConfig.INTAKE_ARM_SUPPLY_CURRENT_LIMIT;
        _intakeArmMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        _intakeArmMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        _intakeArmMotorConfig.Slot0.kP = Constants.Intake.INTAKE_ARM_KP;
        _intakeArmMotorConfig.Slot0.kI = Constants.Intake.INTAKE_ARM_KI;
        _intakeArmMotorConfig.Slot0.kD = Constants.Intake.INTAKE_ARM_KD;

        StatusCode wheelStatus = ApplyConfig.applyConfigWithRetry("Intake Wheel", getName(), () -> _intakeWheelMotor.getConfigurator().apply(_intakeWheelMotorConfig));
        if (!wheelStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake wheel motor: " + wheelStatus, false);
        }
        StatusCode armStatus = ApplyConfig.applyConfigWithRetry("Intake Arm", getName(), () -> _intakeArmMotor.getConfigurator().apply(_intakeArmMotorConfig));
        if (!armStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake arm motor: " + armStatus, false);
        }

        _wheelPower = 0.0;
        _targetArmPosition = Rotation2d.fromRotations(Constants.Intake.Hardware.INTAKE_ARM_MIN);
        _intakeArmPositionDutyCycle = new PositionDutyCycle(0.0).withSlot(0);
    }
    
    @Override
    public void periodic() {
        updateState();
    }

    public void setIntakeWheel(double power) {
        _wheelPower = power;
    }

    public void setIntakeArm(Rotation2d position) {
        double clampedPosition = MathUtil.clamp(position.getRotations(), Constants.Intake.Hardware.INTAKE_ARM_MIN, Constants.Intake.Hardware.INTAKE_ARM_MAX);
        _targetArmPosition = Rotation2d.fromRotations(clampedPosition);
    }

    public void extendArm() {
        setIntakeArm(Rotation2d.fromRotations(Constants.Intake.Hardware.INTAKE_ARM_INTAKE_POSITION));
    }

    public void retractArm() {
        setIntakeArm(Rotation2d.fromRotations(Constants.Intake.Hardware.INTAKE_ARM_RETRACT_POSITION));
    }

    public void intake() {
        extendArm();
        setIntakeWheel(Constants.Intake.Hardware.INTAKE_WHEEL_POWER);
    }

    public void reverseIntake() {
        extendArm();
        setIntakeWheel(Constants.Intake.Hardware.REVERSE_INTAKE_WHEEL_POWER);
    }

    public void retract() {
        retractArm();
        setIntakeWheel(Constants.Intake.Hardware.RETRACT_WHEEL_POWER);
    }

    public boolean isAtIntakePosition() {
        return Math.abs(_intakeArmMotor.getPosition().getValueAsDouble() - Constants.Intake.Hardware.INTAKE_ARM_INTAKE_POSITION) < Constants.Intake.Hardware.INTAKE_ARM_ACCURACY_TOLERANCE;
    }

    public boolean isAtRetractPosition() {
        return Math.abs(_intakeArmMotor.getPosition().getValueAsDouble() - Constants.Intake.Hardware.INTAKE_ARM_RETRACT_POSITION) < Constants.Intake.Hardware.INTAKE_ARM_ACCURACY_TOLERANCE;
    }

    private void updateState() {
        _intakeWheelMotor.set(_wheelPower);
        _intakeArmMotor.setControl(_intakeArmPositionDutyCycle.withPosition(_targetArmPosition.getRotations()));
    }

    public Command runIntakeWheel(double power) {
        return run(() -> setIntakeWheel(power));
    }

    public Command runIntakeArm(Rotation2d position) {
        return run(() -> setIntakeArm(position));
    }

}
