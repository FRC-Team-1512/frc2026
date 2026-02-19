package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

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
    private double _armPower;

    public Intake() {
        _intakeWheelMotor = new TalonFX(RobotMap.CAN.INTAKE_WHEEL);
        _intakeArmMotor = new TalonFX(RobotMap.CAN.INTAKE_ARM);

        TalonFXConfiguration _intakeWheelMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration _intakeArmMotorConfig = new TalonFXConfiguration();

        _intakeWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        _intakeWheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        _intakeArmMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        _intakeArmMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode wheelStatus = ApplyConfig.applyConfigWithRetry("Intake Wheel", getName(), () -> _intakeWheelMotor.getConfigurator().apply(_intakeWheelMotorConfig));
        if (!wheelStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake wheel motor: " + wheelStatus, false);
        }
        StatusCode armStatus = ApplyConfig.applyConfigWithRetry("Intake Arm", getName(), () -> _intakeArmMotor.getConfigurator().apply(_intakeArmMotorConfig));
        if (!armStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake arm motor: " + armStatus, false);
        }

        _wheelPower = 0.0;
        _armPower = 0.0;
    }
    
    @Override
    public void periodic() {
        updateState();
    }

    public void setIntakeWheel(double power) {
        _wheelPower = power;
    }

    public void setIntakeArm(double power) {
        _armPower = power;
    }

    private void updateState() {
        _intakeWheelMotor.setVoltage(_wheelPower * 12.0);
        _intakeArmMotor.setVoltage(_armPower * 12.0);
    }

    public Command runIntakeWheel(double power) {
        return run(() -> setIntakeWheel(power));
    }

    public Command runIntakeArm(double power) {
        return run(() -> setIntakeArm(power));
    }

}
