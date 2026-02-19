package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;

public class Intake extends SubsystemBase {

    private final TalonFX _intakeWheelMotor;
    private final TalonFX _intakeArmMotor;

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
    }
    
    @Override
    public void periodic() {
        updateState();
    }

    public void setIntakeWheel(double power) {
        _intakeWheelMotor.setVoltage(power * 12.0);
    }

    public void setIntakeArm(double power) {
        _intakeArmMotor.setVoltage(power * 12.0);
    }

    private void updateState() {
        // Update any necessary state here
    }

}
