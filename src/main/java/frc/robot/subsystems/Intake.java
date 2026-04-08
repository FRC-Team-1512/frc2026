package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.PositionDutyCycle;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.ApplyConfig;

public class Intake extends SubsystemBase {

    private final TalonFX _intakeWheelMotor;
    private final TalonFX _intakeArmMotor;

    private final com.ctre.phoenix6.StatusSignal<edu.wpi.first.units.measure.Angle> _intakeArmPositionSignal;

    private double _wheelPower;
    // private Rotation2d _targetArmPosition;
    // private PositionDutyCycle _intakeArmPositionDutyCycle;

    public enum ArmState {
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED
    }

    private ArmState _armState = ArmState.RETRACTED;
    private DutyCycleOut _dutyCycleOut = new DutyCycleOut(0.0);
    private final Timer _stateTimer = new Timer();

    private final DoublePublisher _armPositionPublisher;
    private final StringPublisher _armStatePublisher;
    private final DoublePublisher _armCurrentPublisher;

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
        _intakeArmMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Intake.MotorConfig.DUTY_CYCLE_OPEN_LOOP_RAMP_PERIOD; // Ramps output to protect belt
        // _intakeArmMotorConfig.Slot0.kP = Constants.Intake.INTAKE_ARM_KP;
        // _intakeArmMotorConfig.Slot0.kI = Constants.Intake.INTAKE_ARM_KI;
        // _intakeArmMotorConfig.Slot0.kD = Constants.Intake.INTAKE_ARM_KD;

        StatusCode wheelStatus = ApplyConfig.applyConfigWithRetry("Intake Wheel", getName(), () -> _intakeWheelMotor.getConfigurator().apply(_intakeWheelMotorConfig));
        if (!wheelStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake wheel motor: " + wheelStatus, false);
        }
        StatusCode armStatus = ApplyConfig.applyConfigWithRetry("Intake Arm", getName(), () -> _intakeArmMotor.getConfigurator().apply(_intakeArmMotorConfig));
        if (!armStatus.isOK()) {
            DriverStation.reportError("Failed to apply config for intake arm motor: " + armStatus, false);
        }

        _wheelPower = 0.0;
        // _targetArmPosition = Rotation2d.fromRotations(Constants.Intake.Hardware.INTAKE_ARM_RETRACT_POSITION);
        // _intakeArmPositionDutyCycle = new PositionDutyCycle(0.0).withSlot(0);

        _intakeArmPositionSignal = _intakeArmMotor.getPosition();

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");
        _armPositionPublisher = table.getDoubleTopic("ArmPosition").publish();
        _armStatePublisher = table.getStringTopic("ArmState").publish();
        _armCurrentPublisher = table.getDoubleTopic("ArmCurrent").publish();
    }
    
    @Override
    public void periodic() {
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(_intakeArmPositionSignal);
        updateState();
        _armPositionPublisher.set(_intakeArmPositionSignal.getValueAsDouble());
        _armStatePublisher.set(_armState.toString());
        _armCurrentPublisher.set(_intakeArmMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Intake: Intake Arm Desired Position", _targetArmPosition.getRotations());
    }

    public void setIntakeWheel(double power) {
        _wheelPower = power;
    }

    // public void setIntakeArm(Rotation2d position) {
    //     double clampedPosition = MathUtil.clamp(position.getRotations(), Constants.Intake.Hardware.INTAKE_ARM_MIN, Constants.Intake.Hardware.INTAKE_ARM_MAX);
    //     _targetArmPosition = Rotation2d.fromRotations(clampedPosition);
    // }

    public void extendArm() {
        if (_armState != ArmState.EXTENDED && _armState != ArmState.EXTENDING) {
            _armState = ArmState.EXTENDING;
            _stateTimer.restart();
        }
    }

    public void retractArm() {
        if (_armState != ArmState.RETRACTED && _armState != ArmState.RETRACTING) {
            _armState = ArmState.RETRACTING;
            _stateTimer.restart();
        }
    }

    // public void extendHalfArm() {
    //     setIntakeArm(Rotation2d.fromRotations((Constants.Intake.Hardware.INTAKE_ARM_INTAKE_POSITION + Constants.Intake.Hardware.INTAKE_ARM_RETRACT_POSITION) / 2.0));
    // }

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
        return _armState == ArmState.EXTENDED;
    }

    public boolean isAtRetractPosition() {
        return _armState == ArmState.RETRACTED;
    }

    private void updateState() {
        _intakeWheelMotor.set(_wheelPower);

        double armCurrent = _intakeArmMotor.getStatorCurrent().getValueAsDouble();

        switch (_armState) {
            case EXTENDING:
                _intakeArmMotor.setControl(_dutyCycleOut.withOutput(Constants.Intake.Hardware.INTAKE_ARM_EXTEND_POWER));
                if (_stateTimer.hasElapsed(Constants.Intake.Hardware.INTAKE_ARM_DEBOUNCE_SEC) && armCurrent > Constants.Intake.Hardware.INTAKE_ARM_CURRENT_THRESHOLD) {
                    _armState = ArmState.EXTENDED;
                }
                break;
            case RETRACTING:
                _intakeArmMotor.setControl(_dutyCycleOut.withOutput(Constants.Intake.Hardware.INTAKE_ARM_RETRACT_POWER));
                if ((_stateTimer.hasElapsed(Constants.Intake.Hardware.INTAKE_ARM_DEBOUNCE_SEC) && armCurrent > Constants.Intake.Hardware.INTAKE_ARM_CURRENT_THRESHOLD) ||
                        _stateTimer.hasElapsed(Constants.Intake.Hardware.INTAKE_ARM_RETRACT_TIMEOUT_SEC)) {
                    _armState = ArmState.RETRACTED;
                }                /*
                if ((_stateTimer.hasElapsed(0.25) && armCurrent > Constants.Intake.Hardware.INTAKE_ARM_CURRENT_THRESHOLD)) {
                    _armState = ArmState.RETRACTED;
                }
                */
                break;
            case EXTENDED:
                _intakeArmMotor.setControl(_dutyCycleOut.withOutput(Constants.Intake.Hardware.INTAKE_ARM_KEEP_EXTENDED_POWER));
                break;
            case RETRACTED:
                _intakeArmMotor.setControl(_dutyCycleOut.withOutput(Constants.Intake.Hardware.INTAKE_ARM_KEEP_RETRACTED_POWER));
                break;
        }
        // _intakeArmMotor.setControl(_intakeArmPositionDutyCycle.withPosition(_targetArmPosition.getRotations()));
    }

    public Command runIntakeWheel(double power) {
        return run(() -> setIntakeWheel(power));
    }

    // public Command runIntakeArm(Rotation2d position) {
    //     return run(() -> setIntakeArm(position));
    // }
}
