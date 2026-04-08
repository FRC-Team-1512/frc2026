package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.ApplyConfig;

import frc.robot.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule extends SubsystemBase {

    private final TalonFX _steerMotor;
    private final TalonFX _driveMotor;
    private final CANcoder _encoder;

    private final PositionVoltage _steerPositionVoltage;
    private final VelocityVoltage _driveVelocityVoltage;

    private final Translation2d _moduleLocation;
    private final int _moduleIndex;
    private SwerveModuleState _desiredModuleState;

    private final StatusSignal<Angle> _drivePositionSignal;
    private final StatusSignal<AngularVelocity> _driveVelocitySignal;
    private final StatusSignal<Angle> _steerPositionSignal;

    // ======================================================================================

    public SwerveModule(int steerPort, int drivePort, int encoderPort, ModuleConfiguration config) {

        _steerMotor = new TalonFX(steerPort);
        _driveMotor = new TalonFX(drivePort);
        _encoder = new CANcoder(encoderPort);

        // -------------------------------------------------------------------------------------

        TalonFXConfiguration _steerMotorConfig = new TalonFXConfiguration();

        if(Constants.Drivetrain.STEER_INVERTED) {
            _steerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            _steerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        _steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        _steerMotorConfig.Feedback.FeedbackRemoteSensorID = _encoder.getDeviceID();
        _steerMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.STEER_SENSOR_TO_MECHANISM_RATIO;
        _steerMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.STEER_ROTOR_TO_SENSOR_RATIO;
        _steerMotorConfig.MotorOutput.NeutralMode = Constants.Drivetrain.STEER_MOTOR_NEUTRAL_MODE;
        _steerMotorConfig.Slot0.kP = Constants.Drivetrain.STEER_KP;
        _steerMotorConfig.Slot0.kI = Constants.Drivetrain.STEER_KI;
        _steerMotorConfig.Slot0.kD = Constants.Drivetrain.STEER_KD;
        _steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        //==== hard code ====
        _steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _steerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.MotorConfig.STEER_SUPPLY_CURRENT_LIMIT;
        _steerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _steerMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Drivetrain.MotorConfig.STEER_STATOR_CURRENT_LIMIT;

        _steerMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.STEER_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.STEER_PEAK_VOLTAGE));

        TalonFXConfiguration _driveMotorConfig = new TalonFXConfiguration();
        _driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.DRIVE_SENSOR_TO_MECHANISM_RATIO;
        _driveMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.DRIVE_ROTOR_TO_SENSOR_RATIO;
        _driveMotorConfig.MotorOutput.NeutralMode = Constants.Drivetrain.DRIVE_MOTOR_NEUTRAL_MODE;
        _driveMotorConfig.Slot0.kS = Constants.Drivetrain.DRIVE_KS;
        _driveMotorConfig.Slot0.kV = Constants.Drivetrain.DRIVE_KV;
        _driveMotorConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        _driveMotorConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        _driveMotorConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;

        //==== hard code ====
        _driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        _driveMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.MotorConfig.DRIVE_SUPPLY_CURRENT_LIMIT;
        _driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _driveMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.Drivetrain.MotorConfig.DRIVE_STATOR_CURRENT_LIMIT;
        _driveMotorConfig.Voltage.withPeakForwardVoltage(Volts.of(Constants.Drivetrain.DRIVE_PEAK_VOLTAGE))
            .withPeakReverseVoltage(Volts.of(-Constants.Drivetrain.DRIVE_PEAK_VOLTAGE));
        
        CANcoderConfiguration _encoderConfig = new CANcoderConfiguration();

        if (Constants.Drivetrain.STEER_INVERTED) {
            _encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }else {
            _encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }

        _encoderConfig.MagnetSensor.MagnetOffset = config.encoderOffset.getRotations();

        // -------------------------------------------------------------------------------------

        _steerPositionVoltage = new PositionVoltage(0).withSlot(0);

        _driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        // -------------------------------------------------------------------------------------

        StatusCode steerMotorStatus = ApplyConfig.applyConfigWithRetry("Steer", config.moduleName, () -> _steerMotor.getConfigurator().apply(_steerMotorConfig));
        if (!steerMotorStatus.isOK()) {
            DriverStation.reportError(
                "Failed to apply steer motor config for module " + config.moduleName + 
                " after retries. Status: " + steerMotorStatus,
                false
            );
        }
        StatusCode driveMotorStatus = ApplyConfig.applyConfigWithRetry("Drive", config.moduleName, () -> _driveMotor.getConfigurator().apply(_driveMotorConfig));
        if (!driveMotorStatus.isOK()) {
            DriverStation.reportError(
                "Failed to apply drive motor config for module " + config.moduleName + 
                " after retries. Status: " + driveMotorStatus,
                false
            );
        }
        StatusCode encoderStatus = ApplyConfig.applyConfigWithRetry("Encoder", config.moduleName, () -> _encoder.getConfigurator().apply(_encoderConfig));
        if (!encoderStatus.isOK()) {
            DriverStation.reportError("Failed to apply encoder config for module " + config.moduleName + 
                " after retries. Status: " + encoderStatus,
                false
            );
        }

        // -------------------------------------------------------------------------------------

        _moduleLocation = config.position;
        _moduleIndex = config.index;
        _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d(0.0));

        _drivePositionSignal = _driveMotor.getPosition();
        _driveVelocitySignal = _driveMotor.getVelocity();
        _steerPositionSignal = _encoder.getAbsolutePosition();
        
        // Optional: Optimize CAN bus utilization by changing update frequencies
        _drivePositionSignal.setUpdateFrequency(Constants.TICK_PER_SECOND);
        _driveVelocitySignal.setUpdateFrequency(Constants.TICK_PER_SECOND);
        _steerPositionSignal.setUpdateFrequency(Constants.TICK_PER_SECOND);
    }

    @Override
    public void periodic() {
        // Refresh signals together to reduce CAN bus latency and GC allocations
        BaseStatusSignal.refreshAll(_drivePositionSignal, _driveVelocitySignal, _steerPositionSignal);

        updateState();
    }

    // ======================================================================================

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = new SwerveModuleState(
            state.speedMetersPerSecond, 
            state.angle
        );
        optimizedState.optimize(getSteerAngle());
        
        _desiredModuleState.speedMetersPerSecond = optimizedState.speedMetersPerSecond;
        _desiredModuleState.angle = optimizedState.angle;
    }

    public void setState(double speedMetersPerSecond, Rotation2d angle) {
        setState(new SwerveModuleState(speedMetersPerSecond, angle));
    }

    private void updateState() {
        double steerRotationPosition = _desiredModuleState.angle.getRotations();
        _steerMotor.setControl(_steerPositionVoltage.withPosition(steerRotationPosition));

        double driveRotationsPerSecond = _desiredModuleState.speedMetersPerSecond 
            / (Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI);
        _driveMotor.setControl(_driveVelocityVoltage.withVelocity(driveRotationsPerSecond));
    }

    // ======================================================================================

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(_steerPositionSignal.getValueAsDouble());
    }

    public double getDriveVelocityMetersPerSecond() {
        double rps = _driveVelocitySignal.getValueAsDouble();
        return rps * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI;
    }

    public SwerveModuleState getDesiredState() {
        return _desiredModuleState;
    }

    public double getDrivePositionMeters() {
         double rotations = _drivePositionSignal.getValueAsDouble();
         return rotations * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePositionMeters(),
            getSteerAngle()
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocityMetersPerSecond(),
            getSteerAngle()
        );
    }

    public boolean isHealthy() {
        return _steerMotor.isAlive() && _driveMotor.isAlive() && _encoder.isConnected();
    }

    public Translation2d getLocation() {
        return _moduleLocation;
    }

    public int getIndex() {
        return _moduleIndex;
    }
    
    // ======================================================================================

    public static class ModuleConfiguration {
        public String moduleName = "";
        public int index = 0;
        public Translation2d position = new Translation2d();
        public Rotation2d encoderOffset = new Rotation2d(0.0);
    }
}
