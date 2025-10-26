package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final TalonFX _steerMotor;
    private final TalonFX _driveMotor;

    private final TalonFXConfiguration _steerMotorConfig;
    private final TalonFXConfiguration _driveMotorConfig;

    private final Translation2d _moduleLocation;
    private final int _moduleIndex;
    private SwerveModuleState _desiredModuleState;
    private Rotation2d _steerAngularOffset;

    public SwerveModule(int steerPort, int drivePort, int encoderPort, ModuleConfiguration config) {
        _steerMotor = new TalonFX(steerPort, config.canBus);
        _driveMotor = new TalonFX(drivePort, config.canBus);

        _steerMotorConfig = new TalonFXConfiguration();
        _steerMotorConfig.Feedback.FeedbackRemoteSensorID = encoderPort;
        _steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        _steerMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.STEER_SENSOR_TO_MECHANISM_RATIO;
        _steerMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.STEER_ROTOR_TO_SENSOR_RATIO;
        _steerMotorConfig.Slot0.kP = Constants.Drivetrain.STEER_KP;
        _steerMotorConfig.Slot0.kI = Constants.Drivetrain.STEER_KI;
        _steerMotorConfig.Slot0.kD = Constants.Drivetrain.STEER_KD;
        _steerMotor.getConfigurator().apply(_steerMotorConfig);

        _driveMotorConfig = new TalonFXConfiguration();
        _driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.Hardware.DRIVE_SENSOR_TO_MECHANISM_RATIO;
        _driveMotorConfig.Feedback.RotorToSensorRatio = Constants.Drivetrain.Hardware.DRIVE_ROTOR_TO_SENSOR_RATIO;
        _driveMotorConfig.Slot0.kP = Constants.Drivetrain.DRIVE_KP;
        _driveMotorConfig.Slot0.kI = Constants.Drivetrain.DRIVE_KI;
        _driveMotorConfig.Slot0.kD = Constants.Drivetrain.DRIVE_KD;
        _driveMotor.getConfigurator().apply(_driveMotorConfig);

        _moduleLocation = config.position;
        _moduleIndex = config.index;
        _desiredModuleState = new SwerveModuleState(0.0, new Rotation2d(0.0));
        _steerAngularOffset = config.encoderOffset;
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = new SwerveModuleState(
            state.speedMetersPerSecond, 
            state.angle.plus(_steerAngularOffset)
        );
        optimizedState.angle = optimizedState.angle.plus(_steerAngularOffset);
        optimizedState.optimize(getSteerAngle().minus(_steerAngularOffset));

        PositionDutyCycle steerRequest = new PositionDutyCycle(optimizedState.angle.getRadians());
        _steerMotor.setControl(steerRequest);

        VelocityDutyCycle driveRequest = new VelocityDutyCycle(optimizedState.speedMetersPerSecond);
        _driveMotor.setControl(driveRequest);

        _desiredModuleState = state;
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(_steerMotor.getPosition().getValue().in(Units.Radians)).minus(_steerAngularOffset);
    }

    public SwerveModuleState getDesiredState() {
        return _desiredModuleState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition (
            _driveMotor.getPosition().getValue().in(Units.Rotations) * Constants.Drivetrain.Hardware.WHEEL_DIAMETER_METER * Math.PI,
            getSteerAngle()
        );
    }

    public Translation2d getLocation() {
        return _moduleLocation;
    }

    public int getIndex() {
        return _moduleIndex;
    }

    public static class ModuleConfiguration {
        public String moduleName = "";
        public int index = 0;
        public Translation2d position = new Translation2d();
        public Rotation2d encoderOffset = new Rotation2d(0.0);
        public boolean encoderInverted = false;
        public String canBus = "CANivore";
    }
}
