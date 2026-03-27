package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SuperStructure extends SubsystemBase {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;
    private final Supplier<Pose2d> _poseSupplier;
    private final Supplier<Translation2d> _velocitySupplier;

    private double _targetDistanceMeters;

    private final EnumSet<SuperStructureState> _activeStates = EnumSet.noneOf(SuperStructureState.class);
    private SuperStructureState _defaultIdleState;

    public final DirectControl direct = new DirectControl();

    public SuperStructure(Intake intake, Indexer indexer, Shooter shooter, Supplier<Pose2d> poseSupplier, Supplier<Translation2d> velocitySupplier) {
        _intake = intake;
        _indexer = indexer;
        _shooter = shooter;
        _poseSupplier = poseSupplier;
        _velocitySupplier = velocitySupplier;

        _targetDistanceMeters = 0.0;

        _defaultIdleState = SuperStructureState.IDLE_EXPANDED;

        _activeStates.add(_defaultIdleState);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = _poseSupplier.get();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean _isRedAlliance = alliance.filter(value -> value == Alliance.Red).isPresent();
        
        Translation2d target;
        if(_isRedAlliance) {
            target = Constants.TARGET_RED;
        } else {
            target = Constants.TARGET_BLUE;
        }
        _targetDistanceMeters = target.getDistance(currentPose.getTranslation());

        SmartDashboard.putNumber("SuperStructure: Distance Meters", _targetDistanceMeters);

        if (_activeStates.contains(SuperStructureState.INTAKE)) {
            _intake.intake();
        }
        if (_activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _intake.reverseIntake();
        }
        if (!_activeStates.contains(SuperStructureState.REVERSE_INTAKE) && !_activeStates.contains(SuperStructureState.INTAKE) && _defaultIdleState == SuperStructureState.IDLE) {
            _intake.retract();
        }
        if (_activeStates.contains(SuperStructureState.SHOOT)) {
            _shooter.setShooterFromDistance(_targetDistanceMeters);
            if (_shooter.isReadyToShoot()) {
                _indexer.setIndexer(1.0);
            } else {
                _indexer.setIndexer(0.0);
            }
        } else if (!_activeStates.contains(SuperStructureState.IDLE) && !_activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _shooter.setShooterVelocity(0.0);
            _indexer.setIndexer(0.0);
        }
        if (_activeStates.contains(SuperStructureState.IDLE)) {
            _intake.retract();
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
        }
        if (_activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _intake.extendArm();
            _intake.setIntakeWheel(0.0);
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
        }

        publishState();
    }

    public void publishState() {
        SmartDashboard.putBoolean("SuperStructure: IDLE", _activeStates.contains(SuperStructureState.IDLE));
        SmartDashboard.putBoolean("SuperStructure: IDLE EXPANDED", _activeStates.contains(SuperStructureState.IDLE_EXPANDED));
        SmartDashboard.putBoolean("SuperStructure: INTAKE", _activeStates.contains(SuperStructureState.INTAKE));
        SmartDashboard.putBoolean("SuperStructure: REVERSE_INTAKE", _activeStates.contains(SuperStructureState.REVERSE_INTAKE));
        SmartDashboard.putBoolean("SuperStructure: SHOOT", _activeStates.contains(SuperStructureState.SHOOT));
    }

    public void requestState(SuperStructureState state) {
        sanitizeStates(state);
    }

    public void revokeState(SuperStructureState state) {
        _activeStates.remove(state);
        sanitizeStates();
    }

    // public void setShootDistance(double distanceMeters) {
    //     _targetDistanceMeters = distanceMeters;
    // }

    public boolean isShootingMode() {
        return _activeStates.contains(SuperStructureState.SHOOT);
    }

    public class DirectControl {
        public void setIntakeWheel(double power) {
            _intake.setIntakeWheel(power);
        }

        public void setIntakeArm(Rotation2d position) {
            _intake.setIntakeArm(position);
        }

        public void setIndexer(double power) {
            _indexer.setIndexer(power);
        }

        public void setShooterVelocity(double velocity) {
            _shooter.setShooterVelocity(velocity);
        }

        public void setShooterHoodAngle(double rotations) {
            _shooter.setHoodAngleRotations(rotations);
        }
    }

    public enum SuperStructureState {
        IDLE,
        IDLE_EXPANDED,
        INTAKE,
        REVERSE_INTAKE,
        SHOOT,
    }

    private void sanitizeStates(SuperStructureState newState) {
        if (newState == SuperStructureState.REVERSE_INTAKE) {
            _activeStates.remove(SuperStructureState.INTAKE);
        } else if (newState == SuperStructureState.INTAKE) {
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (newState == SuperStructureState.IDLE) {
            _defaultIdleState = SuperStructureState.IDLE;
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        } else if (newState == SuperStructureState.IDLE_EXPANDED) {
            _defaultIdleState = SuperStructureState.IDLE_EXPANDED;
            _activeStates.remove(SuperStructureState.IDLE);
        }
        if (newState != SuperStructureState.IDLE && newState != SuperStructureState.IDLE_EXPANDED) {
            _activeStates.remove(SuperStructureState.IDLE);
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        _activeStates.add(newState);
        sanitizeStates();
    }

    private void sanitizeStates() {
        if (_activeStates.isEmpty()) {
            _activeStates.add(_defaultIdleState);
        }
        if (_activeStates.contains(SuperStructureState.IDLE) && _activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        if (_activeStates.contains(SuperStructureState.INTAKE) && _activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (_activeStates.contains(SuperStructureState.IDLE) || _activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _activeStates.remove(SuperStructureState.INTAKE);
            _activeStates.remove(SuperStructureState.REVERSE_INTAKE);
            _activeStates.remove(SuperStructureState.SHOOT);
        }
    }

    public Command requestIntake() {
        return runOnce(() -> requestState(SuperStructureState.INTAKE));
    }

    public Command requestReverseIntake() {
        return runOnce(() -> requestState(SuperStructureState.REVERSE_INTAKE));
    }

    public Command requestShoot() {
        return runOnce(() -> requestState(SuperStructureState.SHOOT));
    }

    public Command requestIdle() {
        return runOnce(() -> requestState(SuperStructureState.IDLE));
    }

    public Command requestIdleExpanded() {
        return runOnce(() -> requestState(SuperStructureState.IDLE_EXPANDED));
    }

    public Command revokeShoot() {
        return runOnce(() -> revokeState(SuperStructureState.SHOOT));
    }

    public Command revokeIntake() {
        return runOnce(() -> revokeState(SuperStructureState.INTAKE));
    }
}
