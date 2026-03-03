package frc.robot.subsystems;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;

    private double _targetDistanceMeters;

    private final EnumSet<SuperStructureState> _activeStates = EnumSet.noneOf(SuperStructureState.class);
    private SuperStructureState _defaultIdleState;;

    public final DirectControl direct = new DirectControl();

    public SuperStructure(Intake intake, Indexer indexer, Shooter shooter) {
        _intake = intake;
        _indexer = indexer;
        _shooter = shooter;

        _targetDistanceMeters = 0.0;

        _defaultIdleState = SuperStructureState.IDLE;

        _activeStates.add(_defaultIdleState);
    }

    @Override
    public void periodic() {
        if (_activeStates.contains(SuperStructureState.SHOOT)) {
            _shooter.setShooterFromDistance(_targetDistanceMeters);
        } else {
            _shooter.setShooterVelocity(0.0);
            _indexer.setIndexer(0.0);
        }
        /*
        if (_activeStates.contains(SuperStructureState.INTAKE)) {
            _intake.intake();
        }
        if (_activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _intake.reverseIntake();
        }
        if (!_activeStates.contains(SuperStructureState.REVERSE_INTAKE) && !_activeStates.contains(SuperStructureState.INTAKE)) {
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
        */

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

    public void setShootDistance(double distanceMeters) {
        _targetDistanceMeters = distanceMeters;
    }

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

        public void setShooterHoodAngle(Rotation2d angle) {
            _shooter.setHoodAngle(angle);
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
