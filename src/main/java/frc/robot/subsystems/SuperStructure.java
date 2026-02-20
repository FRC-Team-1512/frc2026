package frc.robot.subsystems;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;

    private double _targetDistanceMeters;

    private final EnumSet<SuperStructureState> activeStates = EnumSet.noneOf(SuperStructureState.class);

    public final DirectControl direct = new DirectControl();

    public SuperStructure(Intake intake, Indexer indexer, Shooter shooter) {
        _intake = intake;
        _indexer = indexer;
        _shooter = shooter;

        _targetDistanceMeters = 0.0;
    }

    @Override
    public void periodic() {
        if (activeStates.contains(SuperStructureState.INTAKE)) {
            _intake.intake();
        }
        if (activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            _intake.reverseIntake();
        }
        if (!activeStates.contains(SuperStructureState.REVERSE_INTAKE) && !activeStates.contains(SuperStructureState.INTAKE)) {
            _intake.retract();
        }
        if (activeStates.contains(SuperStructureState.SHOOT)) {
            _shooter.setShooterFromDistance(_targetDistanceMeters);
            if (_shooter.isReadyToShoot()) {
                _indexer.setIndexer(1.0);
            } else {
                _indexer.setIndexer(0.0);
            }
        }
        if (activeStates.contains(SuperStructureState.IDLE)) {
            _intake.retract();
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
        }
        if (activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            _intake.extendArm();
            _intake.setIntakeWheel(0.0);
            _indexer.setIndexer(0.0);
            _shooter.setShooterVelocity(0.0);
        }
    }

    public void requestState(SuperStructureState state) {
        sanitizeStates(state);
    }

    public void revokeState(SuperStructureState state) {
        activeStates.remove(state);
        sanitizeStates();
    }

    public void setShootDistance(double distanceMeters) {
        _targetDistanceMeters = distanceMeters;
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
            activeStates.remove(SuperStructureState.INTAKE);
        } else if (newState == SuperStructureState.INTAKE) {
            activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (newState != SuperStructureState.IDLE && newState != SuperStructureState.IDLE_EXPANDED) {
            activeStates.remove(SuperStructureState.IDLE);
            activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        activeStates.add(newState);
        sanitizeStates();
    }

    private void sanitizeStates() {
        if (activeStates.isEmpty()) {
            activeStates.add(SuperStructureState.IDLE);
        }
        if (activeStates.contains(SuperStructureState.IDLE) && activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            activeStates.remove(SuperStructureState.IDLE_EXPANDED);
        }
        if (activeStates.contains(SuperStructureState.INTAKE) && activeStates.contains(SuperStructureState.REVERSE_INTAKE)) {
            activeStates.remove(SuperStructureState.REVERSE_INTAKE);
        }
        if (activeStates.contains(SuperStructureState.IDLE) || activeStates.contains(SuperStructureState.IDLE_EXPANDED)) {
            activeStates.remove(SuperStructureState.INTAKE);
            activeStates.remove(SuperStructureState.REVERSE_INTAKE);
            activeStates.remove(SuperStructureState.SHOOT);
        }
    }
}
