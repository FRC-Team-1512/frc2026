package frc.robot.subsystems;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    private final Intake _intake;
    private final Indexer _indexer;
    private final Shooter _shooter;

    private final EnumSet<SuperStructureState> activeStates = EnumSet.noneOf(SuperStructureState.class);

    public final DirectControl direct = new DirectControl();

    public SuperStructure(Intake intake, Indexer indexer, Shooter shooter) {
        _intake = intake;
        _indexer = indexer;
        _shooter = shooter;
    }

    public void requestState(SuperStructureState state) {
        sanitizeStates(state);
    }

    public void revokeState(SuperStructureState state) {
        activeStates.remove(state);
        sanitizeStates();
    }

    public class DirectControl {
        public void setIntakeWheel(double power) {
            _intake.setIntakeWheel(power);
        }

        public void setIntakeArm(double power) {
            _intake.setIntakeArm(power);
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
