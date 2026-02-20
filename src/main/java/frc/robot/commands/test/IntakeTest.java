package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeTest extends Command {

    private final Intake _intake;

    public IntakeTest(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void execute() {
        double v = -applyDeadband(RobotContainer.operator.getLeftY(), 0.15);
        double w = -applyDeadband(RobotContainer.operator.getRightY(), 0.15);

        _intake.setIntakeWheel(-v);
        _intake.setIntakeArm(Rotation2d.fromRotations(w * 0.2));
    }

    private static double applyDeadband(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0.0;
        }else {
            return input;
        }
    }
}
