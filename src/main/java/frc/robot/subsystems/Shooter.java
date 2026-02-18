package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private final TalonFX _shooterRightMotor;
    private final TalonFX _shooterLeftMotor;
    private final TalonFX _hood;

    public Shooter() {
        _shooterRightMotor = new TalonFX(RobotMap.CAN.RIGHT_SHOOTER);
        _shooterLeftMotor = new TalonFX(RobotMap.CAN.LEFT_SHOOTER);
        _hood = new TalonFX(RobotMap.CAN.HOOD);
    }
    
}
