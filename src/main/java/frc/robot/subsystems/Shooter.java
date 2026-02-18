package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private final TalonFX _shooterRightMotor;
    private final TalonFX _shooterLeftMotor;
    private final TalonFX _hood;
    private double desiredRPM;
    

    public Shooter() {
        _shooterRightMotor = new TalonFX(RobotMap.CAN.RIGHT_SHOOTER);
        _shooterLeftMotor = new TalonFX(RobotMap.CAN.LEFT_SHOOTER);
        _hood = new TalonFX(RobotMap.CAN.HOOD);
        _shooterRightMotor.setControl(new Follower(_shooterLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned)); //TODO make sure that this is the right configuration
    }
    public void setRpm(double rpm){
        desiredRPM = rpm;
        _shooterLeftMotor.setControl(new VelocityDutyCycle(rpm));

    }
    public double getRPM(){
        return _shooterLeftMotor.getVelocity().getValueAsDouble();
 

    }

    
}
