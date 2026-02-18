package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase{

    private final TalonFX _wheelsMotor;
    private final TalonFX _armMotor;


    public Intake(){
        _wheelsMotor = new TalonFX(RobotMap.CAN.INTAKE_WHEELS);
        _armMotor = new TalonFX(RobotMap.CAN.INTAKE_ARM);
    }

    public void setArmPosition(){

    
    }

    public void setWheelSpeed(double speed){

        _wheelsMotor.set(speed);

    }
    public double getArmPosition(){
        return _armMotor.getPosition().getValueAsDouble();
    }

    
}
