package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
     private final SparkFlex climbMotor = new SparkFlex(14, MotorType.kBrushless);
 
    public ClimbSubsystem(){

    }
    
    public void useClimber(Double speed){
        climbMotor.set(speed);
    }
}
    

