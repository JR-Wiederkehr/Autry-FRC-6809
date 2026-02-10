package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OverbumpSubsystem extends SubsystemBase {
    private final SparkFlex bumpMotor = new SparkFlex(12, MotorType.kBrushless);
 
    public OverbumpSubsystem(){

    }
    
    public void useOverbump(Double speed){
        bumpMotor.set(speed);
    }
}
