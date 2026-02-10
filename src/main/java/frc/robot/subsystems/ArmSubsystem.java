package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(9, MotorType.kBrushless);

    public ArmSubsystem(){

    }

    public void moveArm(double speed){
        armMotor.set(speed);
    }
}
