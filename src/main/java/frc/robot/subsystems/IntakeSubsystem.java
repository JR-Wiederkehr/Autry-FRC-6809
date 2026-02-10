package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax upperIntakeMotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkFlex lowerIntakeMotor = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlex shooterMotor = new SparkFlex(11, MotorType.kBrushless);
    public IntakeSubsystem(){

    }

    public void useIntake(double speed){
        upperIntakeMotor.set(speed);
        lowerIntakeMotor.set(speed);
    }

    public void spinUp(double speed){
        shooterMotor.set(-speed);
    }

    public void useShooter(double speed){
        upperIntakeMotor.set(-speed);
        lowerIntakeMotor.set(speed);
    }
}
