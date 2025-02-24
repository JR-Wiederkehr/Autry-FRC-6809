package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


@SuppressWarnings("unused")
public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);
		private final SparkMax armMotor1 = new SparkMax(10, MotorType.kBrushless);
		private final SparkMax armMotor2 = new SparkMax(9, MotorType.kBrushless);

		

    public ArmSubsystem() {
        
    }

    public void setArmStatus(double x){
  	   armMotor.set(x);
			 armMotor1.set(x);
    }

		public void setElevatorStatus(double x){
			armMotor2.set(x);
	 }
}
