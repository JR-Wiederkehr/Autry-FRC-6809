package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax upperIntakeMotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkFlex lowerIntakeMotor = new SparkFlex(10, MotorType.kBrushless);
    private final SparkFlex shooterMotor = new SparkFlex(11, MotorType.kBrushless);
    private final SparkFlex agitatorMotor = new SparkFlex(13, MotorType.kBrushless);

    private static final double kP = 0.0015;
    private static final double kI = 0.0;
    private static final double kD = 0.0001;

    private static final int kCurrentLimitAmps = 60;
    private static final double targetRPM = 3500.0;

    private final RelativeEncoder m_encoder = shooterMotor.getEncoder();
    private final SparkClosedLoopController m_controller = shooterMotor.getClosedLoopController();

    public IntakeSubsystem(){
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(kCurrentLimitAmps);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void useIntake(double speed){
        upperIntakeMotor.set(speed);
        lowerIntakeMotor.set(speed);
        agitatorMotor.set(speed);
    }

    public void spinUp(double speed){
        m_controller.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public double getRPM(){
        return m_encoder.getVelocity();
    }

    public void useShooter(double speed){
        upperIntakeMotor.set(-speed);
        lowerIntakeMotor.set(speed);
        agitatorMotor.set(speed);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("FlywheelRPM", getRPM());
    }
}
