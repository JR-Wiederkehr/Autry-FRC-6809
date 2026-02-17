package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;

public class ServoSubsystem extends SubsystemBase {
    private final ServoHub servoHub = new ServoHub(23);
    private final ServoHubConfig config = new ServoHubConfig();
    private final ServoChannel channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);

    public double target;

    @SuppressWarnings("removal")
    public ServoSubsystem(){
    config
        .channel0.pulseRange(500, 1500, 2500)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

    // Persist parameters and reset any not explicitly set above to
    // their defaults.
    servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);

    channel0.setPowered(true);
    channel0.setEnabled(true);
    channel0.setPulseWidth(500);
}
    

    private static int conversion(double x){
        x = Math.max(-1.0, Math.min(1.0, x));
        int y = (int) Math.round(1500 + 1000*x);
        return Math.max(500, Math.min(2500, y));
    }

    public void moveServo(){
        channel0.setPulseWidth(conversion(target));
    }

    public void setServoTarget(double x){
        target = x;
    }

}
