/*package frc.robot;
import com.studica.frc.AHRS;


public class IMUCompensation {
    public final AHRS navX = new AHRS(AHRS.NavXComType.kUSB1);

    // Define the offset (in meters) from the robot center
    private static final double X_OFFSET = 13.97; // Example: 10 cm to the right
    private static final double Y_OFFSET = 27.94; // Example: 20 cm forward

    @SuppressWarnings("unused")
    public void update() {
        // Get angular velocity in radians per second
        double omega = Math.toRadians(navX.getRate()); 

        // Compute the velocity correction due to rotation
        double vX_correction = -omega * Y_OFFSET;
        double vY_correction = omega * X_OFFSET;

        // Get the raw velocity from NavX
        double rawVx = navX.getVelocityX(); // m/s
        double rawVy = navX.getVelocityY(); // m/s

        // Apply the correction
        double correctedVx = rawVx - vX_correction;
        double correctedVy = rawVy - vY_correction;

        // Get angular velocity (degrees per second → convert to radians per second)
        double aOmega = Math.toRadians(navX.getRate()); 

        // Compute the velocity correction due to rotation
        double aX_correction = -omega * Y_OFFSET;
        double aY_correction = omega * X_OFFSET;

        // Get the raw yaw from the IMU
        double rawYaw = navX.getYaw();

        // Correct the yaw based on the offset (integrate velocity correction)
        double correctedYaw = rawYaw + Math.toDegrees(Math.atan2(vY_correction, vX_correction));

    }

    public double getCorrectedYaw() {
        // Get angular velocity (degrees per second → convert to radians per second)
        double aOmega = Math.toRadians(navX.getRate()); 

        // Compute the velocity correction due to rotation
        double aX_correction = -aOmega * Y_OFFSET;
        double aY_correction = aOmega * X_OFFSET;

        // Get the raw yaw from the IMU
        double rawYaw = navX.getYaw();

        // Correct the yaw based on the offset (integrate velocity correction)
        double correctedYaw = rawYaw -90 + Math.toDegrees(Math.atan2(aY_correction, aX_correction));

        return correctedYaw;
}}*/
