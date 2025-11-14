package org.firstinspires.ftc.teamcode.mechanisms;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GyroInterface {
    private AHRS navx;
    private navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private boolean calibration_complete = false;

    private double yawOffset = 0.0;

    private Pose2D position = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    private double velocityX = 0.0; // inches/sec
    private double velocityY = 0.0; // inches/sec

    // For filtered acceleration
    private double filteredAccelX = 0.0;
    private double filteredAccelY = 0.0;

    // Constants for filtering & damping
    private final double ALPHA = 0.1; // smoothing factor for acceleration
    private final double VELOCITY_DAMPING = 0.99; // reduces drift over time
    private final double MAX_VELOCITY = 50.0; // maximum velocity in inches/sec



    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx.zeroYaw();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(navx.getYaw() + yawOffset);
    }

    /**
     * Aligns the robot's internal heading and optionally position using an AprilTag reading.
     *
     * @param detectedPose Pose2D from AprilTag detection. If null, only heading is aligned.
     */
    public void alignHeadingWithVision(Pose2D detectedPose) {
        if (detectedPose != null) {
            // Update position
            position = detectedPose;

            // Update heading
            double currentYaw = navx.getYaw();
            yawOffset = Math.toDegrees(detectedPose.getHeading(AngleUnit.RADIANS)) - currentYaw;

            // Reset velocity to prevent drift
            velocityX = 0;
            velocityY = 0;
        }
    }

    public void resetInternalYawToZero() {
        navx.zeroYaw(); // Resets the *internal* reference to 0
        yawOffset = 0.0; // Reset our software offset too if zeroYaw() is used for a full reset.
    }

    public void setYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
    }

    public Pose2D getPosition() {
        return position;
    }

    public void setPosition(Pose2D position) {
        this.position = position;
    }

    public void updatePosition(double dtSeconds) {
        // Get raw acceleration (inches/sec²)
        double rawAccelX = navx.getWorldLinearAccelX() * 39.3701;
        double rawAccelY = navx.getWorldLinearAccelY() * 39.3701;

        // Low-pass filter
        filteredAccelX = ALPHA * rawAccelX + (1 - ALPHA) * filteredAccelX;
        filteredAccelY = ALPHA * rawAccelY + (1 - ALPHA) * filteredAccelY;

        // Integrate acceleration → velocity
        velocityX += filteredAccelX * dtSeconds;
        velocityY += filteredAccelY * dtSeconds;

        // Apply damping and clamp
        velocityX *= VELOCITY_DAMPING;
        velocityY *= VELOCITY_DAMPING;
        velocityX = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, velocityX));
        velocityY = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, velocityY));

        // Rotate robot-centric velocity → field frame
        double headingRad = Math.toRadians(navx.getYaw() + yawOffset);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double fieldVelX = velocityX * cosH - velocityY * sinH;
        double fieldVelY = velocityX * sinH + velocityY * cosH;

        // Integrate velocity → position
        double newX = position.getX(DistanceUnit.INCH) + fieldVelX * dtSeconds;
        double newY = position.getY(DistanceUnit.INCH) + fieldVelY * dtSeconds;
        position = new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.RADIANS, headingRad);

        // reset velocity if robot is nearly stationary
        if (Math.abs(filteredAccelX) < 0.02 && Math.abs(filteredAccelY) < 0.02) {
            velocityX = 0;
            velocityY = 0;
        }
    }
}
