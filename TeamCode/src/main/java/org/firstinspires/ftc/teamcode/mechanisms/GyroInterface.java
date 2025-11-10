package org.firstinspires.ftc.teamcode.mechanisms;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GyroInterface {
    private AHRS navx;
    private navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private boolean calibration_complete = false;

    private double yawOffset = 0.0;

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

    public void alignHeadingWithVision(double visionFieldAngle) {
        double currentNavxAngle = navx.getYaw();

        yawOffset = visionFieldAngle - currentNavxAngle;
    }

    public void resetInternalYawToZero() {
        navx.zeroYaw(); // Resets the *internal* reference to 0
        yawOffset = 0.0; // Reset our software offset too if zeroYaw() is used for a full reset.
    }

    public void setYawOffset(double yawOffset) {
        this.yawOffset = yawOffset;
    }
}
