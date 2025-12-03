package org.firstinspires.ftc.teamcode.mechanisms;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.EkfPoseEstimator;

public class MecanumMovement {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final GyroInterface navx = new GyroInterface();
    //private IMU imu;

    private PIDController headingPID;

    Pose2D robotPosition;

    private double[] previousPos;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        previousPos = new double[]{0, 0, 0, 0};
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        navx.init(hardwareMap, telemetry);
        //imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        //imu.initialize(new IMU.Parameters(RevOrientation));

        headingPID = new PIDController(1.2, 0, 0.05);  // P, I, D — adjust as needed

        // Optional: reduce output sensitivity
        headingPID.setIntegrationBounds(-0.3, 0.3);
        headingPID.setTolerance(Math.toRadians(3));  // 3° tolerance

    }

    public void drive(double forward, double strafe, double rotate) {
        forward *= forward;
        strafe *= strafe;
        rotate *= rotate;
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    /**
     *
     * @param forward -- gamepad.left_stick_y
     * @param strafe -- gamepad.left_stick_x
     * @param rotate -- gamepad.right_stick_x
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                navx.getHeading().getRadians());

        //theta = AngleUnit.normalizeRadians(theta -
        //        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) + 1.57;

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    /**
     *
     * @param position -- position to lock onto
     */
    public void driveFacingPoint(double forward, double strafe, Pose2D position, EkfPoseEstimator Pose) {

        double goalHeadingR = Math.atan2(
                (position.getY(DistanceUnit.METER) - Pose.getY()),
                (position.getX(DistanceUnit.METER) - Pose.getX())
        );

        double currentHeading = navx.getHeading().getRadians();
        double error = angleWrap(goalHeadingR - currentHeading);

        double turnPower = headingPID.calculate(error);

        turnPower = Math.max(-1, Math.min(1, turnPower));

        if (headingPID.atSetPoint()) {
            turnPower = 0;
        }

        driveFieldRelative(forward, strafe, turnPower);
    }

    public double[] getDriveDistances() {
        double[] output = new double[4];

        output[0] = previousPos[0] - frontRightMotor.getCurrentPosition();// Front Right Motor dist
        output[1] = previousPos[1] - frontLeftMotor.getCurrentPosition();// Front Left Motor dist
        output[2] = previousPos[2] - backRightMotor.getCurrentPosition();// Back Right Motor dist
        output[3] = previousPos[3] - backLeftMotor.getCurrentPosition();// Back Left Motor dist

        previousPos[0] = frontRightMotor.getCurrentPosition();
        previousPos[1] = frontLeftMotor.getCurrentPosition();
        previousPos[2] = backRightMotor.getCurrentPosition();
        previousPos[3] = backLeftMotor.getCurrentPosition();

        return output;
    }
}