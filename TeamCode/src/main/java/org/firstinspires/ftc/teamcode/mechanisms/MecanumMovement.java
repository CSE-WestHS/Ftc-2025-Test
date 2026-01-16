package org.firstinspires.ftc.teamcode.mechanisms;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.EkfPoseEstimator;

public class MecanumMovement {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    //private final GyroInterface navx = new GyroInterface();
    //private IMU imu;

    public static PIDController headingPID;

    Pose2D robotPosition;

    private double[] previousPos;

    public Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        previousPos = new double[]{0, 0, 0, 0};
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        this.telemetry = telemetry;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //navx.init(hardwareMap);
        //imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        //imu.initialize(new IMU.Parameters(RevOrientation));

        headingPID = new PIDController(1.2, 0, 0.05);  // P, I, D — adjust as needed

        // Optional: reduce output sensitivity
        headingPID.setIntegrationBounds(-0.3, 0.3);
        headingPID.setTolerance(Math.toRadians(1));  // 3° tolerance

    }

    public void drive(double forward, double strafe, double rotate) {
        forward *= Math.abs(forward);
        strafe *= Math.abs(strafe);
        rotate *= Math.abs(rotate);
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
    public void driveFieldRelative(double forward, double strafe, double rotate, Rotation2d heading) {

        Translation2d chassisSpeeds = new Translation2d(forward, strafe);

        chassisSpeeds.rotateBy(heading.times(-1));

        double newForward = forward * heading.getCos() + strafe * heading.getSin();
        double newStrafe = -forward * heading.getSin() + strafe * heading.getCos();


        /*double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta + Math.PI/2.0 - navx.getHeading().getRadians());



        //theta = AngleUnit.normalizeRadians(theta -
        //        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) + 1.57;

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);*/

        telemetry.addData("passed in heading", heading.getRadians());
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("chassisX", chassisSpeeds.getX());
        telemetry.addData("chassisY", chassisSpeeds.getY());
        telemetry.addData("newForward", newForward);
        telemetry.addData("newStrafe", newStrafe);

        this.drive(newForward, newStrafe, rotate);
    }

    /**
     *
     * @param position -- position to lock onto
     */
    public void driveFacingPoint(double forward, double strafe, Pose2D position, EkfPoseEstimator
        Pose, Rotation2d heading) {

        double goalHeadingR = Math.atan2(
                (position.getY(DistanceUnit.INCH) - Pose.getY()),
                (position.getX(DistanceUnit.INCH) - Pose.getX())
        );

        double currentHeading = heading.getRadians();
        double error = angleWrap(goalHeadingR - currentHeading);

        double turnPower = headingPID.calculate(error);

        turnPower = Math.max(-0.8, Math.min(0.8, turnPower));

        if (headingPID.atSetPoint()) {
            turnPower = 0;
        }

        driveFieldRelative(forward, strafe, turnPower, heading);
    }

    public double[] getDriveDistances() {
        double[] output = new double[4];

        // Capture current positions
        double currentFR = frontRightMotor.getCurrentPosition();
        double currentFL = frontLeftMotor.getCurrentPosition();
        double currentBR = backRightMotor.getCurrentPosition();
        double currentBL = backLeftMotor.getCurrentPosition();

        // Calculate the difference
        output[0] = currentFR - previousPos[0]; // Front Right Motor dist
        output[1] = currentFL - previousPos[1]; // Front Left Motor dist
        output[2] = currentBR - previousPos[2]; // Back Right Motor dist
        output[3] = currentBL - previousPos[3]; // Back Left Motor dist

        // Update previous positions
        previousPos[0] = currentFR;
        previousPos[1] = currentFL;
        previousPos[2] = currentBR;
        previousPos[3] = currentBL;

        // Scale by the meters per tick factor
        final double METERS_PER_TICK = 0.0006077;
        output[0] *= METERS_PER_TICK;
        output[1] *= METERS_PER_TICK;
        output[2] *= METERS_PER_TICK;
        output[3] *= METERS_PER_TICK;

        return output;
    }

}