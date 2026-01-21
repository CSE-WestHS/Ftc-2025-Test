package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.GyroInterface;
import org.firstinspires.ftc.teamcode.mechanisms.VisionInterface;
import org.firstinspires.ftc.teamcode.util.DriveIMULocalizer;

public class Constants {
    public static GyroInterface gyroInterface = new GyroInterface();
    public static VisionInterface visionInterface = new VisionInterface();
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5) // put real mass in kg later
            .forwardZeroPowerAcceleration(-107.132)
            .lateralZeroPowerAcceleration(-169.632)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.001, 0.0, 0.0, 0.3))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0.0, 0.0, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0001, 0.0, 0.0, 0.0, 0.1));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(50.857)
            .yVelocity(43.490);


    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(15)
            .robotLength(18)
            .forwardTicksToInches(0.00818)
            .strafeTicksToInches(0.00806);

    public static Follower createFollower(HardwareMap hardwareMap) {
        gyroInterface.init(hardwareMap);
        visionInterface.init(hardwareMap);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new DriveIMULocalizer(hardwareMap, gyroInterface, localizerConstants, visionInterface))
                .build();
    }
}
