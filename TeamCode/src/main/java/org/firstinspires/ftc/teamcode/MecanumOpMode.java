package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMovement;
import org.firstinspires.ftc.teamcode.mechanisms.VisionInterface;
import org.firstinspires.ftc.teamcode.util.EkfPoseEstimator;

@TeleOp(name = "Mecanum Testing", group = "Testing")
public class MecanumOpMode extends OpMode {

    private MecanumMovement mecanumController = new MecanumMovement();

    private VisionInterface visionInterface = new VisionInterface();

    private EkfPoseEstimator PoseEstimator;

    @Override
    public void init() {
        mecanumController.init(hardwareMap, telemetry);
        visionInterface.init(hardwareMap);
        PoseEstimator = new EkfPoseEstimator(0, 0, 0, 18, 18);
    }

    @Override
    public void loop() {
        double[] driveDistances = mecanumController.getDriveDistances();
        PoseEstimator.predict(driveDistances[1], driveDistances[0], driveDistances[3], driveDistances[2]);
        if (gamepad1.left_bumper) {
            mecanumController.driveFacingPoint(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.RADIANS, 0),
                    PoseEstimator
            );
        } else {
            mecanumController.driveFieldRelative(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }
    }
}
