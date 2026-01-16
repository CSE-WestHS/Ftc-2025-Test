package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.BallMovement;
import org.firstinspires.ftc.teamcode.mechanisms.GyroInterface;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMovement;
import org.firstinspires.ftc.teamcode.mechanisms.VisionInterface;
import org.firstinspires.ftc.teamcode.util.EkfPoseEstimator;

@TeleOp(name = "Ultimate Teleop", group = "Final Bot")
@Configurable
public class UltimateTeleop extends OpMode {
    private final MecanumMovement mecanumController = new MecanumMovement();

    private final BallMovement ballMovement = new BallMovement();

    private final VisionInterface visionInterface = new VisionInterface();

    //private final RobotDrawing robotDrawing = new RobotDrawing();

    private final GyroInterface gyroInterface = new GyroInterface();

    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();

    private EkfPoseEstimator PoseEstimator;

    private Pose2D goalPos;

    public static int goalVelocity = 4000;

    @Override
    public void init() {
        mecanumController.init(hardwareMap, telemetry);
        visionInterface.init(hardwareMap);
        gyroInterface.init(hardwareMap);
        ballMovement.init(hardwareMap, telemetry);
        fieldManager.init();
        PoseEstimator = new EkfPoseEstimator(0, 0, 0, 18, 18);
        goalPos = new Pose2D(DistanceUnit.INCH, 140, 140, AngleUnit.RADIANS, 0); // Replace with logic to get preferred goal or whatever
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            mecanumController.driveFacingPoint(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.RADIANS, 0),
                    PoseEstimator,
                    gyroInterface.getHeading()
            );
        } else if (gamepad1.dpad_up) {
            mecanumController.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        } else {
            mecanumController.driveFieldRelative(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gyroInterface.getHeading()
            );
        }

        if (gamepad1.right_bumper) {
            ballMovement.launch((double) (goalVelocity * 28) / 60);
        } else if (gamepad1.y) {
            ballMovement.setVelocity((double) (goalVelocity * 28) / 60);
        } else {
            ballMovement.setVelocity(0);
        }

        if (gamepad1.left_bumper) {
            ballMovement.intake(1.0);
        } else if (gamepad1.left_trigger > 0.5) {
            ballMovement.intake(-1.0);
        } else {
            ballMovement.intake(0.0);
        }

        visionInterface.update();
        if (visionInterface.canSeeAprilTag()) {
            PoseEstimator.updateWithVisionPose(visionInterface.getX(), visionInterface.getY());
            gyroInterface.alignHeadingWithVision(new Rotation2d(PoseEstimator.getTheta()));
        }
        PoseEstimator.updateWithImuHeading(gyroInterface.getHeading().getRadians());
        fieldManager.setStyle("#FF4081", "#FF4081", 5);
        fieldManager.moveCursor(PoseEstimator.getX(),PoseEstimator.getY());
        fieldManager.circle(10);
        double x = PoseEstimator.getX() + 10 * Math.cos(PoseEstimator.getTheta());
        double y = PoseEstimator.getY() + 10 * Math.sin(PoseEstimator.getTheta());
        fieldManager.setStyle("00FF00", "00FF00", 5);
        fieldManager.line(x, y);
        fieldManager.setStyle("0000FF", "0000FF", 5);
        fieldManager.line(goalPos.getX(DistanceUnit.INCH),goalPos.getY(DistanceUnit.INCH));
        fieldManager.update();
        //robotDrawing.drawRobotRectOnPanels(PoseEstimator.getX(), PoseEstimator.getY(), PoseEstimator.getTheta(), 18, 18, "#3F51B5", fieldManager);
        double[] driveDistances = mecanumController.getDriveDistances();
        PoseEstimator.predict(driveDistances[1], driveDistances[0], driveDistances[3], driveDistances[2]);
        telemetry.addData("X", PoseEstimator.getX());
        telemetry.addData("Y", PoseEstimator.getY());
        telemetry.addData("Theta", PoseEstimator.getTheta());
        telemetry.addData("navx heading", gyroInterface.getHeading().getRadians());
        telemetry.update();
    }
}
