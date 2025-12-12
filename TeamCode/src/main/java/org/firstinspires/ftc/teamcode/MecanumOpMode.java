package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.GyroInterface;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumMovement;
import org.firstinspires.ftc.teamcode.mechanisms.VisionInterface;
import org.firstinspires.ftc.teamcode.util.EkfPoseEstimator;
import org.firstinspires.ftc.teamcode.util.RobotDrawing;

@TeleOp(name = "Mecanum Testing", group = "Testing")
public class MecanumOpMode extends OpMode {

    private final MecanumMovement mecanumController = new MecanumMovement();

    private final VisionInterface visionInterface = new VisionInterface();

    private final RobotDrawing robotDrawing = new RobotDrawing();

    private final GyroInterface gyroInterface = new GyroInterface();

    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();

    private EkfPoseEstimator PoseEstimator;

    @Override
    public void init() {
        mecanumController.init(hardwareMap, telemetry);
        visionInterface.init(hardwareMap, telemetry, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0));
        gyroInterface.init(hardwareMap, telemetry);
        fieldManager.init();
        PoseEstimator = new EkfPoseEstimator(0, 0, 0, 18, 18);
    }

    @Override
    public void loop() {
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
        visionInterface.update();
        if (visionInterface.canSeeAprilTag()) {
            PoseEstimator.updateWithVisionPose(visionInterface.getX(), visionInterface.getY());
            gyroInterface.alignHeadingWithVision(new Rotation2d(PoseEstimator.getTheta()));
        }
        PoseEstimator.updateWithImuHeading(gyroInterface.getHeading().getRadians());
        fieldManager.setStyle("#FF4081", "#FF4081", 5);
        fieldManager.moveCursor(PoseEstimator.getX(),PoseEstimator.getY());
        fieldManager.circle(10);
        fieldManager.update();
        //robotDrawing.drawRobotRectOnPanels(PoseEstimator.getX(), PoseEstimator.getY(), PoseEstimator.getTheta(), 18, 18, "#3F51B5", fieldManager);
        double[] driveDistances = mecanumController.getDriveDistances();
        PoseEstimator.predict(driveDistances[1], driveDistances[0], driveDistances[3], driveDistances[2]);
        telemetry.addData("X", PoseEstimator.getX());
        telemetry.addData("Y", PoseEstimator.getY());
        telemetry.addData("Theta", PoseEstimator.getTheta());
        telemetry.update();
    }
}
