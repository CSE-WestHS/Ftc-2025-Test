package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
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

@TeleOp(name = "Ultimate Teleop", group = "Final Bot")
@Configurable
public class UltimateTeleop extends OpMode {
    private final MecanumMovement mecanumController = new MecanumMovement();

    private final BallMovement ballMovement = new BallMovement();

    private final VisionInterface visionInterface = new VisionInterface();

    //private final RobotDrawing robotDrawing = new RobotDrawing();

    private final GyroInterface gyroInterface = new GyroInterface();

    private final FieldManager fieldManager = PanelsField.INSTANCE.getField();

    //private EkfPoseEstimator PoseEstimator;

    private Pose2D goalPos;

    private Pose2d m_pose;

    public static int goalVelocity = 4000;

    Translation2d m_frontLeftLocation =
            new Translation2d(4.75, 6.25);
    Translation2d m_frontRightLocation =
            new Translation2d(4.75, -6.25);
    Translation2d m_backLeftLocation =
            new Translation2d(-4.75, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-4.75, -0.381);

    MecanumDriveKinematics m_kinematics;

    MecanumDriveOdometry m_odometry;

    public static double headingP = 0.0;
    public static double headingI = 0.0;
    public static double headingD = 0.0;
    public static double headingF = 1.0;
    public static double heading2P = 0.0;
    public static double heading2I = 0.0;
    public static double heading2D = 0.0;
    public static double heading2F = 1.0;

    private enum Alliance {
        RED,
        BLUE;
    }

    private Alliance alliance = Alliance.RED;


    @Override
    public void init() {
        mecanumController.init(hardwareMap, telemetry);
        visionInterface.init(hardwareMap);
        gyroInterface.init(hardwareMap);
        ballMovement.init(hardwareMap, telemetry);
        fieldManager.init();
        //PoseEstimator = new EkfPoseEstimator(0, 0, 0, 18, 15);
        goalPos = new Pose2D(DistanceUnit.INCH, 140, 140, AngleUnit.RADIANS, 0); // Replace with logic to get preferred goal or whatever
        m_kinematics = new MecanumDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );
        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, gyroInterface.getHeading(),
                        new Pose2d(72, 72, new Rotation2d())
                );
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press X for Blue or B for Red");
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            alliance = Alliance.RED;
        }
        telemetry.addData("Alliance:", alliance);
        telemetry.update();
    }

    public void periodic() {
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        double[] driveVelocities = mecanumController.getDriveVelocities();

        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        driveVelocities[1], driveVelocities[0],
                        driveVelocities[3], driveVelocities[2]
                );

        // Get my gyro angle.
        Rotation2d gyroAngle = gyroInterface.getHeading();

        // Update the pose
        m_pose = m_odometry.updateWithTime(System.currentTimeMillis(), gyroAngle, wheelSpeeds);
    }

    @Override
    public void loop() {
        periodic(); // Update position
        visionInterface.update();
        if (visionInterface.canSeeAprilTag()) {
            telemetry.addLine("Can see april tag rn!!!");
            gyroInterface.alignHeadingWithVision(new Rotation2d(visionInterface.getHeading()));
            m_odometry.resetPosition(visionInterface.getRobotPosition(), gyroInterface.getHeading());
        }

        mecanumController.setHeadingPIDF(headingP, headingI, headingD, headingF);
        mecanumController.setHeadingPIDF(heading2P, heading2I, heading2D, heading2F);
        if (gamepad1.right_trigger > 0.2) {
            mecanumController.driveFacingPoint(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    new Pose2D(DistanceUnit.INCH, 10, 10, AngleUnit.RADIANS, 0),
                    m_pose,
                    gyroInterface.getHeading(),
                    (alliance == Alliance.RED)
            );
            ballMovement.setVelocity(goalVelocity);
            if (gamepad1.right_trigger > 0.8) {
                ballMovement.launch(goalVelocity);
            }
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
                    gyroInterface.getHeading(),
                    (alliance == Alliance.RED)
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
        //PoseEstimator.updateWithImuHeading(gyroInterface.getHeading().getRadians());
        fieldManager.setStyle("#FF4081", "#FF4081", 5);
        //fieldManager.moveCursor(PoseEstimator.getX(),PoseEstimator.getY());
        fieldManager.moveCursor(m_pose.getX(), m_pose.getY());
        fieldManager.circle(10);
        //double x = PoseEstimator.getX() + 10 * Math.cos(PoseEstimator.getTheta());
        //double y = PoseEstimator.getY() + 10 * Math.sin(PoseEstimator.getTheta());
        double x = m_pose.getX() + 10 * Math.cos(m_pose.getRotation().getRadians());
        double y = m_pose.getY() + 10 * Math.sin(m_pose.getRotation().getRadians());
        fieldManager.setStyle("00FF00", "00FF00", 5);
        fieldManager.line(x, y);
        fieldManager.setStyle("0000FF", "0000FF", 5);
        fieldManager.line(goalPos.getX(DistanceUnit.INCH),goalPos.getY(DistanceUnit.INCH));
        fieldManager.update();
        //robotDrawing.drawRobotRectOnPanels(PoseEstimator.getX(), PoseEstimator.getY(), PoseEstimator.getTheta(), 18, 18, "#3F51B5", fieldManager);
        //double[] driveDistances = mecanumController.getDriveDistances();
        //PoseEstimator.predict(driveDistances[1], driveDistances[0], driveDistances[3], driveDistances[2]);
        telemetry.addData("X", m_pose.getX());
        telemetry.addData("Y", m_pose.getY());
        telemetry.addData("Theta", m_pose.getHeading());
        telemetry.addData("navx heading", gyroInterface.getHeading().getRadians());

        telemetry.update();
    }
}
