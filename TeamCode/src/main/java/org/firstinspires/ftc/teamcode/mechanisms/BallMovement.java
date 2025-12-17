package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallMovement {
    // Rotate intake
    // Motors spin up until at speed
    // Servo rotates to allow ball through
    // Servo closes until motor is at speed again

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx intake;

    public static int goalVelocity = 1000;
    public static double percentageError = 0.03;

    public static double closedPos = 0.0;
    public static double openPos = 0.28;

    public static double p = 10;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 12.5;
    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Servo ballStopper;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        ballStopper = hardwareMap.get(Servo.class, "ball_stopper");

        ballStopper.setDirection(Servo.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
    }
}
