package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Testing", group = "Testing")
//@Disabled
@Configurable
public class ShooterTestTeleop extends OpMode {
    private DcMotorEx launcher;
    private DcMotorEx launcher2;

    private DcMotorEx intake;

    public static int goalVelocity = 1000;
    public static double percentageError = 0.03;

    public static double closedPos = 0.0;
    public static double openPos = 0.28;

    public static double p = 11.0;
    public static double i = 0.1;
    public static double d = 1.0;
    public static double f = 12.5;
    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    //private Servo ballStopper;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //ballStopper = hardwareMap.get(Servo.class, "ball_stopper");

        //ballStopper.setDirection(Servo.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);

        intake.setZeroPowerBehavior(BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));

        //graphManager.getConfig().invoke().setGraphUpdateInterval(50);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));

        if (gamepad1.right_bumper) {
            launcher.setVelocity((double) (goalVelocity * 28) /60);
            launcher2.setVelocity((double) (goalVelocity * 28) /60);
        } else {
            launcher.setVelocity(0);
            launcher2.setVelocity(0);
        }

        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.5) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }
        double launcherVel = (launcher.getVelocity()*60)/28;

        if (launcherVel > goalVelocity*(1-percentageError) && launcherVel < goalVelocity*(1+percentageError)) {
            //ballStopper.setPosition(openPos);
        } else {
            //ballStopper.setPosition(closedPos);
        }

        panelsTelemetry.addData("goalVelocity", goalVelocity);//(5000*28)/60); // *28 and / 60 to turn from rpm to tps
        panelsTelemetry.addData("realVelocity", (launcher.getVelocity()*60)/28);
        panelsTelemetry.addData("realVelocity2", (launcher2.getVelocity()*60)/28);

        panelsTelemetry.addLine(String.format("goalVelocity = %d, realVelocity = %f, realVelocity2 = %f", goalVelocity, (launcher.getVelocity()*60)/28, (launcher2.getVelocity()*60)/28));//(5000*28)/60); // *28 and / 60 to turn from rpm to tps

        panelsTelemetry.addData("p", new PIDFCoefficients(p, i, d, f).p);
        panelsTelemetry.addData("i", new PIDFCoefficients(p, i, d, f).i);
        panelsTelemetry.addData("d", new PIDFCoefficients(p, i, d, f).d);
        panelsTelemetry.addData("f", new PIDFCoefficients(p, i, d, f).f);

        panelsTelemetry.update(telemetry);
    }
}
