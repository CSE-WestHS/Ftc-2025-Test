package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallMovement {
    // Rotate intake
    // Motors spin up until at speed
    // Servo rotates to allow ball through
    // Servo closes until motor is at speed again

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx intake;
    public static double percentageError = 0.03;

    public static double closedPos = 0.0;
    public static double openPos = 0.28;

    public static double p = 11.0;
    public static double i = 0.1;
    public static double d = 1.0;
    public static double f = 12.5;
    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Telemetry telemetry;

    //private Servo ballStopper;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //ballStopper = hardwareMap.get(Servo.class, "ball_stopper");

        //ballStopper.setDirection(Servo.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f)); //new PIDFCoefficients(16, 18, 1, 0));
    }

    public void setVelocity(double goalVelocity) {
        launcher.setVelocity(goalVelocity);
        launcher2.setVelocity(goalVelocity);
    }

    public void launch(double goalVelocity) {
        launcher.setVelocity(goalVelocity);
        launcher2.setVelocity(goalVelocity);

        double launcherVel = (launcher.getVelocity()*60)/28;
        if (launcherVel > goalVelocity*(1-percentageError) && launcherVel < goalVelocity*(1+percentageError)) {
            //ballStopper.setPosition(openPos);
            telemetry.addLine("Ball can shoot");
        } else {
            //ballStopper.setPosition(closedPos);
        }
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    /**
     * Calculates the required initial velocity (v0) to reach a specific
     * coordinate (d, h) given a launch angle.
     *
     * @param d Horizontal distance to the target (inches)
     * @param h Vertical height of the target (inches)
     * @param theta Launch angle (degrees)
     * @return The required initial speed in inches per second, or NaN if the target is unreachable at that angle.
     */
    public static double calculateApproxSpeed(double d, double h, double theta) {
        double g = 386.09; // Acceleration due to gravity (in/s^2)
        double numerator = g*(d*d); // gd^2
        double angle = Math.toRadians(theta); // Convert to radians
        double denominator = 2 * (Math.cos(angle)*Math.cos(angle)) * (d * Math.tan(angle)-h);

        if (denominator == 0) {
            return Double.NaN;
        }

        double vIdeal = Math.sqrt(numerator / denominator);

        // --- Drag Compensation ---
        // k represents energy lost to air resistance per inch.
        // If shots fall short, increase k. If they go long, decrease k.
        double k = 0.0015;
        return vIdeal * (1 + (k * d));
    }
}
