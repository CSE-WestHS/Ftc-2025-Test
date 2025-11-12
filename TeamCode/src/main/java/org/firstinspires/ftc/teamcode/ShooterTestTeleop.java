package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Shooter Testing", group = "Testing")
//@disabled
public class ShooterTestTeleop extends OpMode {
    private DcMotorEx launcher;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10)); //new PIDFCoefficients(16, 18, 1, 0));


    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            launcher.setVelocity(5000, AngleUnit.DEGREES);
        } else {
            launcher.setVelocity(0);
        }

        packet.put("goalVelocity", 5000);//(5000*28)/60); // *28 and / 60 to turn from rpm to tps
        packet.put("realVelocity", (launcher.getVelocity()*60)/28);

        dashboard.sendTelemetryPacket(packet);
    }
}
