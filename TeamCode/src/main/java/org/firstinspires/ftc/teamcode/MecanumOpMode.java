package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumMovement;

@TeleOp(name = "Mecanum Testing", group = "Testing")
public class MecanumOpMode extends OpMode {

    private MecanumMovement mecanumController = new MecanumMovement();

    @Override
    public void init() {
        mecanumController.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        mecanumController.driveFieldRelative(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }
}
