package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Graph")
@Configurable
@Disabled
public class TestGraph extends OpMode {
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private final ElapsedTime timer = new ElapsedTime();

    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;

    @Override
    public void init() {
        timer.reset();

        //graphManager.getConfig().invoke().setGraphUpdateInterval(50);

        updateSignals();
    }

    @Override
    public void loop() {
        updateSignals();
    }

    @SuppressLint("DefaultLocale")
    private void updateSignals() {
        double t = timer.seconds();
        sinVariable = Math.sin(t);
        cosVariable = Math.cos(t);

        panelsTelemetry.addLine(String.format("sin = %f, cos = %f, const = %f", sinVariable, sinVariable, constVariable));

        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("const", constVariable);
        panelsTelemetry.update(telemetry);
    }
}