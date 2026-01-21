package org.firstinspires.ftc.teamcode.util;

public class Conversions {
    public static double RPMToTicks(double RPM) {
        return (RPM * 28) / 60;
    }

    public static double TicksToRPM(double ticks) {
        return (ticks * 60) / 28;
    }
}
