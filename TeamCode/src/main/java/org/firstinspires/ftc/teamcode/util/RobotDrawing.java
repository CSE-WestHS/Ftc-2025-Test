package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;

public class RobotDrawing {
    /**
     * Draw a rotated rectangle representing the robot on Panels Field using fieldManager.
     *
     * @param cx        center X (field coords, inches)
     * @param cy        center Y (field coords, inches)
     * @param heading   yaw in radians (field coord frame)
     * @param width     width (side-to-side) in inches
     * @param length    length (front-to-back) in inches
     * @param colorHex  CSS hex color string, e.g. "#FF4081"
     */
    public void drawRobotRectOnPanels(double cx, double cy, double heading,
                                      double width, double length, String colorHex,
                                      FieldManager fieldManager) {
        // half extents in local robot frame
        double hw = width / 2;
        double hl = length / 2;

        // compute points in local robot frame
        double[][] corners = {{-hw, hl}, {hw, hl}, {hw, -hl}, {-hw, -hl}};

        for (int i = 0; i < corners.length; i++) {
            corners[i] = rotateCoordinate(corners[i], heading);
        }

        for (int i = 0; i < corners.length; i++) {
            corners[i][0] += cx;
            corners[i][1] += cy;
        }

        fieldManager.setStyle("", colorHex, 5);

        for (int i = 0; i < corners.length; i++) {
            int next = (i + 1) % corners.length;
            fieldManager.moveCursor(corners[i][0], corners[i][1]);
            fieldManager.line(corners[next][0], corners[next][1]);
        }

        double[] headingLine = rotateCoordinate(new double[]{0, hl}, heading);

        fieldManager.moveCursor(cx, cy);
        fieldManager.line(headingLine[0] + cx, headingLine[1] + cy);

        // commit update to Panels
        fieldManager.update();
    }

    private double[] rotateCoordinate(double[] xy, double heading) {
        double x = xy[0];
        double y = xy[1];

        x = x*Math.cos(heading) - y*Math.sin(heading);
        y = x*Math.sin(heading) + y*Math.cos(heading);

        return new double[]{x, y};
    }
}