package org.firstinspires.ftc.teamcode.util;

public class EkfPoseEstimator {
    // State: [x, y, theta]
    private double[] x = new double[3];

    // Covariance matrix (3x3)
    private double[][] P = new double[3][3];

    // Process noise covariance Q (3x3)
    // TUNE BASED ON TRUST FOR DIFFERENT SENSORS
    private double[][] Q = new double[3][3];

    // Measurement noise for heading R_theta (1x1)
    private double R_theta = 0.02 * 0.02; // variance (rad^2) - should be tuned

    // Measurement noise for vision position R_pos (2x2)
    private double[][] R_pos = new double[2][2];

    // Robot dimensions (meters)
    private final double wheelbaseLength;
    private final double wheelbaseWidth;

    /**
     * Initialize the state and covariance matrix.
     * @param initialX - pedro coordinates
     * @param initialY - pedro coordinates
     * @param initialTheta - radians
     * @param wheelbaseLength - meters
     * @param wheelbaseWidth - meters
     */
    public EkfPoseEstimator(double initialX, double initialY, double initialTheta, double wheelbaseLength, double wheelbaseWidth) {
        this.wheelbaseLength = wheelbaseLength;
        this.wheelbaseWidth = wheelbaseWidth;

        // Initialize state
        x[0] = initialX;
        x[1] = initialY;
        x[2] = wrapRad(initialTheta);

        // Initialize covariance with small uncertainty
        P[0][0] = 0.1;
        P[1][1] = 0.1;
        P[2][2] = 0.05;

        // Default Q: small
        Q[0][0] = 1e-4;
        Q[1][1] = 1e-4;
        Q[2][2] = 1e-4;

        // Vision pos noise default (meters^2)
        R_pos[0][0] = 0.02 * 0.02;
        R_pos[1][1] = 0.02 * 0.02;
    }

    /* Public setters for tuning */

    /**
     * Set the process noise covariance Q.
     * @param qx - variance for x
     * @param qy - variance for y
     * @param qtheta - variance for theta
     */
    public void setProcessNoise(double qx, double qy, double qtheta) {
        Q[0][0] = qx;
        Q[1][1] = qy;
        Q[2][2] = qtheta;
    }

    /**
     * Set the imu heading measurement noise R_theta.
     * @param varRad2 - variance (rad^2)
     */
    public void setImuNoiseVariance(double varRad2) { R_theta = varRad2; }

    /**
     * Set the vision position measurement noise R_pos.
     * @param varX - variance for x
     * @param varY - variance for y
     */
    public void setVisionNoise(double varX, double varY) {
        R_pos[0][0] = varX; R_pos[1][1] = varY;
    }

    /* Prediction step using differential drive odometry */
    // dl, dr are distances travelled by left and right wheels since last update (meters).
    // This function updates x and P.

    /**
     * Predict the state and covariance using differential drive odometry.
     * @param dFL - distance travelled by left front wheel (meters)
     * @param dFR - distance travelled by right front wheel (meters)
     * @param dBL - distance travelled by left back wheel (meters)
     * @param dBR - distance travelled by right back wheel (meters)
     */
    public void predict(double dFL, double dFR, double dBL, double dBR) {
        double L = wheelbaseLength;
        double W = wheelbaseWidth;

        // Compute forward kinematics:
        double dx_robot = (dFL + dFR + dBL + dBR) / 4.0;
        double dy_robot = (-dFL + dFR + dBL - dBR) / 4.0;
        double dTheta = (-dFL + dFR - dBL + dBR) / (4.0 * (L/2 + W/2));


        // ----- 2. Convert to field-relative deltas -----
        double thetaMid = x[2] + dTheta / 2.0;
        double dx_field = dx_robot * Math.cos(thetaMid) - dy_robot * Math.sin(thetaMid);
        double dy_field = dx_robot * Math.sin(thetaMid) + dy_robot * Math.cos(thetaMid);

        // ----- 3. Update state -----
        x[0] += dx_field;
        x[1] += dy_field;
        x[2] = wrapRad(x[2] + dTheta);

        // ----- 4. Compute Jacobian F -----
        double[][] F = new double[3][3];
        F[0][0] = 1;
        F[0][1] = 0;
        F[0][2] = -dx_robot * Math.sin(thetaMid) - dy_robot * Math.cos(thetaMid);

        F[1][0] = 0;
        F[1][1] = 1;
        F[1][2] = dx_robot * Math.cos(thetaMid) - dy_robot * Math.sin(thetaMid);

        F[2][0] = 0;
        F[2][1] = 0;
        F[2][2] = 1;

        // ----- 5. Update covariance -----
        P = add(multiply(F, multiply(P, transpose(F))), Q);
    }

    /* Update step using IMU heading measurement (absolute heading in radians) */

    /**
     * Update the state and covariance using IMU heading measurement.
     * @param ztheta - measured heading (radians)
     */
    public void updateWithImuHeading(double ztheta) {
        ztheta = wrapRad(ztheta);

        // Measurement model: h(x) = theta
        // H = [0 0 1]
        double[] H = new double[]{0.0, 0.0, 1.0};

        // Innovation y = z - h(x)
        double y = angleDiff(ztheta, x[2]); // ensure smallest angle difference

        // S = H P H^T + R_theta (scalar)
        double s = H[0]*(P[0][0]*H[0] + P[0][1]*H[1] + P[0][2]*H[2])
                 + H[1]*(P[1][0]*H[0] + P[1][1]*H[1] + P[1][2]*H[2])
                 + H[2]*(P[2][0]*H[0] + P[2][1]*H[1] + P[2][2]*H[2])
                 + R_theta;

        // K = P H^T / S -> 3x1 vector
        double[] K = new double[3];
        for (int i=0; i<3; i++) {
            K[i] = (P[i][0]*H[0] + P[i][1]*H[1] + P[i][2]*H[2]) / s;
        }

        // Update state: x = x + K * y
        for (int i=0; i<3; i++) x[i] += K[i] * y;

        x[2] = wrapRad(x[2]);

        // Update covariance: P = (I - KH) P
        double[][] KH = new double[3][3];
        for (int i=0; i<3; i++) for (int j=0; j<3; j++) KH[i][j] = K[i] * ((j==2)?1.0:0.0);

        double[][] I = identity(3);
        double[][] IminusKH = subtract(I, KH);
        P = multiply(IminusKH, P);
    }

    /* Update step using vision absolute pose measurement (x,y) */

    /**
     * Update the state and covariance using vision absolute pose measurement.
     * @param zx - measured x (meters)
     * @param zy - measured y (meters)
     */
    public void updateWithVisionPose(double zx, double zy) {
        // Measurement h(x) = [x; y]
        // H = [1 0 0; 0 1 0] (2x3)

        double[][] H = new double[][]{
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        };

        // Innovation y = z - h(x)
        double[] y = new double[]{ zx - x[0], zy - x[1] };

        // S = H P H^T + R_pos (2x2)
        double[][] HP = multiply(H, P); // (2x3)*(2x3) = 2x3
        double[][] HPHt = multiply(HP, transpose(H)); // 2x2
        double[][] S = add(HPHt, R_pos); // 2x2

        // K = P H^T S^{-1} -> (3x3)*(3x2)*(2x2) = 3x2
        double[][] PHt = multiply(P, transpose(H)); // 3x2
        double[][] Sinv = inverse2x2(S);
        double[][] K = multiply(PHt, Sinv); // 3x2

        // Update x = x + K * y (3x2 * 2x1)
        double[] Ky = new double[3];
        for (int i=0; i<3; i++) Ky[i] = K[i][0]*y[0] + K[i][1]*y[1];
        for (int i=0; i<3; i++) x[i] += Ky[i];

        x[2] = wrapRad(x[2]);

        // Update P = (I - K H) P
        double[][] KH = multiply(K, H); // 3x3
        double[][] I = identity(3);
        double[][] IminusKH = subtract(I, KH);
        P = multiply(IminusKH, P);
    }

    /* Getters */

    public double getX() { return -x[0]*39.37; }
    public double getY() { return -x[1]*39.37; }
    public double getTheta() { return x[2]; }
    public double[][] getCovariance() { return copy(P); }

    /* Matrix helpers */
    /** Returns nxn identity matrix. */
    private static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i=0; i<n; i++) I[i][i] = 1.0;
        return I;
    }

    /** Transposes a matrix. */
    private static double[][] transpose(double[][] A) {
        int r = A.length, c = A[0].length;
        double[][] T = new double[c][r];
        for (int i=0; i<r; i++) for (int j=0; j<c; j++) T[j][i] = A[i][j];
        return T;
    }

    /** Matrix multiplication A * B. */
    private static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, m = A[0].length, c = B[0].length;
        double[][] C = new double[r][c];
        for (int i=0; i<r; i++) {
            for (int j=0; j<c; j++) {
                double s = 0;
                for (int k=0; k<m; k++) s += A[i][k]*B[k][j];
                C[i][j] = s;
            }
        }
        return C;
    }

    /** Matrix-vector multiplication A * v. */
    private static double[] multiply(double[][] A, double[] v) {
        int r = A.length, c = A[0].length;
        double[] out = new double[r];
        for (int i=0; i<r; i++) {
            double s = 0;
            for (int j=0; j<c; j++) s += A[i][j]*v[j];
            out[i] = s;
        }
        return out;
    }

    /** Matrix addition A + B. */
    private static double[][] add(double[][] A, double[][] B) {
        int r = A.length, c = A[0].length;
        double[][] C = new double[r][c];
        for (int i=0; i<r; i++) for (int j=0; j<c; j++) C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    /** Matrix subtraction A - B. */
    private static double[][] subtract(double[][] A, double[][] B) {
        int r = A.length, c = A[0].length;
        double[][] C = new double[r][c];
        for (int i=0; i<r; i++) for (int j=0; j<c; j++) C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    /** Matrix-scalar multiplication A * scalar. */
    private static double[][] multiply(double[][] A, double scalar) {
        int r = A.length, c = A[0].length;
        double[][] C = new double[r][c];
        for (int i=0; i<r; i++) for (int j=0; j<c; j++) C[i][j] = A[i][j] * scalar;
        return C;
    }

    /** Deep copy of a matrix. */
    private static double[][] copy(double[][] A) {
        int r = A.length, c = A[0].length;
        double[][] C = new double[r][c];
        for (int i=0; i<r; i++) for (int j=0; j<c; j++) C[i][j] = A[i][j];
        return C;
    }

    /**
     * Inverse of a 2x2 matrix:
     * [a b]
     * [c d]
     *
     * @param m 2x2 matrix
     * @return inverse of m
     */
    private static double[][] inverse2x2(double[][] m) {
        double a = m[0][0], b = m[0][1], c = m[1][0], d = m[1][1];
        double det = a*d - b*c;
        if (Math.abs(det) < 1e-12) det = 1e-12; // prevent divide by 0
        double[][] inv = new double[2][2];
        inv[0][0] = d / det;
        inv[0][1] = -b / det;
        inv[1][0] = -c / det;
        inv[1][1] = a / det;
        return inv;
    }

    /** Wraps angle to (-pi, +pi) */
    private static double wrapRad(double ang) {
        while (ang <= -Math.PI) ang += 2.0*Math.PI;
        while (ang > Math.PI) ang -= 2.0*Math.PI;
        return ang;
    }

    /**
     * Computes the smallest signed angle difference between b and a.
     * Useful for heading innovations:
     *     angleDiff(measured, predicted)
     */
    private static double angleDiff(double a, double b) {
        double d = a - b;
        return wrapRad(d);
    }
}
