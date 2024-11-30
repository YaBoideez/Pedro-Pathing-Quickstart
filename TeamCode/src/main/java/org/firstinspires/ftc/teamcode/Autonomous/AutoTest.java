package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

/**
 * This autonomous OpMode runs a custom path using Pedro Pathing.
 */
@Config
@Autonomous(name = "CustomPathAuto", group = "Autonomous Pathing Tuning")
public class AutoTest extends OpMode {
    private Follower follower;
    private Path curvePath;

    /**
     * Initializes the follower and creates a simple curve path.
     */
    @Override
    public void init() {
        // Initialize the Follower
        follower = new Follower(hardwareMap);

        // Define points for the Bezier curve
        Point start = new Point(0, 0, Point.CARTESIAN);           // Starting point
        Point mid = new Point(20, 20, Point.CARTESIAN);           // Control point for curve
        Point end = new Point(40, 0, Point.CARTESIAN);            // Endpoint

        // Create the curve path
        curvePath = new Path(
                new BezierCurve(start, mid, end)                     // Smooth curve
        );
        //curvePath.setConstantHeadingInterpolation(0);   sets constant heading on path

        // Instruct the follower to follow the curve path
        follower.followPath(curvePath);

        // Set up telemetry
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(this.telemetry, dashboardTelemetry);
        telemetry.addLine("Curve path initialized. Robot will move along the curve.");
        telemetry.update();
    }

    /**
     * Runs the OpMode, updating the follower and toggling between forward and backward paths.
     */
    @Override
    public void loop() {
        // Update the follower
        follower.update();

        // Stop once the path is complete
        if (!follower.isBusy()) {
            telemetry.addLine("Path complete.");
            telemetry.update();
            requestOpModeStop();
        }

        // Debug telemetry
        follower.telemetryDebug(telemetry);
    }
}