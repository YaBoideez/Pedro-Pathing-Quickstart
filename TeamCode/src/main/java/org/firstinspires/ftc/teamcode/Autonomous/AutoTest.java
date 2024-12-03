package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name = "CustomPathAuto", group = "Autonomous Pathing Tuning")
public class AutoTest extends OpMode {
    private Follower follower;
    private PathChain initialPath;
    private PathChain loopingPath;
    private ElapsedTime timer = new ElapsedTime();

    private DcMotor Shoulder = null;
    private DcMotor Elbow = null;
    private DcMotor Arm_extenstion = null;
    private Servo Wrist = null;
    private Servo Gripper = null;

    private enum State {
        FOLLOW_INITIAL_PATH,
        FOLLOW_LOOPING_PATH,
        WAIT,
        MOVE_ARM,
        DONE
    }

    private State currentState = State.FOLLOW_INITIAL_PATH;
    private int loopCount = 0; // Track the number of iterations
    private static final int MAX_LOOP_COUNT = 3; // Number of times to repeat the loop

    /**
     * Initializes the follower and sets up paths for the autonomous sequence.
     */
    @Override
    public void init() {

        // Initialize motors
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");
        Arm_extenstion = hardwareMap.get(DcMotor.class, "Arm_extenstion");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Gripper = hardwareMap.get(Servo.class, "Gripper");

        // Initialize the Follower
        follower = new Follower(hardwareMap);

        // Define points for the paths
        Point start = new Point(36, 60, Point.CARTESIAN);   // Starting point
        Point mid = new Point(35, 25, Point.CARTESIAN);    // Intermediate point (for pick-up)
        Point end = new Point(55, 55, Point.CARTESIAN);    // Endpoint (for drop-off)

        // Define the initial PathChain
        initialPath = follower.pathBuilder()
                .addPath(new BezierLine(start, end)) // Start to End
                .setConstantHeadingInterpolation(45) // Drop-off
                .addPath(new BezierLine(start, mid)) // Start to Mid
                .setConstantHeadingInterpolation(0)  // Pick-up
                .build();

        // Define the looping PathChain
        loopingPath = follower.pathBuilder()
                .addPath(new BezierCurve(mid, start, end)) // Mid -> Start -> End
                .setConstantHeadingInterpolation(35) // Drop-off
                .addPath(new BezierCurve(end, start, mid)) // End -> Start -> Mid
                .setConstantHeadingInterpolation(0)  // Pick-up
                .setPathEndTimeoutConstraint(3.0)    // Timeout constraint
                .build();

        // Start with the initial PathChain
        follower.followPath(initialPath, true);

        // Set up telemetry
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(this.telemetry, dashboardTelemetry);
        telemetry.addLine("Paths initialized. Robot will start moving.");
        telemetry.update();
    }

    /**
     * State machine to control the autonomous sequence.
     */
    @Override
    public void loop() {
        switch (currentState) {
            case FOLLOW_INITIAL_PATH:
                follower.update();
                if (!follower.isBusy()) {
                    currentState = State.FOLLOW_LOOPING_PATH;
                    follower.followPath(loopingPath, true); // Start the looping path
                    telemetry.addLine("Initial path complete. Starting looping path.");
                    telemetry.update();
                }
                break;

            case FOLLOW_LOOPING_PATH:
                follower.update();
                if (!follower.isBusy()) {
                    loopCount++;
                    if (loopCount < MAX_LOOP_COUNT) {
                        currentState = State.WAIT;
                        timer.reset();
                        telemetry.addLine("Loop " + loopCount + " complete. Waiting...");
                    } else {
                        currentState = State.DONE;
                        telemetry.addLine("All loops complete. Ending autonomous.");
                    }
                    telemetry.update();
                }
                break;

            case WAIT:
                if (timer.seconds() > 2.0) { // Wait for 2 seconds
                    currentState = State.FOLLOW_LOOPING_PATH;
                    follower.followPath(loopingPath, true); // Repeat the looping path
                    telemetry.addLine("Restarting looping path.");
                    telemetry.update();
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous complete. Robot is holding position.");
                telemetry.update();
                requestOpModeStop(); // End the OpMode
                break;

            case MOVE_ARM:
                follower.update();
                if (!follower.isBusy()) {
                    calculationIK(30, 30);
                    currentState = State.DONE;
                    telemetry.addLine("Autonomous complete. Robot is holding position.");
                    telemetry.update();
                }
                break;
        }

        // Debug telemetry for the follower
        follower.telemetryDebug(telemetry);
    }
    public void calculationIK(double xTarget, double zTarget) {
        double L1 = 29.21; //
        double L2 = 40.64;
        // Normal inverse kinematics calculation

        double distanceToTarget = Math.sqrt(xTarget * xTarget + zTarget * zTarget);

        if (distanceToTarget > (L1 + L2)) {
            telemetry.addData("Error", "Target is out of reach.");
            telemetry.update();
        } else {
            double cosTheta2 = (L1 * L1 + L2 * L2 - distanceToTarget * distanceToTarget) / (2 * L1 * L2);
            double theta2 = Math.acos(cosTheta2);

            double k1 = L1 + L2 * Math.cos(theta2);
            double k2 = L2 * Math.sin(theta2);
            double theta1 = Math.atan2(zTarget, xTarget) - Math.atan2(k2, k1);

            double theta1Deg = Math.toDegrees(theta1);
            double theta2Deg = Math.toDegrees(theta2) - 180;

            int ShoulderTargetPos = (int) (theta1Deg * 73.3486);
            int ElbowTargetPos = (int) (theta2Deg * 38.6972);

            Shoulder.setTargetPosition(ShoulderTargetPos);
            Elbow.setTargetPosition(ElbowTargetPos);
            Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shoulder.setPower(0.8);
            Elbow.setPower(0.8);

            telemetry.addData("Theta1", theta1Deg);
            telemetry.addData("Theta2", theta2Deg);
            telemetry.addData("Shoulder Target", ShoulderTargetPos);
            telemetry.addData("Elbow Target", ElbowTargetPos);
            telemetry.update();
        }

    }
}