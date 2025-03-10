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
    private PathChain initialDrop;
    private PathChain initialPick;
    private PathChain loopingPick;
    private PathChain loopingDrop;
    private ElapsedTime timer = new ElapsedTime();

    private DcMotor Shoulder = null;
    private DcMotor Elbow = null;
    private DcMotor Arm_extenstion = null;
    private Servo Wrist = null;
    private Servo Gripper = null;

    private enum State {
        FOLLOW_INITIAL_PICK,
        FOLLOW_INITIAL_DROP,
        FOLLOW_LOOPING_PICK,
        FOLLOW_LOOPING_DROP,
        INITIAL_PICK_UP,
        INITIAL_DROP,
        LOOPING_PICK,
        LOOPING_DROP,
        DONE
    }

    private State currentState = State.FOLLOW_INITIAL_DROP;
    private int pickLoopCount = 0; // Track the number of iterations
    private int dropLoopCount = 0; // Track the number of iterations
    private static final int MAX_LOOP_COUNT = 3; // Number of times to repeat the loop



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
        Point start = new Point(35.4, 64.66, Point.CARTESIAN);   // Starting point
        Point mid = new Point(37.1, 28.86, Point.CARTESIAN);    // Intermediate point (for pick-up)
        Point end = new Point(53, 57.5, Point.CARTESIAN);    // Endpoint (for drop-off)

        // Define the initial Picking up PathChain
        initialDrop = follower.pathBuilder()
                .addPath(new BezierLine(start, end)) // Start to End
                .setConstantHeadingInterpolation(45) // Drop-off
                .build();

        // Define the initial Picking up PathChain
        initialPick = follower.pathBuilder()
                .addPath(new BezierLine(start, mid)) // Start to Mid
                .setConstantHeadingInterpolation(0)  // Pick-up
                .build();

        loopingPick = follower.pathBuilder()
                .addPath(new BezierCurve(end, start, mid)) // End -> Start -> Mid
                .setConstantHeadingInterpolation(0)  // Pick-up
                .build();

        // Define the looping PathChain
        loopingDrop = follower.pathBuilder()
                .addPath(new BezierCurve(mid, start, end)) // Mid -> Start -> End
                .setConstantHeadingInterpolation(45) // Drop-off
                .setPathEndTimeoutConstraint(3.0)    // Timeout constraint
                .build();

        // Start with the initial PathChain
        follower.followPath(initialDrop, true);

        // Set up telemetry
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(this.telemetry, dashboardTelemetry);
        telemetry.addLine("Paths initialized. Robot will start moving.");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (currentState) {
            case FOLLOW_INITIAL_DROP:
                follower.update();
                if (!follower.isBusy()) {
                    currentState = State.INITIAL_DROP;
                    follower.followPath(initialPick, true); // Start the looping path
                    telemetry.addLine("Initial path complete. Starting looping path.");
                    telemetry.update();
                }
                break;

            case FOLLOW_INITIAL_PICK:
                follower.update();
                if (!follower.isBusy()) {
                    currentState = State.INITIAL_PICK_UP;
                    follower.followPath(loopingDrop, true); // Start the looping path
                    telemetry.addLine("Initial path complete. Starting looping path.");
                    telemetry.update();
                }
                break;

            case FOLLOW_LOOPING_PICK:
                follower.update();
                if (!follower.isBusy()) {
                    pickLoopCount++;
                    if (pickLoopCount < MAX_LOOP_COUNT) {
                        currentState = State.LOOPING_PICK;
                        timer.reset();
                        telemetry.addLine("Loop " + pickLoopCount + " complete. Waiting...");
                        telemetry.update();

                        // Move the arm to intermediate position during the wait
                        //calculationIK(25, 0); // Example position during wait
                    } else {
                        currentState = State.DONE;
                        telemetry.addLine("All loops complete. Ending autonomous.");
                        telemetry.update();
                    }
                }
                break;

            case INITIAL_DROP:
               if (timer.seconds() > 1.5 && timer.seconds() <= 2.0) {
                    telemetry.addLine("Moving arm to target position 1...");
                    //calculationIK(50, 0); // Target position 2
                    telemetry.update();
                }

                if (timer.seconds() > 5.0) { // Wait for 2 seconds total
                    currentState = State.FOLLOW_INITIAL_PICK;
                    follower.followPath(initialPick, true); // Repeat the looping path
                    telemetry.addLine("Restarting looping path.");
                    telemetry.update();
                }
                break;

            case INITIAL_PICK_UP:
                // Move arm dynamically during wait
                if (timer.seconds() > 0.5 && timer.seconds() <= 2.5) {
                    telemetry.addLine("Moving arm to target position 1...");
                    //calculationIK(25, 0); // Target position 1
                    telemetry.update();
                }
                if (timer.seconds() > 5.0) { // Wait for 5 seconds total
                    currentState = State.FOLLOW_LOOPING_DROP;
                    follower.followPath(loopingDrop, true); // Repeat the looping path
                    telemetry.addLine("Restarting looping path.");
                    telemetry.update();
                }
                break;

            case LOOPING_DROP:
                if (timer.seconds() > 1.5 && timer.seconds() <= 2.0) {
                    telemetry.addLine("Moving arm to target position 2...");
                    //calculationIK(40, 0); // Target position 2
                    telemetry.update();
                }

                if (timer.seconds() > 5.0) { // Wait for 5 seconds total
                    currentState = State.FOLLOW_LOOPING_PICK;
                    follower.followPath(loopingPick, true); // Repeat the looping path
                    telemetry.addLine("Restarting looping path.");
                    telemetry.update();
                }
                break;

            case LOOPING_PICK:
                // Move arm dynamically during wait
                if (timer.seconds() > 0.5 && timer.seconds() <= 1.5) {
                    telemetry.addLine("Moving arm to target position 1...");
                    //calculationIK(30, 0); // Target position 1
                    //telemetry.update();
                }
                if (timer.seconds() > 5.0) { // Wait for 5 seconds total
                    currentState = State.FOLLOW_LOOPING_DROP;
                    follower.followPath(loopingDrop, true); // Repeat the looping path
                    telemetry.addLine("Restarting looping path.");
                    telemetry.update();
                }
                break;


            case FOLLOW_LOOPING_DROP:
                follower.update();
                if (!follower.isBusy()) {
                    dropLoopCount++;
                    if (dropLoopCount < MAX_LOOP_COUNT) {
                        currentState = State.LOOPING_DROP;
                        timer.reset();
                        telemetry.addLine("Loop " + dropLoopCount + " complete. Waiting...");
                        telemetry.update();
                        // Move the arm to intermediate position during the wait
                        //calculationIK(30, 0); // Example position during wait
                    } else {
                        currentState = State.DONE;
                        telemetry.addLine("All loops complete. Ending autonomous.");
                        telemetry.update();
                    }
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous complete. Robot is holding position.");
                telemetry.update();
                requestOpModeStop(); // End the OpMode
                break;
        }

        // Debug telemetry for the follower
        follower.telemetryDebug(telemetry);
    }

    public void calculationIK(double xTarget, double zTarget) {
        double L1 = 29.21; // Length of first arm segment
        double L2 = 40.64; // Length of second arm segment

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