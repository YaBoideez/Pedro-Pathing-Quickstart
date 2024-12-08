/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="CompTeleopV1", group="Linear OpMode")
//@Disabled
public class CompTeleopV1 extends LinearOpMode {

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.035  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.035 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.035  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorFL = null;
    private DcMotor MotorBL = null;
    private DcMotor MotorFR = null;
    private DcMotor MotorBR = null;
    private DcMotor Shoulder = null;
    private DcMotor Elbow = null;
    private DcMotor Arm_extenstion = null;
    private Servo Wrist = null;
    private Servo Gripper = null;

    SparkFunOTOS myOtos;


    int currentServoPosition;
    private boolean ikFlag = true;

    @Override
    public void runOpMode() {

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MotorFL = hardwareMap.get(DcMotor.class, "MotorFL");
        MotorBL = hardwareMap.get(DcMotor.class, "MotorBL");
        MotorFR = hardwareMap.get(DcMotor.class, "MotorFR");
        MotorBR = hardwareMap.get(DcMotor.class, "MotorBR");
        Shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        Elbow = hardwareMap.get(DcMotor.class, "Elbow");
        Arm_extenstion = hardwareMap.get(DcMotor.class, "Arm_extenstion");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Gripper = hardwareMap.get(Servo.class, "Gripper");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        MotorFL.setDirection(DcMotor.Direction.REVERSE);
        MotorBL.setDirection(DcMotor.Direction.REVERSE);
        MotorFR.setDirection(DcMotor.Direction.FORWARD);
        MotorBR.setDirection(DcMotor.Direction.FORWARD);

        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_extenstion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //setArmPosition(0, 20);

        waitForStart();
        runtime.reset();
        double xTarget = 10;
        double zTarget = 25;
        currentServoPosition = (int) 1;
        Wrist.setPosition(currentServoPosition);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Shoulder.setPower(gamepad2.right_stick_y);
            //Elbow.setPower(gamepad2.left_stick_y);


            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Reset the tracking if the user requests it
            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            // Re-calibrate the IMU if the user requests it
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            // Update the telemetry on the driver station
            telemetry.update();
            double max;

            //Use left bumper to drive to net position

            if(gamepad1.left_bumper){
                goToTarget(0,0,0);
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if (gamepad2.left_bumper){
                yaw += .1;
            }
            if (gamepad2.right_bumper){
                yaw -= .1;

            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            Arm_extenstion.setPower(-gamepad2.right_trigger);
            Arm_extenstion.setPower(gamepad2.left_trigger);
            // Reset encoders if dpad_down is pressed
            if (gamepad2.dpad_right) {
                Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm_extenstion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //setting the target in XZ plane
            xTarget += -gamepad2.left_stick_y;
            zTarget += -gamepad2.right_stick_y;

            //Turns the wrist up, retracts the extension and shoulder and arm to init position
            if (gamepad2.dpad_left){
                ikFlag  = true;
                Wrist.setPosition(0.0); // wrist up
                sleep(300);
                sleep(300);
                Arm_extenstion.setTargetPosition(0);
                Arm_extenstion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_extenstion.setPower(0.9);
                if (Gripper.getPosition() == 0.8) {
                    sleep(2000);
                }
                xTarget = 10;
                zTarget = 25;
            }
            // set the IK flag to false so that we don't move the shoulder and arm
            // close wrist
            // reach and extend the arm
            if (gamepad2.dpad_up){
                ikFlag = false;
                Wrist.setPosition(0);
                sleep(300);
                Elbow.setTargetPosition(-900);
                sleep(600);
                Shoulder.setTargetPosition(500);
                sleep(500);
                Arm_extenstion.setTargetPosition(-2300);
                Arm_extenstion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm_extenstion.setPower(1);
                //xTarget = -3.6312;
                //zTarget = 67.1264;
                //calculationIK(xTarget,zTarget);
                sleep(1500);
                //Wrist.setPosition(0.6);
            }

            if (gamepad2.dpad_down) {
                Gripper.setPosition(0.8);
                sleep(500);
                zTarget -= 5;
                calculationIK(xTarget, zTarget);
                sleep(500);
                Gripper.setPosition(1);
                sleep(500);
                zTarget += 7;
                calculationIK(xTarget, zTarget);
            }

            if (gamepad2.touchpad_finger_1){
                ikFlag = true;
                xTarget = 0.2209;
                zTarget = 64.7306;

            }




            // Trigger IK calculation with gamepad2.x (instead of gamepad2.square)
            if (xTarget > 0 && xTarget<49 && ikFlag) {
                calculationIK(xTarget, zTarget);
            }
            Rotate_wrist();
            Open_Close_Claw();


            telemetry.addData("Shoulder Target Pos", Shoulder.getTargetPosition());
            telemetry.addData("Shoulder Current Pos", Shoulder.getCurrentPosition());
            telemetry.addData("Elbow Target Pos", Elbow.getTargetPosition());
            telemetry.addData("Elbow Current Pos", Elbow.getCurrentPosition());
            telemetry.addData("Slide Current Pos", Arm_extenstion.getCurrentPosition());
            telemetry.addData("Slide Target Pos", Arm_extenstion.getTargetPosition());
            telemetry.addData("Wrist Current:", Wrist.getPosition());
            telemetry.addData("Gripper Current:", Gripper.getPosition());




            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            MotorFL.setPower(leftFrontPower);
            MotorFR.setPower(rightFrontPower);
            MotorBL.setPower(leftBackPower);
            MotorBR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("X target", xTarget);
            telemetry.addData("Z target", zTarget);

            telemetry.update();

            sleep(0);
        }
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

    private void Rotate_wrist() {
        if (gamepad2.a) {
            currentServoPosition = Math.min(currentServoPosition + 4, 180);
            Wrist.setPosition(currentServoPosition / 180.0);
        } else if (gamepad2.b) {
            currentServoPosition = Math.max(currentServoPosition - 4, 0);
            Wrist.setPosition(currentServoPosition / 180.0);
        }

    }

    private void Open_Close_Claw() {
        if (gamepad2.x) {
            Gripper.setPosition(0.8); // Open
        } else if (gamepad2.y) {
            Gripper.setPosition(1); // Close
        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 180);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    public void pickUpBlock(){
        ikFlag = true;
        calculationIK(14.5263,11.6001);
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        MotorFL.setPower(leftFrontPower);
        MotorFR.setPower(rightFrontPower);
        MotorBL.setPower(leftBackPower);
        MotorBR.setPower(rightBackPower);
    }

    public void goToTarget(double xTarget, double yTarget, double headingTarget) {

        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        ElapsedTime runtime = new ElapsedTime();

        runtime.reset();

        while (runtime.seconds() < 3.0) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Determine x, y and heading error so we can use them to control the robot automatically.
            double xError = (xTarget - pos.x);
            double yError = (yTarget - pos.y);
            double headingError = (headingTarget - pos.h);


            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(yError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-xError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }
}