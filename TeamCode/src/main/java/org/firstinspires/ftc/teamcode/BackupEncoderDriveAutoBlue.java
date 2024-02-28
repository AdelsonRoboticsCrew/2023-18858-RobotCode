/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Drive Backup", group="Robot")
@Disabled
public class BackupEncoderDriveAutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor armLift;
    DcMotor armTurn;
    //DcMotor airplane;
    Servo leftClaw; //rev servo goes from 0 - 1
    Servo rightClaw; //rev servo goes from 0 - 1
    Servo clawTurn;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        telemetry.addData("Status", "Initialized");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        rightClaw = hardwareMap.get(Servo.class, "right claw");
        leftClaw = hardwareMap.get(Servo.class, "left claw");
        clawTurn = hardwareMap.get(Servo.class, "claw turn");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armTurn.setDirection(DcMotor.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorFloatAndZero();
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 3);
        encoderDrive(TURN_SPEED, 5, 5, -5, -5, 2);
        encoderDrive(DRIVE_SPEED, 18, 18, 18, 18, 3);
        armTurn.setTargetPosition(210);
        armLift.setTargetPosition(3620);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.7);
        armLift.setPower(0.7);
        leftClaw.setPosition(0.655);
        rightClaw.setPosition(0.655);
        armTurn.setTargetPosition(0);
        armLift.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.7);
        armLift.setPower(0.7);
        encoderDrive(DRIVE_SPEED, -6, 6, 6, -6, 3);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderArm(double speed,
                             double armTurnInches, double armLiftInches,
                             double timeoutS) {
        int newArmTurnTarget;
        int newArmLiftTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTurnTarget = armTurn.getCurrentPosition() + (int)(armTurnInches * COUNTS_PER_INCH);
            newArmLiftTarget = armLift.getCurrentPosition() + (int)(armLiftInches * COUNTS_PER_INCH);

            armTurn.setTargetPosition(newArmTurnTarget);
            armLift.setTargetPosition(newArmLiftTarget);


            // Turn On RUN_TO_POSITION
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            armTurn.setPower(Math.abs(speed));
            armLift.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (armTurn.isBusy() && armLift.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newArmTurnTarget,  newArmLiftTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        armTurn.getCurrentPosition(), armLift.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armTurn.setPower(0);
            armLift.setPower(0);


            // Turn off RUN_TO_POSITION
            armTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }
    public void setMotorFloatAndZero() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
