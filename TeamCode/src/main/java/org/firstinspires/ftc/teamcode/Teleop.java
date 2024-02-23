package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Base Teleop", group="Robot")
public class Teleop extends OpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor armLift;
    DcMotor armTurn;
    //DcMotor airplane;
    Servo leftClaw; //rev servo goes from 0 - 1
    Servo rightClaw; //rev servo goes from 0 - 1
    Servo clawTurn; //axon servo goes from idk - idk
    //Servo claw; this was used for the old iteration of the claw
    boolean lastPressed = false;

    @Override
    public void init() {
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
        //claw = hardwareMap.get(Servo.class, "claw");
        //airplane = hardwareMap.get(DcMotor.class, "drone");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armTurn.setDirection(DcMotor.Direction.REVERSE);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorFloatAndZero();
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double lbPow = drive - strafe + turn;
        double rbPow = drive + strafe - turn;
        double lfPow = drive + strafe + turn;
        double rfPow = drive - strafe - turn;

        double divisor = Math.max(Math.max(lfPow, lbPow), Math.max(rfPow, rbPow));
        if (divisor >= 0.8) {
            lbPow /= divisor;
            rbPow /= divisor;
            lfPow /= divisor;
            rfPow /= divisor;
        }
        lbPow *= 0.8;
        lfPow *= 0.8;
        rbPow *= 0.8;
        rfPow *= 0.8;
        leftFront.setPower(lfPow);
        leftBack.setPower(lbPow);
        rightFront.setPower(rfPow);
        rightBack.setPower(rbPow);

        telemetry.addData("Position", armLift.getCurrentPosition());

        /*
         * gamepad 1 dpad up:
         * this button when pressed will extend the arm to the max position
         */
        if (gamepad1.dpad_up) {
            armLift.setTargetPosition(3620);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        /*
         * gamepad 1 dpad right:
         * this button when pressed will extend the arm to half the max position
         */
        if (gamepad1.dpad_right) {
            armLift.setTargetPosition(1250);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        /*
         * gamepad 1 dpad down:
         * this button when pressed will extend the arm to the 0 position
         */
        if (gamepad1.dpad_down) {
            armLift.setTargetPosition(0);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        /*
         * gamepad 1 button a:
         * rotate the arm so it is touching the floor (initialized position)
         * in order for it not to break the robot, the claw must be rotated to the position
         * that is in accordance with picking up pixels (about 90 degrees)
         */
        if (gamepad1.a) {
            armTurn.setTargetPosition(0);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.15);
        }
        /*
         * gamepad 1 button b:
         * rotate the arm so it is high enough to go under the bars but not drag the claw on the ground
         */
        if (gamepad1.b) {
            armTurn.setTargetPosition(80);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }
        /*
         * gamepad 1 button x:
         * rotate the arm so it is placing the pixels low on the backboard
         * values must change
         */
        if (gamepad1.x) {
            armTurn.setTargetPosition(230);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }
        /*
         * gamepad 1 button y:
         * rotate the arm so it is placing the pixels high on the backboard
         * values must change
         */
        if (gamepad1.y) {
            armTurn.setTargetPosition(420);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }

        //code second gamepad!!!

        if(gamepad2.left_bumper){
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.5);
        }

        if(gamepad2.right_bumper){
            rightClaw.setPosition(0.655);
            leftClaw.setPosition(0.655);
        }
        if(gamepad2.a){
            rightClaw.setPosition(0.655);
        }

        if(gamepad2.b){
            leftClaw.setPosition(0.655);
        }

        if(gamepad2.y){
            rightClaw.setPosition(0.5);
        }
        if(gamepad2.x){
            leftClaw.setPosition(0.5);
        }
        if(gamepad2.dpad_up){
            clawTurn.setPosition(0.3);
        }

        if(gamepad2.dpad_down){
            clawTurn.setPosition(0.6);
        }

        if(gamepad2.left_stick_button){
            armTurn.setTargetPosition(400);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);

        }

       /* if(gamepad2.right_stick_button){
            insert drone thing
        } */
        if (armTurn.getCurrentPosition() == 0) {
            armTurn.setPower(0);
        }
        if (armLift.getCurrentPosition() == 0) {
            armLift.setPower(0);
        }
/*
first controller controls: (drive and arm)
joystick 1 - drive and strafe
joystick 2 - turn

a - down
b - hold up to drive
x - place low
y - place high

dpad up - extend all the way
dpad down - retract all the way
dpad right - extend half way
dpad left -

second controller controls: (claw and endgame)
left bumper - open both
right bumper - close both

x - open left
b - close left
y - open right
a - close right

dpad up - rotate to place claw
dpad down - rotate to pick up claw

left joystick in - hanging position
right joystick in - drone launch





        if(gamepad1.right_stick_button && !lastPressed){
            airplane.setPower(0.4);
            lastPressed = true;
        }

        if(gamepad1.right_stick_button && lastPressed){
            airplane.setPower(0);
            lastPressed = false;
        }
*/
        telemetry.update();
    }

    @Override
    public void stop() {

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