package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;





@TeleOp(name="Base Teleop", group="Robot")
public class Teleop extends OpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor armLift;
    DcMotor armTurn;

    Servo claw;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        claw = hardwareMap.get(Servo.class, "claw");

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
        if (divisor >= 0.7) {
            lbPow /= divisor;
            rbPow /= divisor;
            lfPow /= divisor;
            rfPow /= divisor;
        }
        leftFront.setPower(lfPow);
        leftBack.setPower(lbPow);
        rightFront.setPower(rfPow);
        rightBack.setPower(rbPow);

        telemetry.addData("Position", armLift.getCurrentPosition());

        if (gamepad1.dpad_up) {
            armLift.setTargetPosition(3620);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);

        }
        if (gamepad1.dpad_left) {
            armLift.setTargetPosition(1250);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        if (gamepad1.dpad_right) {
            armLift.setTargetPosition(600);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        if (gamepad1.dpad_down) {
            armLift.setTargetPosition(0);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setPower(1);
        }
        if (gamepad1.a) {
            armTurn.setTargetPosition(0);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.15);
        }
        if (gamepad1.y) {
            armTurn.setTargetPosition(270);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }
        if (gamepad1.b) {
            armTurn.setTargetPosition(80);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.15);
        }
        if (gamepad1.x) {
            armTurn.setTargetPosition(210);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }
        if (armTurn.getCurrentPosition() == 0) {
            armTurn.setPower(0);
        }
        if (armLift.getCurrentPosition() == 0) {
            armLift.setPower(0);
        }
        if (gamepad1.right_bumper) {
            claw.setPosition(.655);
        }
        if (gamepad1.left_bumper) {
            claw.setPosition(.5);
        }

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