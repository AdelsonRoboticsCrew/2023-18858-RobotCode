package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;





@TeleOp(name="Base Teleop", group="Robot")
public class Teleop extends OpMode{
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor armLift;
    DcMotor armTurn;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");

        //set the directions for each motor -- i recommend doing this after you have tested the drivebase
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armTurn.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double lbPow = drive - strafe + turn;
        double rbPow = drive + strafe - turn;
        double lfPow = drive + strafe + turn;
        double rfPow = drive - strafe - turn;

        double divisor = Math.max(Math.max(lfPow, lbPow), Math.max(rfPow, rbPow));
        if(divisor > 1.0)
        {
            lbPow/=divisor;
            rbPow/=divisor;
            lfPow/=divisor;
            rfPow/=divisor;
        }
        leftFront.setPower(lfPow);
        leftBack.setPower(lbPow);
        rightFront.setPower(rfPow);
        rightBack.setPower(rbPow);

        if(!gamepad2.b && !gamepad1.y){
            armLift.setPower(0);
        }
        while(gamepad1.b){
            armLift.setPower(1);
        }
        while(gamepad1.y){
            armLift.setPower(-1);
        }
        if(!gamepad1.x && !gamepad1.y){
            armTurn.setPower(0);
        }
        while(gamepad1.x){
            armTurn.setPower(1);
        }
        while(gamepad1.a){
            armTurn.setPower(-1);
        }

    }

    @Override
    public void stop(){

    }
}
