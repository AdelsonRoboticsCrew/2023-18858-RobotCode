package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class DriveBase{
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;

    public DriveBase(DcMotor lb, DcMotor lf, DcMotor rb, DcMotor rf){
        leftBack = lb;
        leftFront = lf;
        rightBack = rb;
        rightFront = rf;
    }

    public void setPow(double pow){
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);
    }

    public void setPow(double lPow, double rPow){
        leftFront.setPower(lPow);
        leftBack.setPower(lPow);
        rightFront.setPower(rPow);
        rightBack.setPower(rPow);
    }

    public void setPow(double lfPow, double lbPow, double rfPow, double rbPow){
        leftFront.setPower(lfPow);
        leftBack.setPower(lbPow);
        rightFront.setPower(rfPow);
        rightBack.setPower(rbPow);
    }
}
