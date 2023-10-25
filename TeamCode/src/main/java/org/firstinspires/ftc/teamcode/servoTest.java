package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class servoTest extends OpMode{
    CRServo servo1;
    CRServo servo2;
    boolean pressed;
    boolean pressed2;

    @Override
    public void init(){
        servo1 = hardwareMap.get(CRServo.class, "servo left");
        servo2 = hardwareMap.get(CRServo.class, "servo right");

        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);


    }

    @Override
    public void loop(){
        pressed = gamepad1.a;
        pressed2 = gamepad1.b;
        if(pressed){
            servo1.setPower(1);
            servo2.setPower(1);
        }
        if(pressed2){
            servo1.setPower(0);
            servo2.setPower(0);
        }

    }
}
