package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Servo test", group="Robot")
public class servoTester extends OpMode{
    Servo claw;

    @Override
    public void init(){
        claw = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop(){
        if(gamepad1.a){
            claw.setPosition(0.5);
        }

        if(gamepad1.b){
            claw.setPosition(0.7);
        }
    }
}
