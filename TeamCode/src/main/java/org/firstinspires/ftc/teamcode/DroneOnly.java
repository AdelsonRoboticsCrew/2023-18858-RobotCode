package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="Drone tester", group="Robot")
public class DroneOnly extends OpMode {


    DcMotor airplane;


    boolean lastPressed = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        airplane = hardwareMap.get(DcMotor.class, "drone");


        //setMotorFloatAndZero();


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


        if (gamepad1.b) {
            airplane.setPower(1);
        }

        if (gamepad1.a) {
            airplane.setPower(0);
        }
        telemetry.update();
    }

    @Override
    public void stop() {

    }

}


