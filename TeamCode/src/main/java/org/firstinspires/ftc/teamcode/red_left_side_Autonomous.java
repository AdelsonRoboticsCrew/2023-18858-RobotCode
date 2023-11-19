package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="red left side Autonomous", group="Robot")
public class red_left_side_Autonomous extends LinearOpMode
{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Status Initialized");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();

        waitForStart();
        while(opModeIsActive()) {
            if (runtime.seconds() < 1.65) { //add a tiny bit more time, but it's basically good
                frontLeft.setPower(0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(0.75);
            }
            else if (runtime.seconds() < 6.0 && runtime.seconds() >= 1.65) {
                frontLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backLeft.setPower(-0.75);
                backRight.setPower(0.75);
            }
            else if (runtime.seconds() < 9.0 && runtime.seconds() >= 6.0) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(-0.75);
                backLeft.setPower(-0.75);
                backRight.setPower(-0.75);
            }
            else if (runtime.seconds() < 11.0 && runtime.seconds() >= 9.0) {
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
        }
        sleep(1000);
        stop();
    }
}