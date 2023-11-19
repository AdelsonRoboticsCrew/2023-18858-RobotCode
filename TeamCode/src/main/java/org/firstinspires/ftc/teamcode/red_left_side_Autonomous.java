package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="red left side Autonomous", group="Robot")
public class red_left_side_Autonomous extends OpMode
{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
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

    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        if(runtime.seconds() < 2.0 )
        {
            frontLeft.setPower(0.75);
            frontRight.setPower(0.75);
            backLeft.setPower(0.75);
            backRight.setPower(0.75);
        }
        else if(runtime.seconds() < 7.0 )
        {
            frontLeft.setPower(-0.75);
            frontRight.setPower(0.75);
            backLeft.setPower(0.75);
            backRight.setPower(-0.75);
        }
        else if(runtime.seconds() < 9.0 )
        {
            frontLeft.setPower(-0.75);
            frontRight.setPower(-0.75);
            backLeft.setPower(-0.75);
            backRight.setPower(-0.75);
        }
        else if(runtime.seconds() < 11.0 )
        {
            frontLeft.setPower(-0.75);
            frontRight.setPower(0.75);
            backLeft.setPower(0.75);
            backRight.setPower(-0.75);
        }
    }
}