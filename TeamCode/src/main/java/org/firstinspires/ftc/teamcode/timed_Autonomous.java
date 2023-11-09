package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="practice autonomous", group="Robot")
public class timed_Autonomous extends OpMode
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


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
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
        while(runtime.seconds() < 3.0 )
        {
            frontLeft.setPower(0.75);
            frontRight.setPower(0.75);
            backLeft.setPower(0.75);
            backRight.setPower(0.75);
        }
        while(runtime.seconds() < 5.0 )
        {
            frontLeft.setPower(0.75);
            frontRight.setPower(-0.75);
            backLeft.setPower(-0.75);
            backRight.setPower(0.75);
        }
        while(runtime.seconds() < 8.0 )
        {
            frontLeft.setPower(-0.75);
            frontRight.setPower(-0.75);
            backLeft.setPower(-0.75);
            backRight.setPower(-0.75);
        }


    }
}

