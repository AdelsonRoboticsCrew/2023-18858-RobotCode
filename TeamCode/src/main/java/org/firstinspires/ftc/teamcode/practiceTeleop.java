package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Practice Teleop", group="Robot")
public class practiceTeleop extends OpMode
{

  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor backRight;
  private Servo claw;



    @Override
    public void init(){

        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        claw = hardwareMap.get(Servo.class, "Claw");

        claw.setDirection(Servo.Direction.FORWARD);
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
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        double frontLeftPower = drive+strafe+turn;
        double frontRightPower = drive-strafe-turn;
        double backLeftPower = drive-strafe+turn;
        double backRightPower = drive+strafe-turn;

        double divisor = Math.max(Math.max(frontLeftPower, backLeftPower), Math.max(frontRightPower, backRightPower));
        if(divisor > 1)
        {
            frontLeftPower/=divisor;
            frontRightPower/=divisor;
            backLeftPower/=divisor;
            backRightPower/=divisor;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


         if(gamepad1.right_bumper)
         {
            claw.setPosition(0.8);
         }

         if(gamepad1.left_bumper);
        {
            claw.setPosition(0.0);
        }





    }

    @Override
    public void stop(){

    }

}

