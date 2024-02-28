package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="Encoder Test", group="Robot")
public class encoderTest extends OpMode{
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor armTurn;
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        armTurn.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop(){

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double lbPow = drive - strafe + turn;
        double rbPow = drive + strafe - turn;
        double lfPow = drive + strafe + turn;
        double rfPow = drive - strafe - turn;

        double divisor = Math.max(Math.max(lfPow, lbPow), Math.max(rfPow, rbPow));
        if (divisor >= 0.8) {
            lbPow /= divisor;
            rbPow /= divisor;
            lfPow /= divisor;
            rfPow /= divisor;
        }
        lbPow *= 0.8;
        lfPow *= 0.8;
        rbPow *= 0.8;
        rfPow *= 0.8;
        leftFront.setPower(lfPow);
        leftBack.setPower(lbPow);
        rightFront.setPower(rfPow);
        rightBack.setPower(rbPow);

        if (gamepad1.b) {
            armTurn.setTargetPosition(120);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
        }

        telemetry.addData("Left front: ", leftFront.getCurrentPosition());
        telemetry.addData("Left back: ", leftBack.getCurrentPosition());
        telemetry.addData("Right front: ", rightFront.getCurrentPosition());
        telemetry.addData("Right back: ", rightBack.getCurrentPosition());


    }
}
