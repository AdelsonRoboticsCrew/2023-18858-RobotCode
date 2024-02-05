package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@TeleOp(name="pull arm", group="Robot")
public class pullArnBack extends OpMode{
    DcMotor armLift;
    DcMotor armTurn;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        //airplane = hardwareMap.get(DcMotor.class, "drone");

        armLift.setDirection(DcMotor.Direction.REVERSE);
        armTurn.setDirection(DcMotor.Direction.REVERSE);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    public void loop(){
        if(gamepad1.a){
            armTurn.setTargetPosition(420);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTurn.setPower(0.4);
            armLift.setPower(0.3);
        }
        if(gamepad1.b){
            armLift.setPower(0);
            armTurn.setTargetPosition(20);
            armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
}
