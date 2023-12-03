package org.firstinspires.ftc.teamcode.robotutil;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Arm {
    public DcMotor armLift;
    public DcMotor armTurn;
    public Servo claw;
    private final int ARM_HEIGHT_PLACE = 3620;
    private final double SERVO_HOLD = 0.5;
    private final double SERVO_DROP = 0.655;
    //TODO: add constants for armTurn :)
    public Arm(HardwareMap hardwareMap){
        init(hardwareMap);
    }
    private void init(HardwareMap hardwareMap){
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        claw = hardwareMap.get(Servo.class, "claw");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.scaleRange(.5,.655);
        claw.setPosition(0.655);
        armTurn.setDirection(DcMotorSimple.Direction.REVERSE);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
