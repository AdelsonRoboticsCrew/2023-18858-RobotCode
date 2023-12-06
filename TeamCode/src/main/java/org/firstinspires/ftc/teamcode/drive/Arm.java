package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Arm {
    public DcMotor armLift;
    public DcMotor armTurn;
    public Servo claw;
    public final int ARM_HEIGHT_PLACE_TALL = 3620;
    public final int ARM_HEIGHT_PLACE_MEDIUM = 1250;
    public final int ARM_HEIGHT_PLACE_SHORT = 600;
    public double SERVO_HOLD = 0.5;
    public final double SERVO_DROP = 0.655;
    public final int ARM_DRIVE = 80;
    public final int ARM_TURN_PLACE = 210;
    public final int ARM_TURN_PLACE_HIGHER = 270;
    public final int ARM_HANG = 320;
    public Arm(HardwareMap hardwareMap){
        init(hardwareMap);
    }
    private void init(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        claw = hardwareMap.get(Servo.class, "claw");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);;
        claw.setPosition(0.655);
        armTurn.setDirection(DcMotorSimple.Direction.REVERSE);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void holdPixel(){
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setPosition(SERVO_HOLD);
        armTurn.setTargetPosition(ARM_DRIVE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void dropPixel(){
        claw.setPosition(SERVO_DROP);
    }
    public void raiseArmAuto(){
        armTurn.setTargetPosition(ARM_TURN_PLACE);
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_TALL);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void dropArm(){
        armLift.setTargetPosition(0);
        armTurn.setTargetPosition(ARM_DRIVE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
