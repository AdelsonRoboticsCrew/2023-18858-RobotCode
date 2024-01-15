package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Arm {
    private DcMotor armLift;
    private DcMotor armTurn;
    public Servo claw;
    private final int ARM_HEIGHT_PLACE_TALL = 3620;
    public final int ARM_HEIGHT_PLACE_MEDIUM = 1250;
    public final int ARM_HEIGHT_PLACE_SHORT = 600;
    private double SERVO_HOLD = 0.5;
    private final double SERVO_DROP = 0.655;
    private final int ARM_DRIVE = 80;
    private final int ARM_LIGHT_LIFT = 40;
    private final int ARM_TURN_PLACE = 210;
    public final int ARM_TURN_PLACE_HIGHER = 270;
    public final int ARM_HANG = 400;
    public Arm(HardwareMap hardwareMap){
        init(hardwareMap);
    }
    private void init(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, "arm lift");
        claw = hardwareMap.get(Servo.class, "claw");
        armTurn = hardwareMap.get(DcMotor.class, "arm turn");
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //claw.scaleRange(.5, .655);
        claw.setPosition(0.5);
        armTurn.setDirection(DcMotorSimple.Direction.REVERSE);
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void holdPixel(){
        armTurn.setTargetPosition(10);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(.65);
        claw.setPosition(SERVO_HOLD);
        armTurn.setTargetPosition(ARM_DRIVE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(.65);
    }

    public void onlyHoldPixel(){
        claw.setPosition(SERVO_HOLD);
    }
    public void dropPixel(){
        claw.setPosition(SERVO_DROP);
    }
    public void raiseArmAuto(){
        armTurn.setTargetPosition(ARM_TURN_PLACE);
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_TALL);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.7);
        armLift.setPower(0.7);
    }

    public void raiseArmForPixel(){
        armLift.setTargetPosition(ARM_LIGHT_LIFT);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
    }
    public void dropArm(){
        armLift.setTargetPosition(0);
        armTurn.setTargetPosition(ARM_DRIVE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(1);
        armLift.setPower(1);
    }
    public void armOff(){
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(1);
        while(true){
            if(armTurn.getCurrentPosition() == 0){
                armTurn.setPower(0);
                break;
            }
            break;
        }
    }
    public void armLiftHigh(){
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_TALL);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
    }
    public void armLiftMed(){
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_MEDIUM);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
    }
    public void armLiftLow(){
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_SHORT);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
    }
    public void armLiftDown(){
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(1);
        while(true){
            if(armLift.getCurrentPosition() == 0){
                armLift.setPower(0);
            }
            break;
        }
    }
    public void armTurnDown(){
        armTurn.setTargetPosition(10);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.2);
    }
    public void armTurnPLaceHigh(){
        armTurn.setTargetPosition(ARM_TURN_PLACE_HIGHER);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.5);
    }
    public void armDrive(){
        armTurn.setTargetPosition(ARM_DRIVE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.3);
    }
    public void armTurnPlace(){
        armTurn.setTargetPosition(ARM_TURN_PLACE);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.5);
    }
    public void hang(){
        armTurn.setTargetPosition(ARM_HANG);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setPower(0.5);
        armLift.setTargetPosition(ARM_HEIGHT_PLACE_TALL);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(0.5);
    }
}
