package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Arm;
@Autonomous(group = "robot")
public class RedRight extends OpMode {
    public enum State {
        TRAJECTORY_1,
        ARM_PICK_UP,
        DRIVE_1,
        ARM_LIFT,
        FORWARD_1,
        ARM_DROP,
        PARK,
        OFF
    }
    public SampleMecanumDrive robot;
    //public Arm arm;
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
    public State currentState;
    public Pose2d currentPose;
    public TrajectorySequence trajSeq;
    public void init(){
        robot = new SampleMecanumDrive(hardwareMap);
        //arm = new Arm(hardwareMap);
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
        currentPose = new Pose2d(11, -59, Math.toRadians(90));
        robot.setPoseEstimate(currentPose);
        currentState = State.ARM_PICK_UP;
    }
    public void init_loop(){
        //add camera and recognition stuff here
    }
    public void start(){}
    public void loop(){
        robot.update();
        switch (currentState){
            case ARM_PICK_UP:
                armTurn.setTargetPosition(0);
                armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                claw.setPosition(0.5);
                armTurn.setTargetPosition(ARM_DRIVE);
                armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentState = State.DRIVE_1;
                break;
            case DRIVE_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .forward(25)
                        .strafeRight(33)
                        .turn(Math.toRadians(-90))
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, -35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_LIFT;
                break;
            case ARM_LIFT:
                armTurn.setTargetPosition(ARM_TURN_PLACE);
                armLift.setTargetPosition(ARM_HEIGHT_PLACE_TALL);
                armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentState = State.FORWARD_1;
            case FORWARD_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .forward(4)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(48, -35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROP;
                break;
            case ARM_DROP:
                claw.setPosition(SERVO_DROP);
                armLift.setTargetPosition(0);
                armTurn.setTargetPosition(ARM_DRIVE);
                armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .back(4)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, -35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARK;
                break;
            case PARK:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(27)
                        .turn(Math.toRadians(180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, -6, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
            case OFF:
                break;
        }
    }
}
