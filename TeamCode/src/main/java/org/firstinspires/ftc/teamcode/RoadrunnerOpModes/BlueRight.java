package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Arm;

@Autonomous(group = "robot")
public class BlueRight extends OpMode {
    private enum State{
        //TRAJECTORY_1,
        ARM_PICK_UP,
        DRIVE_1,
        ARM_LIFT,
        FORWARD_1,
        ARM_DROP,
        PARK,
        OFF
    }

    private SampleMecanumDrive robot;
    private Arm arm;
    private State currentState;
    private Pose2d currentPose;

    @Override
    public void init() {
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        currentPose = new Pose2d(-34, 62, Math.toRadians(270));
        robot.setPoseEstimate(currentPose);
        currentState = State.ARM_PICK_UP;
    }

    @Override
    public void init_loop() {
        //add camera stuff
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.update();
        switch (currentState){
            case ARM_PICK_UP:
                arm.holdPixel();
                TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                //telemetry.addData("debug 1", currentState);
                currentState = State.DRIVE_1;
                break;
            case DRIVE_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .forward(58)
                        .waitSeconds(1)
                        .strafeLeft(78)
                        .turn(Math.toRadians(90))
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, 7, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                arm.raiseArmAuto();
                //telemetry.addData("debug 2", currentState);
                currentState = State.FORWARD_1;
                break;
            /*case ARM_LIFT:
                arm.raiseArmAuto();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(3)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                telemetry.addData("debug 3", currentState);
                currentState = State.FORWARD_1;*/
            case FORWARD_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(30)
                        .waitSeconds(1.5)
                        .forward(10)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(54, 37, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROP;
                break;
            case ARM_DROP:
                arm.claw.setPosition(0.655);
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .back(10)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, 37, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARK;
                break;
            case PARK:
                arm.dropArm();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(26)
                        .turn(Math.toRadians(-180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(50, 11, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
            case OFF:
                arm.armOff();
                break;
        }
    }
    @Override
    public void stop(){

    }
}

