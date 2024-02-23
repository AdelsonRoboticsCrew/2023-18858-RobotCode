package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(group = "robot")
public class BlueLeftTwoPlacedPixels extends OpMode{
    private enum State {
        //TRAJECTORY_1, this is going to be used for camera stuff
        LEFT,
        RIGHT,
        MIDDLE,
        ARM_LIFT_A ,
        ARM_PICK_UP,
        DRIVE_1,
        ARM_LIFT,
        FORWARD_1,
        ARM_DROP,
        TURN_AROUND,
        PICK_UP1,
        PICK_UP2,
        SECOND_PLACE,
        DROP_SECOND,
        PARK,
        OFF
    }

    private SampleMecanumDrive robot;
    private Arm arm;
    private State currentState;
    private Pose2d currentPose;
    @Override
    public void init(){
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        currentPose = new Pose2d(11, 62, Math.toRadians(270));
        robot.setPoseEstimate(currentPose);
        currentState = State.ARM_PICK_UP;
    }
    @Override
    public void init_loop(){
        //add camera and recognition stuff here
    }
    @Override
    public void start(){}
    @Override
    public void loop(){
        robot.update();
        switch (currentState){
            /*case ARM_LIFT_A:
                arm.raiseArmForPixel();
                TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentState = State.ARM_PICK_UP; */
            case ARM_PICK_UP:
                arm.onlyHoldPixel();
                TrajectorySequence trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentState = State.DRIVE_1;
                break;
            case DRIVE_1:
                arm.raiseArmAuto();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .forward(25)
                        .back(5)
                        .strafeLeft(34)
                        .turn(Math.toRadians(90))
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(48, 42, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1;
                break;
            /*case ARM_LIFT:
                arm.raiseArmAuto();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentState = State.FORWARD_1;*/
            case FORWARD_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(48, 42, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROP;
                break;
            case ARM_DROP:
                arm.claw.setPosition(0.7);
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .back(5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(43, 42, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.TURN_AROUND;
                break;
            case TURN_AROUND:
                arm.dropArm();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1)
                        .turn(Math.toRadians(180))
                        .strafeLeft(13)
                        .forward(28)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(15, 29, Math.toRadians(180));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PICK_UP1;
                break;
            case PICK_UP1:
                arm.fullyDropArm();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(15, 29, Math.toRadians(180));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PICK_UP2;
                break;
            case PICK_UP2:
                arm.claw.setPosition(0.5);
                arm.raiseArmAuto();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1)
                        .back(28)
                        .turn(Math.toRadians(180))
                        .forward(10)
                        .strafeLeft(7) //57 -38
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(50, 39, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DROP_SECOND;
                break;
            /*case SECOND_PLACE:
                arm.raiseArmAuto();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(3).build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(48, -38, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DROP_SECOND; */
            case DROP_SECOND:
                arm.claw.setPosition(0.7);
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(50, 39, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DROP_SECOND;
            case PARK:
                arm.dropArm();
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .back(5)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(45, 36, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
            case OFF:
                arm.armOff();
                break;
        }
    }
    @Override
    public void stop(){}
}