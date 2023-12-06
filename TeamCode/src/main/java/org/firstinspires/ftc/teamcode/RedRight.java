package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    public Arm arm;
    public State currentState;
    public Pose2d currentPose;
    public TrajectorySequence trajSeq;
    public void init(){
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
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
                arm.holdPixel();
                currentState = State.DRIVE_1;
                break;
            case DRIVE_1:
                trajSeq = robot.trajectorySequenceBuilder(currentPose)
                        .forward(25)
                        .strafeRight(33)
                        .turn(Math.toRadians(90))
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, -35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_LIFT;
                break;
            case ARM_LIFT:
                arm.raiseArmAuto();
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
                arm.dropPixel();
                arm.dropArm();
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
                        .strafeLeft(24)
                        .turn(Math.toRadians(180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeq);
                currentPose = new Pose2d(44, -9, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
            case OFF:
                break;
        }
    }
    public void stop(){}
}
