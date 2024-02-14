package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupblue;
import org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupdetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "robot")
public class BlueLeft extends OpMode {
    private enum State {
        //TRAJECTORY_1,
        ARM_PICK_UP,
        DRIVE_1L,
        DRIVE_1M,
        DRIVE_1R,
        ARM_LIFT,
        FORWARD_1L,
        FORWARD_1M,
        FORWARD_1R,
        ARM_DROPL,
        ARM_DROPM,
        ARM_DROPR,

        PARKL,
        PARKM,
        PARKR,
        LEFT, MID, RIGHT, OFF
    }
    private SampleMecanumDrive robot;
    private Arm arm;
    private State currentState;
    private Pose2d currentPose;
    OpenCvCamera webcam;
    whatifyouwokeupblue detector = new whatifyouwokeupblue(telemetry);

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        currentPose = new Pose2d(11, 62, Math.toRadians(270));
        robot.setPoseEstimate(currentPose);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);//, cameraMonitorViewId);

        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Location " + detector.getLocation());
        if(detector.getLocation() == whatifyouwokeupdetector.Location.LEFT){
            currentState = State.LEFT;
        }
        if(detector.getLocation() == whatifyouwokeupdetector.Location.MIDDLE){
            currentState = State.MID;
        }
        if(detector.getLocation() == whatifyouwokeupdetector.Location.RIGHT){
            currentState = State.RIGHT;
        }
        updateTelemetry(telemetry);
    }

    @Override
    public void start() {
        webcam.stopStreaming();
    }

    @Override
    public void loop() {
        robot.update();
        switch (currentState){
            case LEFT:
                arm.holdPixel();
                TrajectorySequence trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .forward(23)
                        .turn(Math.toRadians(45))
                        .forward(7)
                        .waitSeconds(1)
                        .back(7)
                        .turn(Math.toRadians(-45))
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(11, 46, Math.toRadians(270));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1L;
                break;
            case DRIVE_1L:
                arm.raiseArmAuto();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .back(15)
                        .strafeLeft(30)
                        .forward(17)
                        .turn(Math.toRadians(90))
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(46,44, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1L;
                break;
            case FORWARD_1L:
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeLeft(8)
                        .forward(7)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(50, 52, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROPL;
                break;
            case ARM_DROPL:
                arm.claw.setPosition(0.7);
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .back(8)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(41, 52, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKL;
                break;
            case PARKL:
                arm.dropArm();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(19)
                        .turn(Math.toRadians(180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(49, 33, 0);
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case MID:
                arm.holdPixel();
                TrajectorySequence trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .forward(27)
                        .back(19)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(11, 54, Math.toRadians(270));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1M;
                break;
            case DRIVE_1M:
                arm.raiseArmAuto();
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(20)
                        .forward(20)
                        .turn(Math.toRadians(90))
                        .forward(15)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(46, 34, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1M;
                break;
            case FORWARD_1M:
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeLeft(1)
                        .forward(6)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(52, 35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROPM;
                break;
            case ARM_DROPM:
                arm.claw.setPosition(0.7);
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .back(10)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(42, 35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKM;
                break;
            case PARKM:
                arm.dropArm();
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(23)
                        .turn(Math.toRadians(180))
                        .back(7)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(49, 12, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case RIGHT:
                arm.holdPixel();
                TrajectorySequence trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .forward(20)
                        .turn(Math.toRadians(-45))
                        .forward(8)
                        .waitSeconds(1)
                        .back(8)
                        .turn(Math.toRadians(45))
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(11,50, Math.toRadians(270));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1R;
                break;
            case DRIVE_1R:
                arm.raiseArmAuto();
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .back(10)
                        .strafeLeft(10)
                        .forward(20)
                        .turn(Math.toRadians(90))
                        .forward(23)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(44, 40, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1R;
                break;
            case FORWARD_1R:
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeRight(5)
                        .forward(6)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(50, 35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROPR;
                break;
            case ARM_DROPR:
                arm.claw.setPosition(0.7);
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .back(8)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(42, 35, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKR;
                break;
            case PARKR:
                arm.dropArm();
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(20)
                        .turn(Math.toRadians(180))
                        .back(7)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(49, 15, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case OFF:
                arm.armOff();
                break;
        }
    }
    @Override
    public void stop(){

    }
}

