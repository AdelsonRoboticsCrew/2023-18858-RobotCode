package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;

import static org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupdetector.Location.LEFT;
import static org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupdetector.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupdetector.Location.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencvvvv.whatifyouwokeupdetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "robot")
public class RedRight extends OpMode{

    WebcamName webcamName;
    private enum State {
        //TRAJECTORY_1, this is going to be used for camera stuff
        MID,
        RIGHT,
        LEFT,
        ARM_LIFT_A ,
        ARM_PICK_UP,
        DRIVE_1L,
        DRIVE_1M,
        DRIVE1_R,
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
        TEST,
        OFF
    }
    private SampleMecanumDrive robot;
    private Arm arm;
    private State currentState;
    private Pose2d currentPose;
    OpenCvCamera webcam;

    // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    whatifyouwokeupdetector detector = new whatifyouwokeupdetector(telemetry);


    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);

        currentPose = new Pose2d(11, -62, Math.toRadians(90));
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
    public void init_loop(){
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
    public void start() { webcam.stopStreaming();}
    @Override
    public void loop(){
        robot.update();
        switch (currentState){
            case LEFT: //spike mark
                arm.holdPixel();
                TrajectorySequence trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .forward(20)
                        .turn(Math.toRadians(45))
                        .forward(5)
                        .waitSeconds(1)
                        .back(5)
                        .turn(Math.toRadians(-45))
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(11, -42, Math.toRadians(90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1L;
                break;
            case RIGHT:
                break;
            case DRIVE_1L:
                arm.raiseArmAuto();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .back(10)
                        .strafeRight(10)
                        .forward(15)
                        .turn(Math.toRadians(-90))
                        .forward(23)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(44, -37, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1L;
                break;
            case FORWARD_1L:
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeLeft(5)
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(49, -32, Math.toRadians(0));
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
                currentPose = new Pose2d(41, -32, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKL;
                break;
            case PARKL:
                arm.dropArm();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(23)
                        .turn(Math.toRadians(180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(47, -9, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
            case MID: //spike mark
                arm.holdPixel();
                TrajectorySequence trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .forward(20)
                        .back(20)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(11, -62, Math.toRadians(90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1M;
                break;
            case DRIVE_1M:
                arm.raiseArmAuto();
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(10)
                    .forward(20)
                    .turn(Math.toRadians(-90))
                    .forward(23)
                    .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(44, -42, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1L;
            case OFF:
                arm.armOff();
                break;
            case TEST:
                arm.armLiftDown();
                arm.armOff();
        }
    }
    @Override
    public void stop(){}
}
