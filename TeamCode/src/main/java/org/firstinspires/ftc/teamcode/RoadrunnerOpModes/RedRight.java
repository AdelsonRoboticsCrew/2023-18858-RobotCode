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

    private enum State {
        //TRAJECTORY_1, this is going to be used for camera stuff
        MID,
        RIGHT,
        LEFT,
        ARM_LIFT_A ,
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
                        .forward(8)
                        .waitSeconds(1)
                        .back(8)
                        .turn(Math.toRadians(-45))
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(11, -42, Math.toRadians(90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1L;
                break;
            case DRIVE_1L:
                arm.raiseArmAuto();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .back(10)
                        .strafeRight(10)
                        .forward(20)
                        .turn(Math.toRadians(-90))
                        .forward(23)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(44, -32, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1L;
                break;
            case FORWARD_1L:
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeLeft(7)
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(50, -26, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROPL;
                break;
            case ARM_DROPL:
                arm.claw.setPosition(0.7);
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .back(9)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(41, -26, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKL;
                break;
            case PARKL:
                arm.dropArm();
                trajSeqL = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(17)
                        .turn(Math.toRadians(180))
                        .back(6)
                        .build();
                robot.followTrajectorySequence(trajSeqL);
                currentPose = new Pose2d(47, -9, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case MID: //spike mark
                arm.holdPixel();
                TrajectorySequence trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .forward(28)
                        .back(20)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(11, -54, Math.toRadians(90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1M;
                break;
            case DRIVE_1M:
                arm.raiseArmAuto();
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .strafeRight(20)
                    .forward(20)
                    .turn(Math.toRadians(-90))
                    .forward(15)
                    .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(46, -34, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1M;
                break;
            case FORWARD_1M:
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(51, -34, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.ARM_DROPM;
                break;
            case ARM_DROPM:
                arm.claw.setPosition(0.7);
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(1.5)
                        .back(9)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(42, -34, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKM;
                break;
            case PARKM:
                arm.dropArm();
                trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(22)
                        .turn(Math.toRadians(180))
                        .back(8)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(49, -12, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case RIGHT:
                arm.holdPixel();
                TrajectorySequence trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(0.5)
                        .forward(23)
                        .turn(Math.toRadians(-45))
                        .forward(7)
                        .waitSeconds(1)
                        .back(7)
                        .turn(Math.toRadians(45))
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(11, -39, Math.toRadians(90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.DRIVE_1R;
                break;
            case DRIVE_1R:
                arm.raiseArmAuto();
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .back(15)
                        .strafeRight(30)
                        .forward(17)
                        .turn(Math.toRadians(-90))
                        .forward(5)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(46, -37, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.FORWARD_1R;
                break;
            case FORWARD_1R:
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .waitSeconds(2)
                        .strafeRight(8)
                        .forward(4)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(50, -42, Math.toRadians(0));
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
                currentPose = new Pose2d(42, -42, Math.toRadians(0));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.PARKR;
                break;
            case PARKR:
                arm.dropArm();
                trajSeqR = robot.trajectorySequenceBuilder(currentPose)
                        .strafeLeft(30)
                        .turn(Math.toRadians(180))
                        .back(7)
                        .build();
                robot.followTrajectorySequence(trajSeqR);
                currentPose = new Pose2d(49, -12, Math.toRadians(-90));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case OFF:
                arm.armOff();
                break;
            /* case TEST:
                arm.armLiftDown();
                arm.armOff(); */
        }
    }
    @Override
    public void stop(){}
}
