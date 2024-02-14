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
public class BlueRight extends OpMode {
    private enum State{
        //TRAJECTORY_1,
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
        currentPose = new Pose2d(-34, 62, Math.toRadians(270));
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
                currentPose = new Pose2d(-34, 46, Math.toRadians(270));
                robot.setPoseEstimate(currentPose);
                robot.updatePoseEstimate();
                currentState = State.OFF;
                break;
            case MID:
                arm.holdPixel();
                TrajectorySequence trajSeqM = robot.trajectorySequenceBuilder(currentPose)
                        .forward(29)
                        .back(21)
                        .build();
                robot.followTrajectorySequence(trajSeqM);
                currentPose = new Pose2d(-34, 54, Math.toRadians(270));
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
                currentPose = new Pose2d(-34,50, Math.toRadians(270));
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

