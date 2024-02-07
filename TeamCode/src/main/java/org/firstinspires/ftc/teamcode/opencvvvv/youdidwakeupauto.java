package org.firstinspires.ftc.teamcode.opencvvvv;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name="Red detector", group="robot")
public class youdidwakeupauto extends LinearOpMode {
    OpenCvCamera webcam;
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        whatifyouwokeupdetector detector = new whatifyouwokeupdetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

        /* webcam.openCameraDeviceAsync(
                () -> webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT)
        ); */

        waitForStart();
        switch (detector.getLocation()){
            case LEFT:
                break;
            case RIGHT:
                break;
            case MIDDLE:
                break;
        }
        webcam.stopStreaming();
    }
}