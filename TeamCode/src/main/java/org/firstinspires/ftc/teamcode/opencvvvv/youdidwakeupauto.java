package org.firstinspires.ftc.teamcode.opencvvvv;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name="Prop detector", group="robot")
public class youdidwakeupauto extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        whatifyouwokeupdetector detector = new whatifyouwokeupdetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

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