package org.firstinspires.ftc.teamcode.opencvvvv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class whatifyouwokeupdetector extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }
    private Location location;
    static final Rect LEFT_ROI = new Rect(
           new Point(130, 147),
            new Point(190, 187)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(230, 148),
            new Point(290, 188)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.01;
    public whatifyouwokeupdetector(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(127, 69, 65); //min red
        Scalar highHSV = new Scalar (240, 255, 255); //max red (aka pure)
        /*

         */
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        //add pixels together
        //divide by area
        //divide everything by 255 (max value for grayscale pixel)
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() /255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() /255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if(propLeft){
            //middle
            location = Location.MIDDLE;
            telemetry.addData("Prop location: ", "middle");
        }
        else if(propRight){
            //right
            location = Location.RIGHT;
            telemetry.addData("Prop Location: ", "right");
        }
        else{
            //left
            location = Location.LEFT;
            telemetry.addData("Prop Location: ", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        //I don't know if we actually need this section to be honest.
        Scalar colorSomething = new Scalar(255, 0, 0);
        Scalar colorProp = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.MIDDLE? colorProp:colorSomething);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorProp:colorSomething);
        //Questionable section end

        return mat;
    }

    public Location getLocation(){
        return location;
    }
}
