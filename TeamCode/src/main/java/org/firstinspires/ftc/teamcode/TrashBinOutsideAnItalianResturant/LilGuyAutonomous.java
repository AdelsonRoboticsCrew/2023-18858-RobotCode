package org.firstinspires.ftc.teamcode.TrashBinOutsideAnItalianResturant;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name="Robot: General autonomous", group="Robot")
public class LilGuyAutonomous extends OpMode{

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    public BNO055IMU imu;
    private int autoThingsCount = 0;
    static final double TICKS_PER_REV = ((((1+(46/17))) * (1+(46/11))) * 28);
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double TICKS_PER_INCH = (TICKS_PER_REV/(WHEEL_DIAMETER_INCHES * Math.PI));
    public void motorSpeed(double speed){
        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newlfTarget;
        int newlbTarget;
        int newrfTarget;
        int newrbTarget;
        if (!leftBack.isBusy() && autoThingsCount == 0) {
            newlfTarget = leftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newlbTarget = leftBack.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newrfTarget = rightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            newrbTarget = rightBack.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            leftFront.setTargetPosition(newlfTarget);
            leftBack.setTargetPosition(newlbTarget);
            rightFront.setTargetPosition(newrfTarget);
            rightBack.setTargetPosition(newrbTarget);

            leftFront.setMode(RunMode.RUN_TO_POSITION);
            leftBack.setMode(RunMode.RUN_TO_POSITION);
            rightFront.setMode(RunMode.RUN_TO_POSITION);
            rightBack.setMode(RunMode.RUN_TO_POSITION);
            autoThingsCount++;
            motorSpeed(speed);
        }

    }

    public void init(){
        telemetry.addData("Status", "Initialized");
        leftBack = hardwareMap.get(DcMotor.class, "left back");
        leftFront = hardwareMap.get(DcMotor.class, "left front");
        rightBack = hardwareMap.get(DcMotor.class, "right back");
        rightFront = hardwareMap.get(DcMotor.class, "right front");


        //make sure the directions are correct after testing
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    public void init_loop(){

    }

    public void start(){

    }

    public void loop(){
        double MotorAngleCompensationFactor = 1/((Math.sin((Math.atan(7/6.5))*Math.sin(Math.PI/4)+(Math.cos((Math.atan(7/6.5)))*Math.cos(Math.PI/4)))));
        MotorAngleCompensationFactor*=(1.276143212);//MotorAngleCompensationFactor*0.75
        double inchValue = (Math.sqrt(7.0*7.0 + 6.5*6.5)*(2*Math.PI)/4)*MotorAngleCompensationFactor;
        encoderDrive(1, 39.37, 39.37); //can change num values
        //encoderDrive(1.0, inchValue, -inchValue);
        telemetry.addData("gyro", this.imu.getAngularOrientation().firstAngle);
    }

    public void stop(){

    }
}


