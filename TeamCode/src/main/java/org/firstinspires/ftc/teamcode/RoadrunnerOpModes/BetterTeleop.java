package org.firstinspires.ftc.teamcode.RoadrunnerOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Arm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Better Teleop", group="Robot")
public class BetterTeleop extends OpMode {
    private SampleMecanumDrive robot;
    private Arm arm;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        robot.drive();
        if (gamepad1.dpad_up) {
            arm.armLiftHigh();
        }
        if (gamepad1.dpad_left) {
            arm.armLiftMed();
        }
        if (gamepad1.dpad_right) {
            arm.armLiftLow();
        }
        if (gamepad1.dpad_down) {
            arm.armLiftDown();
        }
        if (gamepad1.a) {
            arm.armTurnDown();
        }
        if (gamepad1.y) {
            arm.armTurnPLaceHigh();
        }
        if (gamepad1.b) {
            arm.armDrive();
        }
        if (gamepad1.x) {
            arm.armTurnPlace();
        }
        if (gamepad1.right_bumper) {
            arm.dropPixel();
        }
        if (gamepad1.left_bumper) {
            arm.holdPixel();
        }
        //telemetry.update(); nothing to update
    }

    @Override
    public void stop() {}
}