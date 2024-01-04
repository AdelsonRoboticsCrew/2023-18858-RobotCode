package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90)))
                                .waitSeconds(0.5)
                                .forward(25)
                                .strafeRight(33)
                                .turn(Math.toRadians(-90))
                                .forward(4)
                                .waitSeconds(0.5)
                                //.back(4)
                                .back(80)
                                .turn(Math.toRadians(180))
                                .forward(25)
                                .back(80)
                                .turn(Math.toRadians(180))
                                .forward(25)
                                .waitSeconds(0.5)
                                .strafeLeft(28)
                                .turn(Math.toRadians(180))
                                .back(6)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}