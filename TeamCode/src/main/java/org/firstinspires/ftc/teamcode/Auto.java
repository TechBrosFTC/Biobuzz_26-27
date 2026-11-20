package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "Auto")
public class Auto extends LinearOpMode {

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-64, 12, Math.toRadians(0));
        Pose2d shotpose = new Pose2d(64, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder estacionar = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-42 , 45), Math.toRadians(0))
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(36 , 60), Math.toRadians(0));

        TrajectoryActionBuilder gardens = drive.actionBuilder(initialPose)
                .lineToX(-60)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-61 , 62))
                .waitSeconds(0.5)
                .lineToY(62)
                .strafeTo(new Vector2d(40, 54));


        TrajectoryActionBuilder flor = drive.actionBuilder(shotpose)
                .strafeToSplineHeading(new Vector2d(40, 54), Math.toRadians(90))
                .strafeTo(new Vector2d(-24 , 48))
                .waitSeconds(0.3)
                .lineToY(56);

        TrajectoryActionBuilder second_shot = drive.actionBuilder(new Pose2d(40, 54, Math.toRadians(90)))
                .waitSeconds(0.3)
                .strafeToSplineHeading(new Vector2d(50, 24), Math.toRadians(135))
                .strafeToSplineHeading(new Vector2d(62, 12), Math.toRadians(180));

        TrajectoryActionBuilder third_shot = drive.actionBuilder(new Pose2d(-24, 56, Math.toRadians(90)))
                .lineToY(50)
                .strafeToSplineHeading(new Vector2d(40, 54), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(50, 24))
                .strafeTo(new Vector2d(58, 12))
                .waitSeconds(0.5);


        Action flower = flor.build();
        Action garden = gardens.build();
        Action shoot2 = second_shot.build();
        Action shoot3 = third_shot.build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        garden,
                        shoot2,
                        flower,
                        shoot3

                )
        );


    }
}
