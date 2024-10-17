package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraBase;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "autonominho", group = "Autonomous")
public class AutonomoExemploSlide extends LinearOpMode {
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    int blue_left = 13 ;
    int blue_right= 12;
    int red_left =  16 ;
    int red_right =15;
    

    @Override
    public void runOpMode() {
        FtcDashboard dashboard;
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 30);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d initialPose = new Pose2d(36, 60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(0, 34, Math.toRadians(90)), Math.toRadians(270))
                .setTangent(95)
                .splineToLinearHeading(new Pose2d(48.5, 40, Math.toRadians(270)), Math.toRadians(0))
                .setReversed(true)
                .splineTo(new Vector2d(52, 51), Math.toRadians(45));

        waitForStart();

        if (isStopRequested()) return;

        // verificar se a tag foi detectada durante o init
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == blue_left || tag.id == blue_right || tag.id == red_left || tag.id == red_right) {
                tagOfInterest = tag;
                break;
            }
        }

        Action trajectoryActionChosen = null;
        if (tagOfInterest != null) {
            if (tagOfInterest.id == blue_left) {
                trajectoryActionChosen = tab1.build();
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}