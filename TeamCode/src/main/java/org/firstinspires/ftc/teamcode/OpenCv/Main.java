package org.firstinspires.ftc.teamcode.OpenCv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

public class Main {

    private static final double FEET_PER_METER = 3.28084;

    public void runAutonomous(AprilTagDetection tagOfInterest) {
        if (tagOfInterest != null) {
            // Exemplo de lógica baseada na ID da tag
            if (tagOfInterest.id == 13) {
                // lógica para blue_left
                // Por exemplo: mover robô para uma posição
            } else if (tagOfInterest.id == 12) {
                // lógica para blue_right
            } else if (tagOfInterest.id == 16) {
                // lógica para red_left
            } else if (tagOfInterest.id == 15) {
                // lógica para red_right
            }

            // Lógica baseada na posição da tag
            if (tagOfInterest.pose.x <= 20) {
                // Fazer algo quando x <= 20
            } else if (tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                // Fazer algo quando x entre 20 e 50
            } else if (tagOfInterest.pose.x >= 50) {
                // Fazer algo quando x >= 50
            }
        }
    }

    public void tagToTelemetry(AprilTagDetection detection, FtcDashboard dashboard, TelemetryPacket packet) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        packet.put("Detected Tag ID", detection.id);
        packet.put("Translation X (feet)", detection.pose.x * FEET_PER_METER);
        packet.put("Translation Y (feet)", detection.pose.y * FEET_PER_METER);
        packet.put("Translation Z (feet)", detection.pose.z * FEET_PER_METER);
        packet.put("Rotation Yaw (degrees)", rot.firstAngle);
        packet.put("Rotation Pitch (degrees)", rot.secondAngle);
        packet.put("Rotation Roll (degrees)", rot.thirdAngle);

        dashboard.sendTelemetryPacket(packet);
    }
}
