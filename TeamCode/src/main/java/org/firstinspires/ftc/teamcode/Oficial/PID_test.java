package org.firstinspires.ftc.teamcode.Oficial;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PID_test extends OpMode {

    DcMotorEx motorArmLeft;
    public PIDController controller;
    public static double p = 0.1, i = 0, d = 0.00057;
    public static double f = 0.043;
    public static int targetArm = 0;

    @Override
    public void  init(){
    controller = new PIDController(p, i, d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    motorArmLeft = hardwareMap.get(DcMotorEx.class,"arm");
    motorArmLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = motorArmLeft.getCurrentPosition();

        //Marcao ajeita aqui o limitador
        if (Math.abs(armPos - targetArm) <= 2) {
            motorArmLeft.setPower(0);
        } else {
            double pid = controller.calculate(armPos, targetArm);
            double ff = Math.cos(Math.toRadians(armPosition())) * f;

            double power = pid + ff;

            //Define a velocidade aqui
            power = Math.max(Math.min(power, 0.8), -0.8);

            motorArmLeft.setPower(power);

            telemetry.addData("Posição Atual", armPos);
            telemetry.addData("Alvo", targetArm);
            telemetry.update();
        }
    }

    double armPosition()
    {
        //Eu repeti o armPos aqui
        int armPos = motorArmLeft.getCurrentPosition();
        double CPR = 700.0;
        double revolutions = armPos/CPR;
        double angle = revolutions* 360;
        return angle % 360;

    }
}




