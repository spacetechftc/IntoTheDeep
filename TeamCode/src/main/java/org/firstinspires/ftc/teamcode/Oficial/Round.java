package org.firstinspires.ftc.teamcode.Oficial;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Oficial : Round", group="Oficial")

public class Round extends LinearOpMode {

    private DcMotor arm = null;
    private DcMotor slide = null;
    private Servo claw = null;
    double prev_error;
    double integral;
    double targetPosition_slide;
    double targetPosition_arm;
    boolean solto = true;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // declara motores do InTAKE
        arm  = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");

        // declara motores do Drive Train
        DcMotor mEF = hardwareMap.dcMotor.get("mEF");
        DcMotor mET = hardwareMap.dcMotor.get("mET");
        DcMotor mDF = hardwareMap.dcMotor.get("mDF");
        DcMotor mDT = hardwareMap.dcMotor.get("mDT");

        slide.setDirection(DcMotor.Direction.REVERSE);

        mDF.setDirection(DcMotor.Direction.REVERSE);
        mDT.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition_slide = 100;
        targetPosition_arm = 10;
        boolean claw_cc = true;


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * 1.1;
            double rx = gamepad2.right_stick_x;

            double armPower;
            double slidePower;

            armPower = -gamepad1.left_stick_y/2;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double mEF_Power = (y + x + rx) / denominator;
            double mET_Power = (y - x + rx) / denominator;
            double mDF_Power = (y - x - rx) / denominator;
            double mDT_Power =  (y + x - rx) / denominator;

            mEF.setPower(mEF_Power);
            mET.setPower(mET_Power);
            mDT.setPower(mDT_Power);
            mDF.setPower(mDF_Power);
            arm.setPower(armPower);



            if(gamepad1.left_trigger > 0 && solto == true)
            {
                if (claw_cc == true){
                    claw.setPosition(0.5);
                    claw_cc = false;
                } else if (claw_cc == false) {
                    claw.setPosition(1.0);
                    claw_cc= true;
                }

            }

            if(gamepad1.left_trigger == 0){
                solto = true;
            } else {
                solto = false;
            }

            if(gamepad1.left_bumper == true)
            {
                pid("slide", targetPosition_slide, 1,0,0.5);
                ChangeSlidePosition();

            }

            if(gamepad1.x == true)
            { if (armPosition() >= 140){
                targetPosition_arm = 10;
            } else {
                targetPosition_arm = 140;
            }
                pid("arm", targetPosition_arm, 0.25,0,1.5);

            }


            telemetry.update();

        }
    }

    public void pid(String motor, double target, double KP, double KI, double KD)
    {

        if(motor.equals("arm")){

            integral = 0;
            prev_error = 0;

            if(targetPosition_arm == 10 ){
                while(!(armPosition() <= targetPosition_arm))
                {
                    targetPosition_arm = target;
                    double erro = targetPosition_arm - armPosition();
                    double P = KP * erro;
                    integral = integral + (prev_error * 0.1);
                    double I = KI * integral;
                    double derivada = (erro - prev_error) / 0.1;
                    double D = KD * derivada;
                    double output = P + I + D;
                    prev_error = erro;

                    arm.setPower(output/2);
                }
                arm.setPower(0);
            } else{
                while(!(armPosition() >= targetPosition_arm))
                {
                    targetPosition_arm = target;
                    double erro = targetPosition_arm - armPosition();
                    double P = KP * erro;
                    integral = integral + (prev_error * 0.1);
                    double I = KI * integral;
                    double derivada = (erro - prev_error) / 0.1;
                    double D = KD * derivada;
                    double output = P + I + D;
                    prev_error = erro;

                    arm.setPower(output/2);
                }
                arm.setPower(0);
            }

        } else if(motor.equals("slide"))
        {
            integral = 0;
            prev_error = 0;

            while(!(slidePosition() == targetPosition_slide )){
                targetPosition_slide = target;
                double erro = targetPosition_slide - slidePosition();
                double P = KP * erro;
                integral = integral + (prev_error * 0.1);
                double I = KI * integral;
                double derivada = (erro - prev_error) / 0.1;
                double D = KD * derivada;
                double output = P + I + D;
                prev_error = erro;

                slide.setPower(output);
            }
            slide.setPower(0);
        }
    }

    double slidePosition()
    {
        double CPR = 288.0;
        int position = slide.getCurrentPosition();
        double revolutions = position/CPR;
        double angle = revolutions* 360;
        return angle % 360;

    }

    public void ChangeSlidePosition()
    {
        if(targetPosition_slide == 0){
            targetPosition_slide =100;
        } else if (targetPosition_slide == 100){
            targetPosition_slide = 0;
        }
    }


    double armPosition()
    {
        double CPR = 560.0;
        int position = arm.getCurrentPosition();
        double revolutions = position/CPR;
        double angle = revolutions* 360;
        return angle % 360;

    }

}