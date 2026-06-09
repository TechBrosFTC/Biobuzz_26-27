package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {
    private IMU imu;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;
   
    double power = 1;
    double erro;
    double proporcional;
    double direcao = 0;
    double ultimoerro = 0;
    double curvapower = 0.4;
    double alvo = 0;
    boolean curva = false;
    double kp = 2.2; //5 Bom, 6 Legal-+
    double kd = 2; // 35 Fica pouco, 40 Fica Pouco, 45 Quase lá
    double ki = 0.0006;
    String state = "";
    double lastpower = 0;
    float f,t,d,e;
    double multiplicador = 0.6;
    //////////////////////////////////////////////////////////////////Norte Abosluto/////////////////////////////////////////////////////////
    double giroabsoluto;
    int direcaoabsoluta;
    int index;
    String direcaomovimento = "";
    ///////////////////////////////////////////////////////////////////////
    boolean modolento = false;
    double posicaoatualgarra = 0;
    double proximaposicaogarra = 0;
    double multiplicadorcurva = 1;
    String dir = "";

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");
        RevHubOrientationOnRobot revOrientaion = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientaion));
        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        imu.resetYaw();
        Thread telemetria = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
/////////////////////////////////////////////////////Telemetria/////////////////////////////////////////////////////////////////////
                    telemetry.addData("giro", gyro());
                    telemetry.addData("direcao:", direcao);
                    telemetry.addData("controle y:", gamepad1.left_stick_y);
                    telemetry.addData("controle x:", gamepad1.left_stick_x);
                    telemetry.addData("power:", power);
                    telemetry.addData("index:", index);
                    telemetry.addData("Direção", dir);
                    telemetry.addData("concatenado", direcaomovimento);
                    telemetry.addData("giro absoluto:", giroabsoluto);
                    telemetry.addData("posicao abosluta:", direcaoabsoluta);
                    telemetry.addData("modolento", modolento);
                    telemetry.addData("posicao atual:", posicaoatualgarra);
                    telemetry.addData("proxima posicao", proximaposicaogarra);
                    telemetry.addData("state", state);
                    telemetry.update();

                }
            }
        });
        waitForStart();
        imu.resetYaw();
        giroabsoluto = 0;

        telemetria.start();
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {
                if (gamepad1.left_stick_y > 0) {
                    f = 0;
                    t = Math.abs(gamepad1.left_stick_y);
                } else {
                    f = Math.abs(gamepad1.left_stick_y);
                    t = 0;
                }
            } else {
                f = 0;
                t = 0;
            }
            if (gamepad1.left_stick_x != 0) {
                if (gamepad1.left_stick_x > 0) {
                    e = 0;
                    d = Math.abs(gamepad1.left_stick_x);
                } else {
                    e = Math.abs(gamepad1.left_stick_x);
                    d = 0;
                }
            } else {
                d = 0;
                e = 0;
            }

            if (f > t && f > d && f > e) { //andar para frente
                dir = "Frente";
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                double derivativa = (erro - ultimoerro) * kd;
                double integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador, direcao);
                ultimoerro = erro;
                state = "andando";
            }

            if (t > f && t > d && t > e) { //andar para trás
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                double derivativa = (erro - ultimoerro) * kd;
                double integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                lastpower = gamepad1.left_stick_y * multiplicador;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador * -1, direcao);
                ultimoerro = erro;
                state = "andando";
            }

            if (d > f && d > t && d > e) { //andar para Direita
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                double derivativa = (erro - ultimoerro) * 1;
                double integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumdireitabase(Math.abs(gamepad1.left_stick_x), -direcao);
                state = "andando";
            }
            if (e > f && e > t && e > d) { //andar para Esquerda
                if (state.equals("parado")) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                double derivativa = (erro - ultimoerro) * 1;
                double integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumesquerdabase(Math.abs(gamepad1.left_stick_x), direcao);
                state = "andando";
            }
            if (f == 0 && t == 0 && d == 0 && e == 0) {
                parar();
                state = "parado";
            }
            if (gamepad1.right_stick_x > 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva * -1);
                direitaTras.setPower(curvapower * multiplicadorcurva * -1);
                esquerdaFrente.setPower(curvapower * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * multiplicadorcurva);
                curva = true;
            } else if (gamepad1.right_stick_x < 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva);
                direitaTras.setPower(curvapower * multiplicadorcurva);
                esquerdaFrente.setPower(curvapower * -1 * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * -1 * multiplicadorcurva);
                curva = true;
            } else {
                if (curva) {
                    parar();
                    sleep(50);
                    imu.resetYaw();
                    curva = false;
                    state = "parado";
                }
            }
            if(gamepad1.cross){
                multiplicador = 0.5;
                multiplicadorcurva = 0.5;
                modolento = true;
                sleep(100);
            }else{
                multiplicador = 0.8;
                multiplicadorcurva = 1;
                modolento = false;
                sleep(100);
            }
        }
    }
    /////////////////////////////////////////////////////funções///////////////////////////////////////////////////////////////////////
    public void mecanumesquerdabase(double power, double direcao){
        direitaFrente.setPower(power);
        direitaTras.setPower((power-direcao)*-1);
        esquerdaFrente.setPower(power*-1);
        esquerdaTras.setPower((power-direcao));
    }

    public void mecanumdireitabase(double power, double direcao){
        if (direcao < 0){
            direitaFrente.setPower((power-direcao)*-1);
            direitaTras.setPower(power);
            esquerdaFrente.setPower((power-direcao));
            esquerdaTras.setPower(power*-1);
        }else{
            direitaFrente.setPower(power*-1);
            direitaTras.setPower((power-direcao));
            esquerdaFrente.setPower(power);
            esquerdaTras.setPower((power-direcao)*-1);
        }
    }
    public void movDirecionado(double power, double direcao){
        if (power < 0) {
            if (direcao >= 0){
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }else{
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);
            }
        }else{
            if (direcao >= 0) {
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);

            } else {
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }
        }
    }

    public void parar(){
        direitaFrente.setPower(0);
        direitaTras.setPower(0);
        esquerdaFrente.setPower(0);
        esquerdaTras.setPower(0);
    }

    public double gyro(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
