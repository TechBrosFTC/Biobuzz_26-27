package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;




@Config
@TeleOp(name = "Ateleop1")
public class Ateleop1_Blue extends LinearOpMode {

    // --- SENSORES E NAVEGAÇÃO / CHASSI ---
    private IMU imu;
    private Limelight limelight;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    private Servo servoRampa;

    // --- MOTORES DE MECANISMOS ---
    private DcMotorEx shooter;
    private DcMotorEx transfer;
    private DcMotorEx intake;
    private DcMotorEx passador;

    private GoBildaPinpointDriver odo;

    // --- SERVOS E TORRETA INSTANCIADA ---
    private Servo servorampa;
    private Servo servoflor;
    private CRServo torreta;
    private CRServo torreta1;
    private DcMotorEx encoder_torreta;
    private Turret turretObjeto;



    private Servo hood;

    // --- ALVO DE TRACKING DA TORRETA (60, 60 em Polegadas/MM) ---


    // --- CONFIGURAÇÕES DE TRANSFER E INTAKE ---
    public static String TRANSFER_NAME = "transfer";
    public static String INTAKE_NAME = "intake";
    public static boolean REVERSE_INTAKE = false;

    // --- GANHOS PIDF TUNÁVEIS NO FTC DASHBOARD ---
    // Shooter PIDF
    public static double SHOOTER_P = 0.0009;
    public static double SHOOTER_I = 0.00001;

    int shot_count = 0;

    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 0.000205;
    public static double SHOOTER_RECOVERY = 0.01;
    public static double SHOOTER_MAX_BOOST = 0.2;
    public static double SHOOTER_A = 31.4;
    public static double SHOOTER_B = 2650;

    /* public static double SHOOTER_P = 120;
    public static double SHOOTER_I = 0;

    int shot_count = 0;

    public static double SHOOTER_D = 0.01;
    public static double SHOOTER_F = 13.4291;
    public static double SHOOTER_A = 31.4;
    public static double SHOOTER_B = 2750;*/

    // Transfer PIDF
    public static double TRANSFER_P = 80;
    public static double TRANSFER_I = 0.0;
    public static double TRANSFER_D = 0.0;
    public static double TRANSFER_F = 13;

    // Passador PIDF
    public static double PASSADOR_P = 80;
    public static double PASSADOR_I = 0.0;
    public static double PASSADOR_D = 0.0;
    public static double PASSADOR_F = 12.5;

    // --- POSIÇÕES AJUSTÁVEIS DO SERVO POSICIONAL ---
    public static double POS_ABERTO = 1.0;
    public static double POS_FECHADO = 0.0;

    // --- CONTROLES DE BOTÃO E ESTADOS DA VARREDURA DA TORRETA ---
    private boolean abriuServoRampa = false;
    private boolean servoEstaAberto = false;
    private boolean anteriorX = false;

    private double searchDirection = -1.0;  // Sentido atual da busca da torreta entre os limites

    // --- GANHOS DO PID DE HEADING DA BASE ---
    double kp = 3;
    double kd = 0;
    double ki = 0;

    double turret_kp = 0.01, turret_ki = 0, turret_kd = 0.001, turret_kf = 0;
    double turret_lasterror = 0, turret_integral = 0;

    double erro;
    double proporcional;
    double integral = 0;
    double ultimoErro = 0;
    double alvo = 0;

    boolean passo = false;

    double rx = 0;
    double multiplicador = 0.8;
    double curvapower = 0.65;

    String state = "parado";
    int index;
    public static double incremento = 0;
    double shot = 1;
    double countshot = 1;
    public static double shooterVelocity = 2765;
    double shooterTicksPerRev = 28.0;
    public static int velinicial = 2800;
    public static double currentShooterRPM = 0;
    boolean atirando = false;

    // Direções discretas para translação
    private static final double[][] DIRECTIONS = {
            { 1,  0}, // direita
            { 1,  1}, // frente-direita
            { 0,  1}, // frente
            {-1,  1}, // frente-esquerda
            {-1,  0}, // esquerda
            {-1, -1}, // trás-esquerda
            { 0, -1}, // trás
            { 1, -1}  // trás-direita
    };
    Thread mira = new Thread(new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()){
                double powerTorretaManual = gamepad2.left_stick_x;
                if (Math.abs(powerTorretaManual) > 0.1) {
                    // Override manual no Gamepad 2
                    torreta.setPower(powerTorretaManual);
                    torreta1.setPower(powerTorretaManual);
                    shooterVelocity = SHOOTER_B;
                } else if (limelight.isValid) {
                    // TAG ENCONTRADA: Rastreia com PID pelo caminho mais curto
                    double error = limelight.getTx()+1.5;
                    if(Math.abs(error) >= 3){
                        double proporcional = turret_kp * error;
                        turret_integral += error*turret_ki;
                        double derivada = (error - turret_lasterror) * turret_kd;
                        turret_lasterror = error;
                        double feedforward = turret_kf * Math.signum(error);
                        double output = proporcional + turret_integral + derivada;
                        torreta.setPower(output);
                        torreta1.setPower(output);
                        shooterVelocity = SHOOTER_A*(limelight.getTy()-27) + SHOOTER_B;
                    }
                } else {
                    torreta.setPower(0.0);
                    torreta1.setPower(0.0);
                    shooterVelocity = SHOOTER_B;
                }
            }
        }
    });

    CustomShooterV2 atirador;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        // Inicialização do IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));

        hood = hardwareMap.get(Servo.class, "hood");

        passador = hardwareMap.get(DcMotorEx.class, "passador");
        passador.setDirection(DcMotorSimple.Direction.FORWARD);
        passador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passador.setVelocityPIDFCoefficients(PASSADOR_P, PASSADOR_I, PASSADOR_D, PASSADOR_F);

        // Mapeamento dos motores do Chassi
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        direitaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        direitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);

        direitaFrente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direitaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdaFrente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean pause = false;

        // Mapeamento e Inicialização dos motores do Teste Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Mapeamento de Transfer e Intake
        transfer = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setVelocityPIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);

        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servorampa = hardwareMap.get(Servo.class, "servorampa");

        // --- INSTANCIAÇÃO DA CLASSE TURRET (SERVOS TORRETA E TORRETA1) ---
        torreta = hardwareMap.get(CRServo.class, "torreta");
        torreta1 = hardwareMap.get(CRServo.class, "torreta1");

        torreta1.setDirection(DcMotorSimple.Direction.FORWARD);
        torreta.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder_torreta = hardwareMap.get(DcMotorEx.class, "intake");

        double ticksPerDegreeTurret = 8192.0 / 36;
        turretObjeto = new Turret(torreta, torreta1, encoder_torreta, ticksPerDegreeTurret, DcMotorSimple.Direction.REVERSE);
        VoltageSensor voltagem;
        /*
         * Hardware do Pinpoint
         */
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderResolution(76.964, DistanceUnit.MM);
        odo.setOffsets(-55, -120, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        voltagem = hardwareMap.voltageSensor.iterator().next();
        // Inicializa o Pinpoint na Posição Inicial (10, 10, Heading 0)
        odo.initialize();
        odo.setPosition(new Pose2D(INCH, 10.0, 10.0, AngleUnit.DEGREES, 0.0));

        telemetry.addData("Status", "Inicializado com sucesso!");
        telemetry.update();

        limelight = new Limelight(hardwareMap, 0);

        servoflor = hardwareMap.get(Servo.class, "servoflor");


        double fator = 0.1;
        double kpLimelight = 0.2;
        telemetry.setMsTransmissionInterval(11);

        boolean passando = false;
        servorampa.setPosition(1);

        // =========================================================================
        // SINTONIA SIMULTÂNEA DO FEEDFORWARD (F) DO SHOOTER E PASSADOR
        // =========================================================================

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
       /* while (opModeIsActive()){
            telemetry.addLine("=== CALIBRAÇÃO SIMULTÂNEA DO FEEDFORWARD (F) ===");
            telemetry.addLine("Acelerando Shooter e Passador a 100% para medir velocidades máximas...");
            telemetry.update();

            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(1.0);



            passador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            passador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            passador.setPower(1.0);

            ElapsedTime calibTimer = new ElapsedTime();
            calibTimer.reset();

            // Aguarda 500ms para ambos vencerem a inércia
            while (opModeIsActive() && calibTimer.milliseconds() < 500) {
                sleep(10);
            }

            // Amostra a velocidade de ambos por 3 segundos para encontrar os valores máximos
            double maxVelTicksShooter = 0.0;
            double maxVelTicksPassador = 0.0;
            calibTimer.reset();
            while (opModeIsActive() && calibTimer.seconds() < 3.0) {
                double currentVelShooter = Math.abs(shooter.getVelocity());
                if (currentVelShooter > maxVelTicksShooter) {
                    maxVelTicksShooter = currentVelShooter;
                }

                double currentVelPassador = Math.abs(passador.getVelocity());
                if (currentVelPassador > maxVelTicksPassador) {
                    maxVelTicksPassador = currentVelPassador;
                }

                double currentRPMShooter = (currentVelShooter * 60.0) / shooterTicksPerRev;
                double currentRPMPassador = (currentVelPassador * 60.0) / shooterTicksPerRev;

                telemetry.addData("Shooter RPM Atual", "%.2f RPM", currentRPMShooter);
                telemetry.addData("Shooter Ticks/s", "%.2f", currentVelShooter);
                telemetry.addData("Passador RPM Atual", "%.2f RPM", currentRPMPassador);
                telemetry.addData("Passador Ticks/s", "%.2f", currentVelPassador);
                telemetry.update();
                sleep(20);
            }

            // Desliga ambos os motores
            shooter.setPower(0.0);
            passador.setPower(0.0);

            // Fórmula da Documentação REV / FTC: F = 32767 / maxVelocityInTicksPerSecond
            if (maxVelTicksShooter > 0) {
                SHOOTER_F = 32767.0 / maxVelTicksShooter;
            }
            if (maxVelTicksPassador > 0) {
                PASSADOR_F = 32767.0 / maxVelTicksPassador;
            }
            break;


        }
        mira.start();
        telemetry.addData("Status", "Calibração Concluída!");
        telemetry.addData("SHOOTER_F Configurado", "%.4f", SHOOTER_F);
        telemetry.addData("PASSADOR_F Configurado", "%.4f", PASSADOR_F);
        telemetry.update();
        sleep(50);


        // Configura os motores para utilizar o PIDF com os Fs calibrados



        passador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        passador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passador.setVelocityPIDFCoefficients(PASSADOR_P, PASSADOR_I, PASSADOR_D, PASSADOR_F);*/

        atirador = new CustomShooterV2(shooter);


        while (opModeIsActive()) {
            // =======================================================
            // 1. PINPOINT ODOMETRY E RESET NO BOTÃO X (GAMEPAD 1)
            // =======================================================


            // =======================================================
            // 2. TRACKING E BUSCA AUTOMÁTICA DA TORRETA (PROCURA DE TAG)
            // =======================================================
            limelight.update();

            if(gamepad1.y){
                servoflor.setPosition(0);
            }
            else if(gamepad1.x){
                servoflor.setPosition(1);
            }

            double powerTorretaManual = gamepad2.left_stick_x;

            if (Math.abs(powerTorretaManual) > 0.1) {
                // Override manual no Gamepad 2
                torreta.setPower(powerTorretaManual);
                torreta1.setPower(powerTorretaManual);
                shooterVelocity = SHOOTER_B;
            } else if (limelight.isValid) {
                // TAG ENCONTRADA: Rastreia com PID pelo caminho mais curto
                double error = limelight.getTx();
                if(Math.abs(error) >= 4){
                    double proporcional = turret_kp * error;
                    turret_integral += error*turret_ki;
                    double derivada = (error - turret_lasterror) * turret_kd;
                    turret_lasterror = error;
                    double feedforward = turret_kf * Math.signum(error);
                    double output = proporcional + turret_integral + derivada;
                    torreta.setPower(output);
                    torreta1.setPower(output);
                    shooterVelocity = SHOOTER_A*(limelight.getTy()-27) + SHOOTER_B;
                }
            } else {
                torreta.setPower(0.0);
                torreta1.setPower(0.0);
                shooterVelocity = SHOOTER_B;
            }

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double power = Math.min(Math.hypot(x, y), 1);
            double[] direction = getDirection(x, y);

            double moveX = direction[0] * power * multiplicador;
            double moveY = direction[1] * power * multiplicador;

            double turnInput = gamepad1.right_stick_x * curvapower;

            mecanum(moveX, moveY, rx + turnInput);



            /*shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
            double shooterTicksPerSec = (shooterVelocity * shot * shooterTicksPerRev) / 60.0;
            shooter.setVelocity(shooterTicksPerSec);
            currentShooterRPM = (shooter.getVelocity() * 60.0) / shooterTicksPerRev;*/
            // Servo Transfer (Gamepad 1)
            atirador.setPIDF( SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F, SHOOTER_RECOVERY, SHOOTER_MAX_BOOST);
            atirador.setTargetRPM(shooterVelocity);
            atirador.update();


            // =======================================================
            // 5. MOTORES TRANSFER E INTAKE (GATILHOS RT / LT)
            // =======================================================
            double transferPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(transferPower) > 0.15) {
                transfer.setPower(transferPower);
                intake.setPower(transferPower);
            } else if(!passando && !atirando){
                transfer.setPower(0.0);
                intake.setPower(0.0);
            }

            if (gamepad1.right_bumper && !passando) {
                if (atirador.isReady()) {
                    passador.setVelocity(convertRPMtoTicks(5500));
                    timer.reset();
                    passando = true;
                }
            }
            if (passando && passador.getVelocity() > convertRPMtoTicks(4500)) {
                servorampa.setPosition(0.51);
                sleep(500);
                transfer.setVelocity(convertRPMtoTicks(1850));
                intake.setPower(0.6);
                atirando = true;
                passando = false;
            }
            if(atirando){
                if(timer.time() > 3){
                    atirando = false;
                }
                /*if(transfer.getCurrentPosition() > (28*4)*2 && !passo){
                    transfer.setPower(0);
                    transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    timer.reset();
                    passo = !passo;
                    shot_count ++;
                }
                if(timer.time() > 0.5 && passo){
                    transfer.setVelocity(3000);
                    passo = !passo;
                    shot_count ++;
                }
                if(shot_count > 8){
                    transfer.setVelocity(0);
                    shot_count = 0;
                    atirando = false;
                }*/
            }
            if(!atirando && !passando){
                servorampa.setPosition(1);
                passador.setVelocity(0);
            }

            /*if (gamepad1.right_bumper && !atirando) {
                if (currentShooterRPM >= shooterVelocity - 100) {
                    passador.setVelocity(convertRPMtoTicks(5200));
                    timer.reset();
                    passando = true;
                }
            }
            if (passando) {
                if (passador.getVelocity() > convertRPMtoTicks(4000)) {
                    atirando = true;
                    passando = false;
                    servorampa.setPosition(0.5);
                    sleep(500);
                    pause = false;
                    shot_count = 0;
                    timer.reset();
                }
            }
            if (atirando) {
                if (timer.seconds() < 0.1 && !pause ) {
                    transfer.setPower(0.75);
                    intake.setPower(1);
                }else if(timer.seconds() < 0.5 && !pause){
                    transfer.setPower(0);
                    intake.setPower(0);
                }
                else{
                    shot_count++;
                    intake.setPower(0);
                    transfer.setPower(0);
                    timer.reset();
                    pause = !pause;
                }
                if (shot_count >= 8) {
                    atirando = false;
                }
            }*/


            // =======================================================
            // 6. TELEMETRIA COMPLETA DE DADOS E ODOMETRIA
            // =======================================================
            odo.update();
            telemetry.addData("--- TORRETA TRACKING (60, 60) ---", "");
            telemetry.addData("Torreta Ângulo Atual", "%.2f°", turretObjeto.getCurrentAngle());
            telemetry.addData("Tx: ", "%.2f°", limelight.getTx());
            telemetry.addData("Ty: ", "%.2f°", limelight.getTy());
            telemetry.addData("Torreta Output PID", "%.2f", turretObjeto.getOutput());
            telemetry.addData("--- SHOOTER (GAMEPAD 2) ---", "");
            telemetry.addData("Shooter Target RPM", shooterVelocity);
            telemetry.addData("Shooter Current RPM", "%.2f", atirador.getRPM());
            telemetry.addData("--- MECANISMOS ---", "");
            telemetry.addData("x", odo.getPosition().getX(INCH));
            telemetry.addData("y", odo.getPosition().getY(INCH));




            telemetry.update();
        }
    }

    public double[] getDirection(double x, double y) {
        double deadband = 0.15;
        if (Math.hypot(x, y) < deadband) {
            index = -1;
            return new double[]{0, 0};
        }
        double angle = Math.toDegrees(Math.atan2(y, x));
        if (angle < 0) {
            angle += 360;
        }
        index = ((int) ((angle + 22.5) / 45)) % 8;
        return DIRECTIONS[index];
    }

    public double headingPID() {
        erro = gyro() - alvo;
        proporcional = kp * erro;
        integral += erro;
        integral = Math.max(-100, Math.min(100, integral));
        double termoIntegral = integral * ki;
        double derivativa = (erro - ultimoErro) * kd;
        ultimoErro = erro;
        return (proporcional + termoIntegral + derivativa) / 100.0;
    }

    public int convertRPMtoTicks(int rpm) {
        return rpm * 28 / 60;
    }

    public void mecanum(double x, double y, double rx) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        esquerdaFrente.setPower(-frontLeftPower);
        esquerdaTras.setPower(backLeftPower);
        direitaFrente.setPower(frontRightPower);
        direitaTras.setPower(backRightPower);
    }
    public double getDistance(double ty) {
        double targetHeight = 1.21;   // 1,21 metros do chão
        double cameraHeight = 0.412;  // 41,2 cm do chão (0,412 m)
        double cameraAngle = 48.0;    // 48 graus de inclinação da horizontal

        double angle = cameraAngle + ty;
        telemetry.addData("angle" , angle);
        telemetry.addData("target/camera: ", targetHeight-cameraHeight);
        telemetry.addData("tan", Math.tan(Math.toRadians(angle)));
        return (targetHeight - cameraHeight) * Math.tan(Math.toRadians(angle));
    }

    public double gyro() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}

