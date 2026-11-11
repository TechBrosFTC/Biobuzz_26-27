package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Classe de controle da Torreta com movimentação por dois Servos (Servo ou CRServo),
 * encoder para leitura de ângulo, limites operacionais de ±540° e lógica de caminho mais curto via PIDF.
 */
public class Turret {

    // Servos de acionamento
    private CRServo crServo1, crServo2;

    // Encoder de leitura de posição
    private DcMotorEx encoder;

    // Limites de rotação da torreta em graus (±540°)
    public static final double MIN_ANGLE = -180;
    public static final double MAX_ANGLE = 180;

    // Fator de conversão Ticks por Grau
    private double ticksPerDegree = 8192.0 / 360.0;

    // Parâmetros e Estado do Controlador PIDF
    private double kp = 0.25;      // Ganho proporcional aumentado para resposta mais firme
    private double ki = 0.0005;    // Ganho integral para vencer erro estático acumulado
    private double kd = 0.1;     // Ganho derivativo para amortecimento
    private double kf = 0.2;      // Feedforward estático para vencer o atrito mecânico (stiction)

    private double tolerance = 4.5; // Tolerância em graus para parar no alvo exato

    private double targetAngle = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastOutput = 0.0;

    /**
     * Construtor passando Servos de Rotação Contínua (CRServo), Encoder e Proporção Ticks/Grau.
     */
    public Turret(CRServo crServo1, CRServo crServo2, DcMotorEx encoder, double ticksPerDegree, DcMotorSimple.Direction direction) {
        this.crServo1 = crServo1;
        this.crServo2 = crServo2;
        this.encoder = encoder;
        this.ticksPerDegree = ticksPerDegree;
        initEncoder(direction);
    }

    /**
     * Construtor utilizando HardwareMap e nomes das configurações.
     */
    public Turret(HardwareMap hardwareMap, String encoderName, String servo1Name, String servo2Name, double ticksPerDegree, DcMotorSimple.Direction direction) {
        this.encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        this.crServo1 = hardwareMap.get(CRServo.class, servo1Name);
        this.crServo2 = hardwareMap.get(CRServo.class, servo2Name);
        this.ticksPerDegree = ticksPerDegree;
        initEncoder(direction);
    }

    private void initEncoder(DcMotorSimple.Direction direction) {
        if (encoder != null) {
            encoder.setDirection(direction);
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Retorna o ângulo atual da torreta em graus com base na leitura do encoder.
     */
    public double getCurrentAngle() {
        if (encoder == null) return 0.0;
        return encoder.getCurrentPosition() / ticksPerDegree;
    }

    /**
     * Retorna a posição bruta do encoder em ticks.
     */
    public int getEncoderTicks() {
        if (encoder == null) return 0;
        return encoder.getCurrentPosition();
    }

    /**
     * Reseta a posição do encoder para 0°.
     */
    public void resetEncoder() {
        if (encoder != null) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        targetAngle = 0.0;
        integral = 0.0;
        lastError = 0.0;
    }

    /**
     * Define os ganhos do controlador PID da torreta.
     */
    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Define os ganhos do controlador PIDF da torreta incluindo o fator de compensação de atrito kf.
     */
    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    /**
     * Define a tolerância aceitável em graus para considerar no alvo.
     */
    public void setTolerance(double toleranceDegrees) {
        this.tolerance = toleranceDegrees;
    }

    /**
     * Define o ângulo alvo garantindo que esteja dentro do limite operável.
     */
    public void setTargetAngle(double angleDegrees) {
        this.targetAngle = Range.clip(angleDegrees, MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Calcula e define o ângulo alvo através do CAMINHO MAIS CURTO,
     * respeitando os limites operacionais.
     *
     * @param desiredAngle Ângulo desejado em graus
     */
    public void goToAngleShortestPath(double desiredAngle) {
        double current = getCurrentAngle();

        // Calcula a menor diferença angular no intervalo [-180, 180]
        double delta = desiredAngle - current;
        while (delta > 180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;

        double candidateTarget = current + delta;

        // Se o caminho mais curto estiver dentro do limite operável, aplica
        if (candidateTarget >= MIN_ANGLE && candidateTarget <= MAX_ANGLE) {
            setTargetAngle(candidateTarget);
        } else {
            setTargetAngle(Range.clip(desiredAngle, MIN_ANGLE, MAX_ANGLE));
        }
    }

    /**
     * Atualiza o cálculo do PIDF com compensação de atrito estático (stiction)
     * e aplica a potência aos servos da torreta.
     */
    public void update() {
        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        double output = 0.0;

        // Se o erro for maior que a tolerância aceitável, calcula o PIDF
        if (Math.abs(error) > tolerance) {
            integral += error;
            integral = Range.clip(integral, -200.0, 200.0); // Anti-windup

            double derivative = error - lastError;

            // Termo PID base + Compensação Feedforward de atrito estático (kf * signum(error))
            output = (kp * error) + (ki * integral) + (kd * derivative) + (Math.signum(error) * kf);
        } else {
            // Se estiver dentro da tolerância, desliga a potência para não oscilar e limpa a integral
            integral = 0.0;
            output = 0.0;
        }

        lastError = error;

        // Limita a saída entre -1.0 e 1.0
        output = Range.clip(output, -1.0, 1.0);
        lastOutput = output;

        // Aplica o comando nos servos de rotação contínua
        if (crServo1 != null && crServo2 != null) {
            crServo1.setPower(output);
            crServo2.setPower(-output); // Invertido para servos opostos no mesmo eixo
        }
    }

    /**
     * Retorna o ângulo alvo atualmente configurado.
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Retorna o erro atual do PID em graus.
     */
    public double getError() {
        return targetAngle - getCurrentAngle();
    }

    /**
     * Retorna a última saída do PID aplicada aos servos.
     */
    public double getOutput() {
        return lastOutput;
    }
}
