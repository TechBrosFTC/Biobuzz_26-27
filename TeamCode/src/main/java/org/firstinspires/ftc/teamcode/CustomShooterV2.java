package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CustomShooterV2 {

    private final DcMotorEx motor;

    private static final double TICKS_PER_REV = 28;

    // PIDF - ganhos para calibração
    private double kP = 0.00015;
    private double kI = 0.000001;
    private double kD = 0.000001;
    private double kF = 0.00015;

    // Recovery Boost
    private double recoveryGain = 0.00008;
    private double maxBoost = 0.15;

    // Shot Detection
    private double shotThreshold = 100;
    private double baselineRPM = 0;
    private double minShotRPM = 0;

    // Adaptação Automática / Gradual do Feedforward (kF)
    private boolean autoKFEnabled = true;
    private double kFAdaptRate = 0.00001; // Taxa de ajuste gradual por segundo
    private double underSpeedRatio = 0.90; // Se currentRPM < targetRPM * 0.90 (ficou muito abaixo do target)
    private double overSpeedRatio = 1.08;  // Se currentRPM > targetRPM * 1.08 (ficou muito acima do target)
    private double minKF = 0.00001;
    private double maxKF = 0.00022;

    private double targetRPM = 0;
    private double currentRPM = 0;
    private double output = 0;
    private double boost = 0;

    // PID
    private double integral = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;

    // Timing
    private long lastUpdate = 0;
    private long shotTime = 0;
    private long recoveryStart = 0;
    private long recoveryStableStart = 0;

    private long lastShotTime = 0;
    private long lastRecoveryMs = 0;

    private boolean shotDetected = false;
    private boolean recovering = false;
    private boolean recoveryCompleted = false;

    public CustomShooterV2(DcMotorEx motor) {
        this.motor = motor;

        motor.setMode(
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    public void setTargetRPM(double rpm) {
        if (rpm != targetRPM) {
            integral = 0;
            lastError = 0;
            filteredDerivative = 0;
        }

        targetRPM = Math.max(0, rpm);

        if (targetRPM == 0) {
            stop();
        }
    }

    public void update() {

        long now = System.nanoTime();

        if (lastUpdate == 0) {
            lastUpdate = now;
            currentRPM = getMeasuredRPM();
            baselineRPM = currentRPM;
            return;
        }

        double dt = (now - lastUpdate) / 1e9;
        lastUpdate = now;

        dt = Math.max(0.001, Math.min(0.05, dt));

        currentRPM = getMeasuredRPM();

        if (targetRPM <= 0) {
            motor.setPower(0);
            return;
        }

        double error = targetRPM - currentRPM;

        // Ajuste Adaptativo Gradual do Feedforward (kF) caso fique muito abaixo ou muito acima do target
        if (autoKFEnabled && !recovering && targetRPM > 0) {
            if (currentRPM < targetRPM * underSpeedRatio && output < 1.0) {
                // Ajuste gradual para cima se estiver muito abaixo do target
                double errorRatio = (targetRPM - currentRPM) / targetRPM;
                kF += kFAdaptRate * errorRatio * dt;
                kF = Math.min(maxKF, kF);
            } else if (currentRPM > targetRPM * overSpeedRatio && output > 0.0) {
                // Ajuste gradual para baixo se estiver muito acima do target
                double overRatio = (currentRPM - targetRPM) / targetRPM;
                kF -= kFAdaptRate * overRatio * dt;
                kF = Math.max(minKF, kF);
            }
        }

        // Shot Detection
        detectShot(now);

        // Recovery measurement
        updateRecovery(now);

        // PID
        double derivative =
            (error - lastError) / dt;

        filteredDerivative =
            0.8 * filteredDerivative +
            0.2 * derivative;

        double p = kP * error;
        double d = kD * filteredDerivative;
        double f = kF * targetRPM;

        // Recovery Boost
        boost = 0;

        if (recovering && error > 0) {
            boost = Math.min(
                maxBoost,
                error * recoveryGain
            );
        }

        // Anti-windup
        double candidateIntegral =
            integral + error * dt;

        candidateIntegral = Math.max(
            -1000,
            Math.min(1000, candidateIntegral)
        );

        double candidateOutput =
            p + kI * candidateIntegral + d + f + boost;

        if ((candidateOutput >= 0 &&
             candidateOutput <= 1) ||
             (candidateOutput > 1 && error < 0) ||
             (candidateOutput < 0 && error > 0)) {

            integral = candidateIntegral;
        }

        output =
            p + kI * integral + d + f + boost;

        output = Math.max(0, Math.min(1, output));

        motor.setPower(output);

        lastError = error;

        // Baseline de RPM
        if (!recovering &&
            currentRPM > targetRPM * 0.8) {

            baselineRPM =
                0.95 * baselineRPM +
                0.05 * currentRPM;
        }
    }

    private double getMeasuredRPM() {
        return motor.getVelocity() *
               60.0 / TICKS_PER_REV;
    }

    private void detectShot(long now) {

        shotDetected = false;

        if (recovering) return;

        if (baselineRPM < targetRPM * 0.8) return;

        if (now - lastShotTime < 250_000_000L) return;

        double drop = baselineRPM - currentRPM;

        if (drop >= shotThreshold) {

            shotDetected = true;
            recovering = true;

            shotTime = now;
            recoveryStart = now;
            lastShotTime = now;

            minShotRPM = currentRPM;

            recoveryStableStart = 0;
            recoveryCompleted = false;
        }
    }

    private void updateRecovery(long now) {

        if (!recovering) return;

        minShotRPM = Math.min(
            minShotRPM,
            currentRPM
        );

        double recoveryTarget = targetRPM * 0.95;

        if (currentRPM >= recoveryTarget) {

            if (recoveryStableStart == 0) {
                recoveryStableStart = now;
            }

            if (now - recoveryStableStart >= 100_000_000L) {

                lastRecoveryMs =
                    (now - recoveryStart) / 1_000_000L;

                recovering = false;
                recoveryCompleted = true;
            }

        } else {
            recoveryStableStart = 0;
        }
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getOutput() {
        return output;
    }

    public double getBoost() {
        return boost;
    }

    public double getMinShotRPM() {
        return minShotRPM;
    }

    public long getRecoveryMs() {
        return lastRecoveryMs;
    }

    public boolean isShotDetected() {
        return shotDetected;
    }

    public boolean isRecovering() {
        return recovering;
    }

    public boolean isReady() {
        return targetRPM > 0 &&
               currentRPM >= targetRPM * 0.95;
    }

    public void stop() {

        targetRPM = 0;
        integral = 0;
        boost = 0;
        output = 0;

        recovering = false;
        recoveryCompleted = false;

        motor.setPower(0);
    }

    public void setPIDF(double p, double i, double d, double f, double recoveryGain, double maxBoost) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        this.maxBoost = maxBoost;
        this.recoveryGain = recoveryGain;
    }

    public void setAutoKFEnabled(boolean enabled) {
        this.autoKFEnabled = enabled;
    }

    public boolean isAutoKFEnabled() {
        return autoKFEnabled;
    }

    public double getKF() {
        return kF;
    }

    public void setKFAdaptRate(double rate) {
        this.kFAdaptRate = rate;
    }

    public void setUnderSpeedRatio(double ratio) {
        this.underSpeedRatio = ratio;
    }

    public void setOverSpeedRatio(double ratio) {
        this.overSpeedRatio = ratio;
    }
}
