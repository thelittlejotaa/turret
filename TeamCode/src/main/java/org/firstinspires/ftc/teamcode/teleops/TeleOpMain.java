package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.LimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@TeleOp(name = "Teste Torreta e Limelight", group = "Testes")
public class TeleOpMain extends OpMode {

    private Turret turret;
    private LimeLight limelight;

    // Controle de borda de subida para o D-Pad
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        turret = new Turret();
        turret.init(hardwareMap);

        limelight = new LimeLight();
        limelight.init(hardwareMap);

        turret.resetTimer();

        telemetry.addLine("======================================");
        telemetry.addLine("Posicione a torreta no CENTRO antes de apertar INIT!");
        telemetry.addLine("======================================");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Obtém a leitura visual da Limelight
        LLResult result = limelight.getLatestResult();

        // 2. Atualiza o controle PID e aplica a trava de Soft Limits
        turret.update(result);

        // 3. Ajuste de kP em tempo real pelo D-Pad do Gamepad 1
        if (gamepad1.dpad_up && !lastDpadUp) {
            turret.setkP(turret.getkP() + 0.001);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            turret.setkP(Math.max(0, turret.getkP() - 0.001));
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // 4. Telemetria para Calibração e Monitoramento
        telemetry.addData("=== CALIBRAÇÃO DE LIMITES (CABO) ===", "");
        telemetry.addData("Ticks Atuais do Encoder", turret.getCurrentPosition());

        telemetry.addData("=== STATUS DA VISÃO ===", "");
        if (result != null && result.isValid()) {
            telemetry.addData("Status Alvo", "ALVO DETECTADO");
            telemetry.addData("Erro Horizontal (tx)", "%.2f°", result.getTx());
        } else {
            telemetry.addData("Status Alvo", "BUSCANDO...");
        }

        telemetry.addData("=== REGULAGEM PID ===", "");
        telemetry.addData("kP", "%.4f (Use D-Pad Cima/Baixo)", turret.getkP());
        telemetry.addData("kD", "%.4f", turret.getkD());

        telemetry.update();
    }
}