module.exports = function (RED) {
    function PIDControllerNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;

        const ctx = node.context(); // <- context voor opslag

        // Basis PID-parameters (startwaarden uit config)
        node.kp = parseFloat(config.kp) || 0;
        node.Ti = parseFloat(config.Ti) || 0;
        node.Td = parseFloat(config.Td) || 0;

        node.outMin = parseFloat(config.outMin);
        node.outMax = parseFloat(config.outMax);
        node.direction = config.direction || "direct"; // "direct" of "reverse"

        // Zelflerend – langzaam, voor trage temperatuurregeling
        node.autoTune   = config.autoTune;
        node.learnRateP = parseFloat(config.learnRateP) || 0.02; // 2% stap
        node.learnRateI = parseFloat(config.learnRateI) || 0.02;
        node.learnRateD = parseFloat(config.learnRateD) || 0.01;

        node.kpMin = parseFloat(config.kpMin) || 0.01;
        node.kpMax = parseFloat(config.kpMax) || 100.0;
        node.TiMin = parseFloat(config.TiMin) || 10.0;    // min 10 s
        node.TiMax = parseFloat(config.TiMax) || 7200.0;  // max 2 uur
        node.TdMin = parseFloat(config.TdMin) || 0.0;
        node.TdMax = parseFloat(config.TdMax) || 600.0;   // max 10 min

        //Probeer eerder opgeslagen parameters te laden
        try {
            const saved = ctx.get("pidParams");
            if (saved && typeof saved === "object") {
                if (typeof saved.kp === "number") node.kp = saved.kp;
                if (typeof saved.Ti === "number") node.Ti = saved.Ti;
                if (typeof saved.Td === "number") node.Td = saved.Td;

                node.status({
                    fill: "green",
                    shape: "dot",
                    text: `geladen Kp=${node.kp.toFixed(2)} Ti=${node.Ti.toFixed(0)}s`
                });
            }
        } catch (e) {
            node.warn("Kon opgeslagen PID-parameters niet lezen: " + e.message);
        }

        // Interne state
        let lastPV = null;
        let lastSP = null;

        let integral = 0;
        let lastError = 0;
        let lastTime = Date.now();
        let firstRun = true;

        // Voor zelflerend gedrag – venster over lange tijd
        const WINDOW_SIZE = 120; // bij ~10 s sample ≈ 20 minuten
        let errorWindow = [];
        let prevAvgAbsError = null;

        function clamp(v, min, max) {
            if (v > max) return max;
            if (v < min) return min;
            return v;
        }

        //Hulpje om huidige parameters op te slaan
        function saveParams() {
            const data = {
                kp: node.kp,
                Ti: node.Ti,
                Td: node.Td
            };
            try {
                // Als je een specifieke file-store wilt gebruiken:
                // ctx.set("pidParams", data, "file");
                ctx.set("pidParams", data);
            } catch (e) {
                node.warn("Kon PID-parameters niet opslaan: " + e.message);
            }
        }

        /**
         * Zelflerende aanpassing specifiek voor trage temperatuurregeling.
         */
        function updateLearning(error) {
            if (!node.autoTune) return;

            // Fout opslaan in venster
            errorWindow.push(error);
            if (errorWindow.length < WINDOW_SIZE) {
                return; // nog niet genoeg data
            }

            // Genoeg data → evaluatie
            let sum = 0;
            let sumAbs = 0;
            let zeroCrossings = 0;
            let prevE = errorWindow[0];

            for (let i = 0; i < errorWindow.length; i++) {
                const e = errorWindow[i];
                sum += e;
                sumAbs += Math.abs(e);
                if (i > 0 && e * prevE < 0) {
                    zeroCrossings++;
                }
                prevE = e;
            }

            const avgError = sum / errorWindow.length;
            const avgAbsError = sumAbs / errorWindow.length;
            const oscFraction = zeroCrossings / (errorWindow.length - 1);

            const ref = Math.max(Math.abs(lastSP) || 1, 1);
            const smallErrorThr = 0.02 * ref; // 2% van SP
            const largeErrorThr = 0.10 * ref; // 10% van SP
            const biasThreshold = 0.05 * ref; // 5% offset

            if (prevAvgAbsError === null) {
                prevAvgAbsError = avgAbsError;
            }

            // 1) Als het al netjes is: niks doen
            if (avgAbsError < smallErrorThr && oscFraction < 0.2) {
                errorWindow = [];
                prevAvgAbsError = avgAbsError;
                return;
            }

            // 2) Oscillaties → Kp omlaag, Ti langer
            if (oscFraction > 0.3 && avgAbsError > smallErrorThr) {
                node.kp *= (1.0 - node.learnRateP);
                node.Ti *= (1.0 + node.learnRateI);
            }
            // 3) Grote fout, weinig oscillatie → traag → Kp omhoog, Ti korter
            else if (avgAbsError > largeErrorThr && oscFraction < 0.2) {
                node.kp *= (1.0 + node.learnRateP);
                node.Ti *= (1.0 - node.learnRateI);
            }
            // 4) Blijvende offset → Ti korter
            else if (Math.abs(avgError) > biasThreshold && oscFraction < 0.3) {
                node.Ti *= (1.0 - node.learnRateI);
            }

            // Grenzen
            node.kp = clamp(node.kp, node.kpMin, node.kpMax);
            if (node.Ti <= 0 || !isFinite(node.Ti)) node.Ti = node.TiMin;
            node.Ti = clamp(node.Ti, node.TiMin, node.TiMax);
            node.Td = clamp(node.Td, node.TdMin, node.TdMax);

            prevAvgAbsError = avgAbsError;
            errorWindow = [];

            // 🔹 Na aanpassen: parameters opslaan
            saveParams();

            node.status({
                fill: "blue",
                shape: "dot",
                text: `auto Kp=${node.kp.toFixed(2)} Ti=${node.Ti.toFixed(0)}s`
            });
        }

        node.on("input", function (msg, send, done) {
            try {
                // PV en SP uit de flow
                if (msg.hasOwnProperty("payload")) {
                    const pv = parseFloat(msg.payload);
                    if (!isNaN(pv)) lastPV = pv;
                }

                if (msg.hasOwnProperty("setpoint")) {
                    const sp = parseFloat(msg.setpoint);
                    if (!isNaN(sp)) lastSP = sp;
                }

                // Optioneel: volledige reset, incl. vergeten van geleerde params
                if (msg.reset === true || msg.reset === "hard") {
                    integral = 0;
                    lastError = 0;
                    firstRun = true;
                    errorWindow = [];
                    ctx.set("pidParams", null); // wis opgeslagen waarden
                    node.status({ fill: "grey", shape: "dot", text: "PID reset" });
                }

                if (lastPV === null || lastSP === null) {
                    node.status({ fill: "yellow", shape: "dot", text: "wachten op PV/SP" });
                    if (done) done();
                    return;
                }

                node.status({});

                const now = Date.now();
                let dt = (now - lastTime) / 1000.0;
                if (dt <= 0 || dt > 60) {
                    dt = 10.0; // typische sampletijd voor traag systeem
                }
                lastTime = now;

                // Ki en Kd
                let Ki = 0;
                let Kd = 0;
                if (node.Ti > 0) {
                    Ki = node.kp / node.Ti;
                }
                if (node.Td > 0) {
                    Kd = node.kp * node.Td;
                }

                // Fout
                let error = lastSP - lastPV;
                if (node.direction === "reverse") {
                    error = -error;
                }

                // PID
                const P = node.kp * error;
                integral += error * dt * Ki;

                let D = 0;
                if (!firstRun) {
                    const derivative = (error - lastError) / dt;
                    D = Kd * derivative;
                } else {
                    firstRun = false;
                }

                lastError = error;

                let output = P + integral + D + (node.outMax + node.outMin)/2.0;

                // Clamping + anti-windup
                if (output > node.outMax) {
                    output = node.outMax;
                    if (error > 0) integral -= error * dt * Ki;
                } else if (output < node.outMin) {
                    output = node.outMin;
                    if (error < 0) integral -= error * dt * Ki;
                }

                // Zelflerend
                updateLearning(error);

                msg.payload = output;
                msg.pid = {
                    out: output,
                    pv: lastPV,
                    setpoint: lastSP,
                    error,
                    P,
                    I: integral,
                    D,
                    dt,
                    Kp: node.kp,
                    Ti: node.Ti,
                    Td: node.Td,
                    Ki,
                    Kd
                };

                send(msg);
                if (done) done();

            } catch (err) {
                node.error(err, msg);
                if (done) done(err);
            }
        });
    }

    RED.nodes.registerType("pid-controller", PIDControllerNode);
};
