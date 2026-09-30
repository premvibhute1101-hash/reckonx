import { InsMechanization } from './src/core/InsMechanization.js';

function runButterworthTest() {
    console.log("=========================================");
    console.log("BUTTERWORTH VS EMA FREQUENCY RESPONSE TEST");
    console.log("=========================================");
    
    const Fs = 50; // 50 Hz sampling rate
    const dt = 1.0 / Fs;
    const duration = 2.0; // 2 seconds
    const numSamples = Math.floor(duration * Fs);

    const f_signal = 2.0; // 2 Hz real motion
    const f_noise = 20.0; // 20 Hz noise

    // Initialize INS (using the new Butterworth filter)
    const ins = new InsMechanization();
    ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });

    // State for the old EMA filter to compare against
    let emaFiltered = { x: 0, y: 0, z: 9.81 };
    const emaAlpha = 0.3; // The hardcoded alpha from the old code

    let maxEma20Hz = 0;
    let maxBw20Hz = 0;
    let minBw2Hz = 999;
    
    // We will measure amplitude loss of the 2Hz signal, and rejection of the 20Hz noise
    // To do this simply, we can run them in isolation, or together and observe the envelope.
    // Let's run isolation tests.

    console.log("--- TEST 1: 2Hz TRUE MOTION SIGNAL (Amplitude = 1.0) ---");
    let peakBw2Hz = 0;
    let peakEma2Hz = 0;
    for (let i = 0; i < numSamples; i++) {
        const t = i * dt;
        const val = Math.sin(2 * Math.PI * f_signal * t);
        
        // Pass through Butterworth
        ins.predict(dt, { x: val, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 });
        const bwOut = ins.getFilteredAccel()!.x;
        if (Math.abs(bwOut) > peakBw2Hz) peakBw2Hz = Math.abs(bwOut);

        // Pass through old EMA
        emaFiltered.x = emaAlpha * val + (1 - emaAlpha) * emaFiltered.x;
        if (Math.abs(emaFiltered.x) > peakEma2Hz && i > Fs) peakEma2Hz = Math.abs(emaFiltered.x); // Wait 1s to settle
    }
    console.log(`Input Amplitude: 1.000`);
    console.log(`Old EMA Output Amplitude: ${peakEma2Hz.toFixed(3)}`);
    console.log(`New Butterworth Output Amplitude: ${peakBw2Hz.toFixed(3)}`);
    console.log(`(Higher is better - means less dampening of real motion)`);

    // Reset filters
    ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });
    emaFiltered = { x: 0, y: 0, z: 9.81 };

    console.log("\n--- TEST 2: 20Hz HIGH-FREQUENCY NOISE (Amplitude = 1.0) ---");
    let peakBw20Hz = 0;
    let peakEma20Hz = 0;
    for (let i = 0; i < numSamples; i++) {
        const t = i * dt;
        const val = Math.sin(2 * Math.PI * f_noise * t);
        
        // Pass through Butterworth
        ins.predict(dt, { x: val, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 });
        const bwOut = ins.getFilteredAccel()!.x;
        // Check peak after settling time
        if (i > Fs && Math.abs(bwOut) > peakBw20Hz) peakBw20Hz = Math.abs(bwOut);

        // Pass through old EMA
        emaFiltered.x = emaAlpha * val + (1 - emaAlpha) * emaFiltered.x;
        if (i > Fs && Math.abs(emaFiltered.x) > peakEma20Hz) peakEma20Hz = Math.abs(emaFiltered.x);
    }
    console.log(`Input Amplitude: 1.000`);
    console.log(`Old EMA Residual Noise: ${peakEma20Hz.toFixed(3)}`);
    console.log(`New Butterworth Residual Noise: ${peakBw20Hz.toFixed(3)}`);
    console.log(`(Lower is better - means better noise rejection)`);

    console.log("\nVERDICT:");
    if (peakBw20Hz < peakEma20Hz) {
        console.log("SUCCESS: 2nd-Order Butterworth rejects high-frequency noise better than the old EMA.");
    } else {
        console.log("FAIL: Butterworth is not rejecting noise better.");
    }
}

runButterworthTest();
