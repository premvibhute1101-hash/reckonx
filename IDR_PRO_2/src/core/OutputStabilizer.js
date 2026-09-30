"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.OutputStabilizer = exports.CvKalmanSmoother = exports.RateLimiter = void 0;
exports.haversineDistance = haversineDistance;
exports.calculateBearing = calculateBearing;
exports.projectLocation = projectLocation;
const ml_matrix_1 = require("ml-matrix");
// Haversine formula to calculate distance in meters between two lat/lon points
function haversineDistance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // Earth radius in meters
    const rLat1 = lat1 * Math.PI / 180;
    const rLat2 = lat2 * Math.PI / 180;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
        Math.cos(rLat1) * Math.cos(rLat2) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c;
}
// Calculate bearing from pt1 to pt2
function calculateBearing(lat1, lon1, lat2, lon2) {
    const rLat1 = lat1 * Math.PI / 180;
    const rLat2 = lat2 * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const y = Math.sin(dLon) * Math.cos(rLat2);
    const x = Math.cos(rLat1) * Math.sin(rLat2) -
        Math.sin(rLat1) * Math.cos(rLat2) * Math.cos(dLon);
    return Math.atan2(y, x);
}
// Project a point given distance and bearing
function projectLocation(lat, lon, distanceMeters, bearing) {
    const R = 6371e3;
    const rLat = lat * Math.PI / 180;
    const rLon = lon * Math.PI / 180;
    const newLat = Math.asin(Math.sin(rLat) * Math.cos(distanceMeters / R) +
        Math.cos(rLat) * Math.sin(distanceMeters / R) * Math.cos(bearing));
    const newLon = rLon + Math.atan2(Math.sin(bearing) * Math.sin(distanceMeters / R) * Math.cos(rLat), Math.cos(distanceMeters / R) - Math.sin(rLat) * Math.sin(newLat));
    return {
        lat: newLat * 180 / Math.PI,
        lon: newLon * 180 / Math.PI
    };
}
class RateLimiter {
    config;
    lastSpeed = 0;
    constructor(config) {
        this.config = config;
    }
    filter(input, previousAccepted) {
        if (!previousAccepted) {
            this.lastSpeed = 0;
            return { output: input };
        }
        const dt = (input.timestamp - previousAccepted.timestamp) / 1000.0;
        if (dt <= 0)
            return { output: input }; // Should not happen in causal processing
        const distMeters = haversineDistance(previousAccepted.lat, previousAccepted.lon, input.lat, input.lon);
        const impliedSpeed = distMeters / dt;
        const impliedAccel = Math.abs(impliedSpeed - this.lastSpeed) / dt;
        let clamped = false;
        let allowedSpeed = impliedSpeed;
        if (impliedSpeed > this.config.maxSpeedMps) {
            clamped = true;
            allowedSpeed = this.config.maxSpeedMps;
        }
        else if (impliedAccel > this.config.maxAccelMps2) {
            clamped = true;
            // Limit speed change based on max accel
            allowedSpeed = this.lastSpeed + Math.sign(impliedSpeed - this.lastSpeed) * this.config.maxAccelMps2 * dt;
            // ensure we don't go negative or exceed max speed
            allowedSpeed = Math.max(0, Math.min(allowedSpeed, this.config.maxSpeedMps));
        }
        if (clamped) {
            const bearing = calculateBearing(previousAccepted.lat, previousAccepted.lon, input.lat, input.lon);
            const allowedDist = allowedSpeed * dt;
            const projected = projectLocation(previousAccepted.lat, previousAccepted.lon, allowedDist, bearing);
            this.lastSpeed = allowedSpeed;
            const clampedInput = { ...input, lat: projected.lat, lon: projected.lon };
            const clampEvent = {
                type: 'rate_limit_clamp',
                rawFix: { lat: input.lat, lon: input.lon },
                clampedFix: { lat: projected.lat, lon: projected.lon },
                impliedSpeed,
                impliedAccel,
                timestamp: input.timestamp
            };
            return { output: clampedInput, clampEvent };
        }
        this.lastSpeed = impliedSpeed;
        return { output: input };
    }
}
exports.RateLimiter = RateLimiter;
class CvKalmanSmoother {
    x; // [lat, lon, v_lat, v_lon]^T (deg, deg, deg/s, deg/s)
    P;
    Q_pos;
    Q_vel;
    lastTime = 0;
    isInitialized = false;
    constructor(qPos = 1e-9, qVel = 1e-5) {
        this.Q_pos = qPos;
        this.Q_vel = qVel;
        this.x = ml_matrix_1.Matrix.zeros(4, 1);
        this.P = ml_matrix_1.Matrix.eye(4).mul(100);
    }
    filter(input) {
        if (!this.isInitialized || this.lastTime === 0 || Math.abs(input.timestamp - this.lastTime) > 10000) {
            this.x = new ml_matrix_1.Matrix([[input.lat], [input.lon], [0], [0]]);
            this.P = ml_matrix_1.Matrix.eye(4).mul(1e-9);
            this.lastTime = input.timestamp;
            this.isInitialized = true;
            return { lat: input.lat, lon: input.lon, wasClamped: false, wasSmoothed: false };
        }
        const dt = (input.timestamp - this.lastTime) / 1000.0;
        if (dt <= 0) {
            return { lat: this.x.get(0, 0), lon: this.x.get(1, 0), wasClamped: false, wasSmoothed: true };
        }
        this.lastTime = input.timestamp;
        // 1. Predict
        const F = ml_matrix_1.Matrix.eye(4);
        F.set(0, 2, dt);
        F.set(1, 3, dt);
        const Q = ml_matrix_1.Matrix.zeros(4, 4);
        Q.set(0, 0, this.Q_pos * dt);
        Q.set(1, 1, this.Q_pos * dt);
        Q.set(2, 2, this.Q_vel * dt);
        Q.set(3, 3, this.Q_vel * dt);
        this.x = F.mmul(this.x);
        this.P = F.mmul(this.P).mmul(F.transpose()).add(Q);
        // 2. Update
        const z = new ml_matrix_1.Matrix([[input.lat], [input.lon]]);
        const H = ml_matrix_1.Matrix.zeros(2, 4);
        H.set(0, 0, 1);
        H.set(1, 1, 1);
        // R scaling
        // ~0.2m variance for GOOD -> ~1e-11 deg^2
        let baseR = 1e-10;
        if (input.gnssState === 'DEGRADED' || input.gnssState === 'WEAK_LOST') {
            baseR = 1e-8; // 100x looser when degraded
        }
        // Apply ekfConfidence if provided (assuming it acts like a multiplier or raw variance)
        const R_val = input.ekfConfidence ? baseR * input.ekfConfidence : baseR;
        const R = ml_matrix_1.Matrix.eye(2).mul(R_val);
        const S = H.mmul(this.P).mmul(H.transpose()).add(R);
        let S_inv;
        try {
            S_inv = (0, ml_matrix_1.inverse)(S);
        }
        catch (e) {
            // Singular matrix fallback
            return { lat: this.x.get(0, 0), lon: this.x.get(1, 0), wasClamped: false, wasSmoothed: true };
        }
        const K = this.P.mmul(H.transpose()).mmul(S_inv);
        const y = z.sub(H.mmul(this.x)); // Innovation
        this.x = this.x.add(K.mmul(y));
        const I = ml_matrix_1.Matrix.eye(4);
        this.P = I.sub(K.mmul(H)).mmul(this.P);
        return {
            lat: this.x.get(0, 0),
            lon: this.x.get(1, 0),
            wasClamped: false,
            wasSmoothed: true
        };
    }
}
exports.CvKalmanSmoother = CvKalmanSmoother;
class OutputStabilizer {
    rateLimiter;
    smoother;
    previousAccepted = null;
    constructor(rateLimiterConfig, qPos = 1e-9, qVel = 1e-5) {
        this.rateLimiter = new RateLimiter(rateLimiterConfig);
        this.smoother = new CvKalmanSmoother(qPos, qVel);
    }
    process(input) {
        // 1. Rate Limiting (Clamping)
        const { output: limitedInput, clampEvent } = this.rateLimiter.filter(input, this.previousAccepted);
        // Store the limited input for next iteration's rate limiting
        this.previousAccepted = limitedInput;
        // 2. Kalman Smoothing
        const smoothedOutput = this.smoother.filter(limitedInput);
        // Combine metadata
        return {
            ...smoothedOutput,
            wasClamped: !!clampEvent,
            clampEvent: clampEvent
        };
    }
}
exports.OutputStabilizer = OutputStabilizer;
