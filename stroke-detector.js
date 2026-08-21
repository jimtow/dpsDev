// stroke-detector.js
//
// Counts rowing strokes from a stream of 3-axis accelerometer samples, independent of exactly how
// the sensor is mounted or oriented, by:
//   1. Continuously finding the principal axis of motion via PCA — the direction the accelerometer
//      swings back and forth along the most — using power iteration on the sample covariance
//      matrix. This is what makes it work whether the sensor is on the handle, the seat, or
//      anywhere else with a strong dominant oscillation.
//   2. Removing the slow-moving gravity/orientation offset from each sample, then projecting it
//      onto that axis to get a single 1-D, zero-centered oscillating signal.
//   3. Running an adaptive-threshold, hysteresis peak detector over that signal to count stroke
//      cycles and estimate stroke rate.

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function matVecMul(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

// Dominant eigenvector of a symmetric 3x3 matrix via power iteration. `seed` doubles as the
// starting guess and the fallback if the matrix is ~0 (no motion yet to define an axis from).
function principalEigenvector(cov, seed) {
  let v = seed;
  for (let i = 0; i < 25; i++) {
    v = matVecMul(cov, v);
    const len = Math.hypot(v[0], v[1], v[2]);
    if (len < 1e-9) return seed;
    v = [v[0] / len, v[1] / len, v[2] / len];
  }
  return v;
}

export class StrokeDetector {
  constructor(options = {}) {
    this.windowMs = options.windowMs ?? 3000; // how much history PCA is computed over
    this.pcaRecalcMs = options.pcaRecalcMs ?? 250; // recompute the principal axis this often
    this.minStrokeIntervalMs = options.minStrokeIntervalMs ?? 350; // refractory period (~170 spm cap)
    this.baselineTauMs = options.baselineTauMs ?? 1200; // slow EMA time constant for detrending
    this.smoothTauMs = options.smoothTauMs ?? 80; // fast EMA time constant for noise smoothing
    this.thresholdFraction = options.thresholdFraction ?? 0.35; // rise needed, as a fraction of recent amplitude
    this.releaseFraction = options.releaseFraction ?? 0.15; // hysteresis release fraction (< thresholdFraction)
    // Absolute floor (in g) below which we never call it a stroke, regardless of what the adaptive
    // envelope has settled to. Without this, sitting still lets the envelope adapt down to the
    // sensor's own noise floor and then that noise trips the "relative" threshold on its own —
    // this is what actually keeps a motionless sensor at a count of 0.
    this.minAmplitude = options.minAmplitude ?? 0.08;
    // Minimum total variance (g^2, summed across axes) a sample window must have before we trust a
    // freshly computed axis. When the sensor is nearly still, the "dominant" direction of pure
    // sensor noise is meaningless and can swing wildly between recalculations — freezing the axis
    // in that case avoids that swing corrupting the signal (see meanVec below for why it would).
    this.minMotionVariance = options.minMotionVariance ?? 0.0004;

    this.onStroke = options.onStroke || null;

    this.samples = []; // {t, ax, ay, az} rolling buffer used to recompute PCA
    this.axis = [1, 0, 0]; // current principal-axis estimate
    this.lastPcaTime = 0;

    // Slow per-axis EMA of the raw [ax, ay, az] vector — tracks gravity + mounting orientation.
    // Subtracting this from each sample *before* projecting is what makes the projection safe even
    // when the axis itself jumps around: if we projected the raw (non-centered) vector instead, an
    // axis change would suddenly expose a different slice of the ~1g gravity offset as a spike.
    this.meanVec = null;
    this.smoothed = null; // fast EMA of the centered, projected signal (what peak detection runs on)
    this.envelope = 0; // rolling estimate of recent oscillation amplitude, for adaptive thresholds
    this.armed = true; // hysteresis state: true = waiting for a rise above threshold
    this._peakCandidate = -Infinity;
    this._lastTime = 0;
    this._lastSignal = 0; // exposed mainly for optional UI/debug plotting

    this.strokeCount = 0;
    this.strokeTimestamps = []; // recent stroke times, for rolling rate averaging
    this.lastStrokeTime = 0;
  }

  reset() {
    this.samples = [];
    this.meanVec = null;
    this.smoothed = null;
    this.envelope = 0;
    this.armed = true;
    this._peakCandidate = -Infinity;
    this._lastTime = 0;
    this._lastSignal = 0;
    this.strokeCount = 0;
    this.strokeTimestamps = [];
    this.lastStrokeTime = 0;
  }

  // Feed in one accelerometer sample: { ax, ay, az, t? } (t defaults to performance.now()).
  addSample({ ax, ay, az, t }) {
    t = t ?? performance.now();
    this.samples.push({ t, ax, ay, az });
    const cutoff = t - this.windowMs;
    while (this.samples.length > 0 && this.samples[0].t < cutoff) this.samples.shift();

    // Update the slow gravity/orientation estimate before projecting (see meanVec comment above).
    const alphaBase = this._alphaFor(this.baselineTauMs, t);
    if (this.meanVec === null) {
      this.meanVec = [ax, ay, az];
    } else {
      this.meanVec = [
        this.meanVec[0] + alphaBase * (ax - this.meanVec[0]),
        this.meanVec[1] + alphaBase * (ay - this.meanVec[1]),
        this.meanVec[2] + alphaBase * (az - this.meanVec[2]),
      ];
    }

    if (t - this.lastPcaTime >= this.pcaRecalcMs && this.samples.length >= 10) {
      this._recomputeAxis();
      this.lastPcaTime = t;
    }

    const centered = [ax - this.meanVec[0], ay - this.meanVec[1], az - this.meanVec[2]];
    const projected = dot(centered, this.axis);
    this._updateDetector(projected, t);
  }

  _recomputeAxis() {
    const n = this.samples.length;
    let mx = 0, my = 0, mz = 0;
    for (const s of this.samples) { mx += s.ax; my += s.ay; mz += s.az; }
    mx /= n; my /= n; mz /= n;

    let cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
    for (const s of this.samples) {
      const dx = s.ax - mx, dy = s.ay - my, dz = s.az - mz;
      cxx += dx * dx; cxy += dx * dy; cxz += dx * dz;
      cyy += dy * dy; cyz += dy * dz; czz += dz * dz;
    }
    cxx /= n; cxy /= n; cxz /= n; cyy /= n; cyz /= n; czz /= n;

    if (cxx + cyy + czz < this.minMotionVariance) return; // not enough motion to trust a new axis; keep the old one

    const cov = [
      [cxx, cxy, cxz],
      [cxy, cyy, cyz],
      [cxz, cyz, czz],
    ];

    let newAxis = principalEigenvector(cov, this.axis);
    // Keep sign continuity so the projected signal doesn't flip (peaks<->troughs) between
    // successive recalculations of the axis.
    if (dot(newAxis, this.axis) < 0) newAxis = newAxis.map((v) => -v);
    this.axis = newAxis;
  }

  _alphaFor(tauMs, t) {
    const dt = this._lastTime ? Math.max(1, t - this._lastTime) : 20;
    return 1 - Math.exp(-dt / tauMs);
  }

  _updateDetector(projected, t) {
    // `projected` is already centered (gravity/orientation removed via meanVec in addSample), so
    // this fast EMA only needs to smooth out sensor/vibration noise, not detrend a slow offset.
    const alphaSmooth = this._alphaFor(this.smoothTauMs, t);
    this.smoothed = this.smoothed === null ? projected : this.smoothed + alphaSmooth * (projected - this.smoothed);

    this._lastSignal = this.smoothed;
    this._lastTime = t;

    // Adaptive amplitude envelope: track how big recent swings have been so the trigger threshold
    // scales with stroke intensity instead of a fixed number (rises fast, decays slowly).
    const absVal = Math.abs(this.smoothed);
    const envAlpha = absVal > this.envelope ? 0.3 : 0.02;
    this.envelope = this.envelope + envAlpha * (absVal - this.envelope);

    const riseThreshold = Math.max(this.envelope * this.thresholdFraction, this.minAmplitude);
    const releaseThreshold = Math.max(
      this.envelope * this.releaseFraction,
      this.minAmplitude * (this.releaseFraction / this.thresholdFraction)
    );

    if (this.armed && this.smoothed > riseThreshold) {
      this.armed = false;
      this._peakCandidate = this.smoothed;
    } else if (!this.armed) {
      if (this.smoothed > this._peakCandidate) this._peakCandidate = this.smoothed;
      if (this.smoothed < releaseThreshold) {
        this.armed = true;
        this._confirmStroke(t);
      }
    }
  }

  _confirmStroke(t) {
    if (t - this.lastStrokeTime < this.minStrokeIntervalMs) return; // refractory period
    this.lastStrokeTime = t;
    this.strokeCount += 1;
    this.strokeTimestamps.push(t);
    if (this.strokeTimestamps.length > 8) this.strokeTimestamps.shift();

    if (this.onStroke) {
      this.onStroke({ count: this.strokeCount, rate: this.getRate(), t });
    }
  }

  getCount() {
    return this.strokeCount;
  }

  // Strokes per minute, averaged over the last few strokes (0 until there are at least two to
  // measure an interval between).
  getRate() {
    const ts = this.strokeTimestamps;
    if (ts.length < 2) return 0;
    const spanMs = ts[ts.length - 1] - ts[0];
    const intervals = ts.length - 1;
    if (spanMs <= 0) return 0;
    return (intervals / spanMs) * 60000;
  }
}
