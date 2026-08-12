package com.openai.railimu

import android.app.Activity
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import java.io.File
import java.util.Locale

class MainActivity : Activity(), SensorEventListener {
    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private val estimator = RailEstimator()
    private val handler = Handler(Looper.getMainLooper())

    private var calibrationAccumulator: CalibrationAccumulator? = null
    private var csvLogger: CsvLogger? = null
    private var completedCsv: File? = null
    private var latestAccel = Vec3.ZERO
    private var latestGyro = Vec3.ZERO
    private var lastUiNs = 0L

    private lateinit var status: TextView
    private lateinit var calibrationInfo: TextView
    private lateinit var gEdit: EditText
    private lateinit var cutoffEdit: EditText
    private lateinit var gravityTauEdit: EditText
    private lateinit var gravityGateEdit: EditText
    private lateinit var maxCorrEdit: EditText
    private lateinit var live: TextView
    private lateinit var calibrate: Button
    private lateinit var start: Button
    private lateinit var stop: Button
    private lateinit var zeroV: Button
    private lateinit var export: Button

    private val finishCalibrationRunnable = Runnable { finishCalibration() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        buildUi()
        if (accelerometer == null || gyroscope == null) {
            status.text = "Raw accelerometer or gyroscope is unavailable on this phone."
            calibrate.isEnabled = false
        } else {
            status.text = "Fix the phone rigidly to the train. Calibrate while the train is stationary."
        }
    }

    override fun onResume() {
        super.onResume()
        accelerometer?.let { sensorManager.registerListener(this, it, 10_000) }
        gyroscope?.let { sensorManager.registerListener(this, it, 10_000) }
    }

    override fun onPause() {
        super.onPause()
        if (estimator.isMeasuring()) stopMeasurement("Stopped because app left foreground")
        sensorManager.unregisterListener(this)
    }

    override fun onSensorChanged(event: SensorEvent) {
        val v = Vec3(event.values[0].toDouble(), event.values[1].toDouble(), event.values[2].toDouble())
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                latestAccel = v
                calibrationAccumulator?.addAccel(v)
                if (estimator.isMeasuring()) {
                    val s = estimator.onAccelerometer(event.timestamp, v)
                    csvLogger?.append(s, estimator.gUsed(), estimator.parameters())
                    if (event.timestamp - lastUiNs >= 100_000_000L) {
                        lastUiNs = event.timestamp
                        refreshUi(s)
                    }
                }
            }
            Sensor.TYPE_GYROSCOPE -> {
                latestGyro = v
                calibrationAccumulator?.addGyro(v)
                estimator.onGyroscope(event.timestamp, v)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun beginCalibration() {
        if (estimator.isMeasuring() || calibrationAccumulator != null) return
        calibrationAccumulator = CalibrationAccumulator()
        calibrate.isEnabled = false
        start.isEnabled = false
        gEdit.isEnabled = false
        status.text = "Calibrating for 5 s. Keep phone and train stationary."
        handler.removeCallbacks(finishCalibrationRunnable)
        handler.postDelayed(finishCalibrationRunnable, 5_000)
    }

    private fun finishCalibration() {
        val acc = calibrationAccumulator ?: return
        calibrationAccumulator = null
        calibrate.isEnabled = true
        val c = runCatching { acc.result() }.getOrElse {
            status.text = "Calibration failed: ${it.message}"
            return
        }
        estimator.setCalibration(c)
        gEdit.setText(String.format(Locale.US, "%.6f", c.measuredG))
        setSettingsEnabled(true)
        start.isEnabled = true
        calibrationInfo.text = String.format(
            Locale.US,
            "measured g = %.6f m/s²\ngravity/support direction in phone frame = [%.5f, %.5f, %.5f]\ngyro bias = [%.6f, %.6f, %.6f] rad/s",
            c.measuredG,
            c.measuredGravityVectorBody.normalized().x,
            c.measuredGravityVectorBody.normalized().y,
            c.measuredGravityVectorBody.normalized().z,
            c.gyroBiasBody.x, c.gyroBiasBody.y, c.gyroBiasBody.z
        )
        status.text = "Calibrated. Edit parameters if desired, then Start."
    }

    private fun readParameters(): RailEstimator.Parameters? {
        val p = RailEstimator.Parameters(
            lowPassCutoffHz = cutoffEdit.text.toString().toDoubleOrNull() ?: return null,
            gravityCorrectionTauS = gravityTauEdit.text.toString().toDoubleOrNull() ?: return null,
            gravityMagnitudeGateMps2 = gravityGateEdit.text.toString().toDoubleOrNull() ?: return null,
            maxGravityCorrectionDegPerSec = maxCorrEdit.text.toString().toDoubleOrNull() ?: return null
        )
        return runCatching { p.validate(); p }.getOrNull()
    }

    private fun startMeasurement() {
        val g = gEdit.text.toString().toDoubleOrNull()
        val p = readParameters()
        if (estimator.calibration() == null) { toast("Calibrate first"); return }
        if (g == null || g !in 5.0..15.0) { toast("Invalid g value"); return }
        if (p == null) { toast("One or more tuning parameters are invalid"); return }
        completedCsv = null
        csvLogger = CsvLogger(this)
        estimator.start(g, p)
        lastUiNs = 0L
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrate.isEnabled = false
        start.isEnabled = false
        stop.isEnabled = true
        zeroV.isEnabled = true
        export.isEnabled = false
        setSettingsEnabled(false)
        status.text = "Recording. Local frame = phone frame at Start. Manual Zero V is safe only when you know the train is stopped."
        refreshUi(estimator.state())
    }

    private fun stopMeasurement(reason: String = "Stopped") {
        if (!estimator.isMeasuring()) return
        estimator.stop()
        completedCsv = csvLogger?.close()
        csvLogger = null
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrate.isEnabled = true
        start.isEnabled = true
        stop.isEnabled = false
        zeroV.isEnabled = false
        export.isEnabled = completedCsv != null
        setSettingsEnabled(true)
        status.text = "$reason. Export CSV to compare tuning parameters."
    }

    private fun zeroVelocityNow() {
        if (!estimator.isMeasuring()) return
        estimator.zeroVelocity()
        toast("Velocity reset to zero")
        refreshUi(estimator.state())
    }

    private fun exportCsv() {
        val src = completedCsv ?: run { toast("No completed CSV"); return }
        val intent = Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "text/csv"
            putExtra(Intent.EXTRA_TITLE, "railimu_${System.currentTimeMillis()}.csv")
        }
        @Suppress("DEPRECATION")
        startActivityForResult(intent, REQUEST_EXPORT)
    }

    @Deprecated("Deprecated in Java")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode != REQUEST_EXPORT || resultCode != RESULT_OK) return
        val uri: Uri = data?.data ?: return
        val src = completedCsv ?: return
        runCatching {
            contentResolver.openOutputStream(uri)?.use { out -> src.inputStream().use { it.copyTo(out) } }
                ?: error("Could not open destination")
        }.onSuccess { toast("CSV exported") }
            .onFailure { toast("Export failed: ${it.message}") }
    }

    private fun refreshUi(s: RailEstimator.State) {
        live.text = String.format(
            Locale.US,
            "raw accel  [%+.4f, %+.4f, %+.4f] m/s²\n" +
                "raw gyro   [%+.5f, %+.5f, %+.5f] rad/s\n\n" +
                "a filtered [%+.4f, %+.4f, %+.4f] m/s²\n" +
                "v local    [%+.4f, %+.4f, %+.4f] m/s\n" +
                "p local    [%+.2f, %+.2f, %+.2f] m\n\n" +
                "speed      %.3f m/s   (%.2f km/h)\n" +
                "|position| %.2f m\n" +
                "time       %.1f s\n\n" +
                "tilt corr  %s   |a|-g err %.4f\n" +
                "corr omega [%+.5f, %+.5f, %+.5f] rad/s\n" +
                "manual ZUPT count %d",
            latestAccel.x, latestAccel.y, latestAccel.z,
            latestGyro.x, latestGyro.y, latestGyro.z,
            s.filteredAccelLocal.x, s.filteredAccelLocal.y, s.filteredAccelLocal.z,
            s.velocityLocal.x, s.velocityLocal.y, s.velocityLocal.z,
            s.positionLocal.x, s.positionLocal.y, s.positionLocal.z,
            s.speed, s.speed * 3.6, s.displacement, s.elapsedSeconds,
            if (s.gravityCorrectionActive) "ON" else "OFF", s.accelMagnitudeError,
            s.gravityCorrectionOmegaBody.x, s.gravityCorrectionOmegaBody.y, s.gravityCorrectionOmegaBody.z,
            s.manualZuptCount
        )
    }

    private fun buildUi() {
        val d = resources.displayMetrics.density
        fun dp(v: Int) = (v * d).toInt()
        fun text(t: String, sp: Float = 14f) = TextView(this).apply { text = t; textSize = sp; setPadding(0, dp(5), 0, dp(5)) }
        fun numberField(default: String) = EditText(this).apply {
            setText(default)
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
            isSingleLine = true
        }
        fun paramRow(root: LinearLayout, label: String, field: EditText, hint: String) {
            val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
            row.addView(text(label, 13f), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.35f))
            row.addView(field, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 0.65f))
            root.addView(row)
            root.addView(text(hint, 11f))
        }

        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(18), dp(18), dp(18), dp(28)) }
        root.addView(text("Rail IMU v0.2", 28f))
        root.addView(text("Raw accelerometer + gyroscope; tunable rail inertial experiment", 14f))
        status = text("Initializing…", 15f); root.addView(status)

        calibrate = Button(this).apply { text = "Calibrate 5 s"; setOnClickListener { beginCalibration() } }
        start = Button(this).apply { text = "Start"; isEnabled = false; setOnClickListener { startMeasurement() } }
        stop = Button(this).apply { text = "Stop"; isEnabled = false; setOnClickListener { stopMeasurement() } }
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(calibrate, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(start, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(stop, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(row)

        root.addView(text("Tuning parameters", 18f))
        gEdit = numberField("9.80665").apply { isEnabled = false }
        cutoffEdit = numberField("1.0")
        gravityTauEdit = numberField("60.0")
        gravityGateEdit = numberField("0.20")
        maxCorrEdit = numberField("0.15")
        paramRow(root, "g used (m/s²)", gEdit, "Filled from calibration; you can override it.")
        paramRow(root, "LPF cutoff (Hz)", cutoffEdit, "2nd-order Butterworth. Lower = smoother but more lag. Valid 0.05–20.")
        paramRow(root, "Tilt correction tau (s)", gravityTauEdit, "Accelerometer slowly corrects gyro roll/pitch drift. Larger = weaker/slower; 0 disables. Default 60.")
        paramRow(root, "|a|-g gate (m/s²)", gravityGateEdit, "Tilt correction runs only when accelerometer magnitude is this close to g. Default 0.20.")
        paramRow(root, "Max tilt corr (deg/s)", maxCorrEdit, "Hard limit on accelerometer attitude correction rate. Default 0.15.")

        calibrationInfo = text("Not calibrated", 13f); root.addView(calibrationInfo)
        live = text("No measurement", 16f).apply { typeface = android.graphics.Typeface.MONOSPACE }; root.addView(live)
        zeroV = Button(this).apply {
            text = "ZERO VELOCITY NOW (train stopped)"
            isEnabled = false
            setOnClickListener { zeroVelocityNow() }
        }
        root.addView(zeroV)
        export = Button(this).apply { text = "Export last CSV"; isEnabled = false; setOnClickListener { exportCsv() } }; root.addView(export)
        root.addView(text("Important: automatic ZUPT is intentionally NOT used. An IMU cannot distinguish a stopped train from perfectly constant-speed cruising, so auto-zeroing from low acceleration/gyro alone would be physically wrong. Use Zero Velocity only at a known stop.\n\nPipeline: raw TYPE_ACCELEROMETER + TYPE_GYROSCOPE → calibrated g/gyro bias → gyro attitude + gated slow accelerometer tilt correction → Start-local gravity subtraction → tunable 2nd-order Butterworth LPF → trapezoidal integration.", 12f))
        setContentView(ScrollView(this).apply { addView(root) })
    }

    private fun setSettingsEnabled(enabled: Boolean) {
        gEdit.isEnabled = enabled && estimator.calibration() != null
        cutoffEdit.isEnabled = enabled
        gravityTauEdit.isEnabled = enabled
        gravityGateEdit.isEnabled = enabled
        maxCorrEdit.isEnabled = enabled
    }

    private fun toast(s: String) = Toast.makeText(this, s, Toast.LENGTH_SHORT).show()

    companion object { private const val REQUEST_EXPORT = 1001 }
}
