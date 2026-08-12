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
    private val estimator = RailEstimator(5.0)
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
    private lateinit var live: TextView
    private lateinit var calibrate: Button
    private lateinit var start: Button
    private lateinit var stop: Button
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
                    csvLogger?.append(s, estimator.gUsed())
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
        gEdit.isEnabled = true
        start.isEnabled = true
        calibrationInfo.text = String.format(
            Locale.US,
            "measured g = %.6f m/s²\ngravity direction in phone frame = [%.5f, %.5f, %.5f]\ngyro bias = [%.6f, %.6f, %.6f] rad/s",
            c.measuredG,
            c.measuredGravityVectorBody.normalized().x,
            c.measuredGravityVectorBody.normalized().y,
            c.measuredGravityVectorBody.normalized().z,
            c.gyroBiasBody.x, c.gyroBiasBody.y, c.gyroBiasBody.z
        )
        status.text = "Calibrated. You may edit g, then press Start. Do not move the phone after calibration."
    }

    private fun startMeasurement() {
        val g = gEdit.text.toString().toDoubleOrNull()
        if (estimator.calibration() == null) { toast("Calibrate first"); return }
        if (g == null || g !in 5.0..15.0) { toast("Invalid g value"); return }
        completedCsv = null
        csvLogger = CsvLogger(this)
        estimator.start(g)
        lastUiNs = 0L
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrate.isEnabled = false
        start.isEnabled = false
        stop.isEnabled = true
        export.isEnabled = false
        gEdit.isEnabled = false
        status.text = "Recording. Reference frame = phone frame at Start; no forward-direction calibration."
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
        export.isEnabled = completedCsv != null
        gEdit.isEnabled = true
        status.text = "$reason. Export CSV if you want to analyze the run."
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
                "raw gyro  [%+.5f, %+.5f, %+.5f] rad/s\n\n" +
                "a_local    [%+.4f, %+.4f, %+.4f] m/s²\n" +
                "v_local    [%+.4f, %+.4f, %+.4f] m/s\n" +
                "p_local    [%+.2f, %+.2f, %+.2f] m\n\n" +
                "speed      %.3f m/s   (%.2f km/h)\n" +
                "|position| %.2f m\n" +
                "time       %.1f s",
            latestAccel.x, latestAccel.y, latestAccel.z,
            latestGyro.x, latestGyro.y, latestGyro.z,
            s.filteredAccelLocal.x, s.filteredAccelLocal.y, s.filteredAccelLocal.z,
            s.velocityLocal.x, s.velocityLocal.y, s.velocityLocal.z,
            s.positionLocal.x, s.positionLocal.y, s.positionLocal.z,
            s.speed, s.speed * 3.6, s.displacement, s.elapsedSeconds
        )
    }

    private fun buildUi() {
        val d = resources.displayMetrics.density
        fun dp(v: Int) = (v*d).toInt()
        fun text(t: String, sp: Float = 14f) = TextView(this).apply { text = t; textSize = sp; setPadding(0, dp(5), 0, dp(5)) }
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(18), dp(18), dp(18), dp(28)) }
        root.addView(text("Rail IMU", 28f))
        root.addView(text("Raw accelerometer + gyroscope; ~5 min rail experiment", 14f))
        status = text("Initializing…", 15f); root.addView(status)

        calibrate = Button(this).apply { text = "Calibrate 5 s"; setOnClickListener { beginCalibration() } }
        start = Button(this).apply { text = "Start"; isEnabled = false; setOnClickListener { startMeasurement() } }
        stop = Button(this).apply { text = "Stop"; isEnabled = false; setOnClickListener { stopMeasurement() } }
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(calibrate, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(start, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(stop, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(row)

        val gRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        gRow.addView(text("g used (m/s²): ", 14f))
        gEdit = EditText(this).apply { setText("9.80665"); isEnabled = false }
        gRow.addView(gEdit, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(gRow)

        calibrationInfo = text("Not calibrated", 13f); root.addView(calibrationInfo)
        live = text("No measurement", 16f).apply { typeface = android.graphics.Typeface.MONOSPACE }; root.addView(live)
        export = Button(this).apply { text = "Export last CSV"; isEnabled = false; setOnClickListener { exportCsv() } }; root.addView(export)
        root.addView(text("Pipeline: TYPE_ACCELEROMETER + TYPE_GYROSCOPE → stationary g/gyro-bias calibration → gyro attitude integration → rotate acceleration into Start-local frame → subtract calibrated gravity → 5 Hz low-pass → trapezoidal integration. No forward calibration, no GPS, no magnetometer, no Android LINEAR_ACCELERATION.", 12f))
        setContentView(ScrollView(this).apply { addView(root) })
    }

    private fun toast(s: String) = Toast.makeText(this, s, Toast.LENGTH_SHORT).show()

    companion object { private const val REQUEST_EXPORT = 1001 }
}
