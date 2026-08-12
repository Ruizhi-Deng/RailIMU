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
import android.view.View
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import java.io.File
import java.util.Locale

class MainActivity : Activity(), SensorEventListener {
    private lateinit var sm: SensorManager
    private var accel: Sensor? = null
    private var gyro: Sensor? = null
    private val estimator = RailEstimator()
    private val handler = Handler(Looper.getMainLooper())

    private var initialAcc: CalibrationAccumulator? = null
    private var stopAcc: CalibrationAccumulator? = null
    private var stopCapture = false
    private var completedCsv: File? = null
    private var latestA = Vec3.ZERO
    private var latestW = Vec3.ZERO
    private var lastUiNs = 0L
    private var lastRouteUiNs = 0L
    private var currentPage = PAGE_RUN

    private lateinit var runPage: View
    private lateinit var routePage: View
    private lateinit var settingsPage: View
    private lateinit var runTab: Button
    private lateinit var routeTab: Button
    private lateinit var settingsTab: Button

    private lateinit var status: TextView
    private lateinit var calibrationInfo: TextView
    private lateinit var speedHero: TextView
    private lateinit var live: TextView
    private lateinit var routeInfo: TextView
    private lateinit var trajectoryView: TrajectoryView

    private lateinit var gEdit: EditText
    private lateinit var cutoffEdit: EditText
    private lateinit var stopWindowEdit: EditText
    private lateinit var accelStdEdit: EditText
    private lateinit var gyroStdEdit: EditText
    private lateinit var biasBridgeEdit: EditText
    private lateinit var tiltClosureEdit: EditText
    private lateinit var endpointVelEdit: EditText

    private lateinit var calibrateBtn: Button
    private lateinit var startBtn: Button
    private lateinit var stopBtn: Button
    private lateinit var stationaryBtn: Button
    private lateinit var exportBtn: Button

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        sm = getSystemService(SENSOR_SERVICE) as SensorManager
        accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        buildUi()
        showPage(PAGE_RUN)
        if (accel == null || gyro == null) {
            status.text = "Raw accelerometer or gyroscope unavailable."
            calibrateBtn.isEnabled = false
        } else {
            status.text = "Fix phone rigidly to train. Calibrate while the train is fully stationary."
        }
    }

    override fun onResume() {
        super.onResume()
        accel?.let { sm.registerListener(this, it, 10_000) }
        gyro?.let { sm.registerListener(this, it, 10_000) }
    }

    override fun onPause() {
        super.onPause()
        cancelStopCapture()
        if (estimator.isMeasuring()) stopMeasurement("Stopped because app left foreground")
        sm.unregisterListener(this)
    }

    override fun onSensorChanged(e: SensorEvent) {
        val v = Vec3(e.values[0].toDouble(), e.values[1].toDouble(), e.values[2].toDouble())
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                latestA = v
                initialAcc?.addAccel(v)
                stopAcc?.addAccel(v)
                if (estimator.isMeasuring()) {
                    val s = estimator.onAccelerometer(e.timestamp, v)
                    if (e.timestamp - lastUiNs >= 100_000_000L) {
                        lastUiNs = e.timestamp
                        refreshUi(s)
                    }
                    if (currentPage == PAGE_ROUTE && e.timestamp - lastRouteUiNs >= 1_000_000_000L) {
                        lastRouteUiNs = e.timestamp
                        updateRouteView()
                    }
                }
            }
            Sensor.TYPE_GYROSCOPE -> {
                latestW = v
                initialAcc?.addGyro(v)
                stopAcc?.addGyro(v)
                estimator.onGyroscope(e.timestamp, v)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun readParams(): RailEstimator.Parameters? {
        val p = RailEstimator.Parameters(
            lowPassCutoffHz = cutoffEdit.text.toString().toDoubleOrNull() ?: return null,
            stationaryWindowS = stopWindowEdit.text.toString().toDoubleOrNull() ?: return null,
            stationaryAccelStdMaxMps2 = accelStdEdit.text.toString().toDoubleOrNull() ?: return null,
            stationaryGyroStdMaxRadS = gyroStdEdit.text.toString().toDoubleOrNull() ?: return null,
            gyroBiasBridgeStrength = biasBridgeEdit.text.toString().toDoubleOrNull() ?: return null,
            tiltClosureStrength = tiltClosureEdit.text.toString().toDoubleOrNull() ?: return null,
            endpointVelocityCorrectionStrength = endpointVelEdit.text.toString().toDoubleOrNull() ?: return null
        )
        return runCatching { p.validate(); p }.getOrNull()
    }

    private fun beginInitialCalibration() {
        if (estimator.isMeasuring() || initialAcc != null) return
        initialAcc = CalibrationAccumulator()
        calibrateBtn.isEnabled = false
        startBtn.isEnabled = false
        status.text = "Initial calibration: keep train and phone stationary for 5 s."
        handler.postDelayed({ finishInitialCalibration() }, 5_000)
    }

    private fun finishInitialCalibration() {
        val acc = initialAcc ?: return
        initialAcc = null
        calibrateBtn.isEnabled = true
        val c = runCatching { acc.result() }.getOrElse {
            status.text = "Calibration failed: ${it.message}"
            return
        }
        estimator.setCalibration(c)
        gEdit.setText(String.format(Locale.US, "%.6f", c.measuredG))
        startBtn.isEnabled = true
        setSettingsEnabled(true)
        calibrationInfo.text = String.format(
            Locale.US,
            "g = %.6f m/s²\naccel 3D std = %.5f m/s²\ngyro bias = [%+.6f, %+.6f, %+.6f] rad/s\ngyro 3D std = %.6f rad/s",
            c.measuredG, c.accelStdMps2,
            c.gyroBiasBody.x, c.gyroBiasBody.y, c.gyroBiasBody.z,
            c.gyroStdRadS
        )
        status.text = "Calibrated. Adjust Settings if desired, then Start."
    }

    private fun startMeasurement() {
        val g = gEdit.text.toString().toDoubleOrNull()
        val p = readParams()
        if (estimator.calibration() == null) {
            toast("Calibrate first")
            return
        }
        if (g == null || g !in 5.0..15.0 || p == null) {
            toast("Invalid parameter in Settings")
            return
        }
        estimator.start(g, p)
        completedCsv = null
        lastUiNs = 0L
        lastRouteUiNs = 0L
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrateBtn.isEnabled = false
        startBtn.isEnabled = false
        stopBtn.isEnabled = true
        stationaryBtn.isEnabled = true
        exportBtn.isEnabled = false
        setSettingsEnabled(false)
        status.text = "RUNNING. Live velocity/route are provisional. At a known full stop, press STATIONARY UPDATE."
        refreshUi(estimator.state())
        updateRouteView()
    }

    private fun beginStationaryUpdate() {
        if (!estimator.isMeasuring() || stopCapture) return
        val p = estimator.parameters()
        stopCapture = true
        stopAcc = CalibrationAccumulator()
        stationaryBtn.isEnabled = false
        stopBtn.isEnabled = false
        status.text = String.format(Locale.US, "Stationary update: remain fully stopped for %.1f s…", p.stationaryWindowS)
        handler.postDelayed({ finishStationaryUpdate() }, (p.stationaryWindowS * 1000.0).toLong())
    }

    private fun finishStationaryUpdate() {
        if (!stopCapture) return
        val acc = stopAcc
        stopAcc = null
        stopCapture = false
        stationaryBtn.isEnabled = true
        stopBtn.isEnabled = true
        val c = runCatching { requireNotNull(acc).result() }.getOrElse {
            status.text = "Stationary capture failed: ${it.message}"
            return
        }
        val p = estimator.parameters()
        if (c.accelStdMps2 > p.stationaryAccelStdMaxMps2 || c.gyroStdRadS > p.stationaryGyroStdMaxRadS) {
            status.text = String.format(
                Locale.US,
                "Rejected: not stationary enough. accel std %.4f / %.4f, gyro std %.5f / %.5f. Segment remains open.",
                c.accelStdMps2, p.stationaryAccelStdMaxMps2,
                c.gyroStdRadS, p.stationaryGyroStdMaxRadS
            )
            return
        }
        val m = runCatching { estimator.finalizeStationary(c) }.getOrElse {
            status.text = "Segment closure failed: ${it.message}"
            return
        }
        status.text = String.format(
            Locale.US,
            "Segment %d finalized · raw end speed %.3f m/s · tilt closure %.3f° · corrected distance %.1f m",
            m.segmentId, m.rawEndpointSpeed, m.tiltClosureDeg, m.correctedDistanceM
        )
        refreshUi(estimator.state())
        updateRouteView()
    }

    private fun cancelStopCapture() {
        stopCapture = false
        stopAcc = null
        handler.removeCallbacksAndMessages(null)
    }

    private fun stopMeasurement(reason: String = "Stopped") {
        if (!estimator.isMeasuring()) return
        cancelStopCapture()
        estimator.stop()
        completedCsv = CsvLogger.write(
            this,
            estimator.exportSamples(),
            estimator.segmentSummaries(),
            estimator.gUsed(),
            estimator.parameters()
        )
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrateBtn.isEnabled = true
        startBtn.isEnabled = true
        stopBtn.isEnabled = false
        stationaryBtn.isEnabled = false
        exportBtn.isEnabled = true
        setSettingsEnabled(true)
        status.text = "$reason. Finalized route remains on Route; CSV is ready in Settings."
        updateRouteView()
    }

    private fun exportCsv() {
        val src = completedCsv ?: run { toast("No completed CSV"); return }
        val intent = Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "text/csv"
            putExtra(Intent.EXTRA_TITLE, "railimu_v04_${System.currentTimeMillis()}.csv")
        }
        @Suppress("DEPRECATION")
        startActivityForResult(intent, 1001)
    }

    @Deprecated("Deprecated in Java")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode != 1001 || resultCode != RESULT_OK) return
        val uri: Uri = data?.data ?: return
        val f = completedCsv ?: return
        runCatching {
            contentResolver.openOutputStream(uri)?.use { out ->
                f.inputStream().use { input -> input.copyTo(out) }
            } ?: error("Could not open destination")
        }.onSuccess { toast("CSV exported") }
            .onFailure { toast("Export failed: ${it.message}") }
    }

    private fun refreshUi(s: RailEstimator.State) {
        speedHero.text = String.format(Locale.US, "%.1f km/h", s.speedMps * 3.6)
        val m = s.lastSegmentSummary
        val closure = if (m == null) {
            "No finalized stop-to-stop segment yet."
        } else {
            String.format(
                Locale.US,
                "Last finalized: segment %d · %.1f m · endpoint drift %.3f m/s · tilt %.3f°",
                m.segmentId, m.correctedDistanceM, m.rawEndpointSpeed, m.tiltClosureDeg
            )
        }
        live.text = String.format(
            Locale.US,
            "Segment %d · %s\n\nAcceleration [%+.4f, %+.4f, %+.4f] m/s²\nVelocity     [%+.4f, %+.4f, %+.4f] m/s\nPosition     [%+.1f, %+.1f, %+.1f] m\nElapsed      %.1f s\n\nFinalized segments: %d\nCorrected distance: %.1f m\n%s",
            s.segmentId,
            if (s.currentSegmentFinalized) "just finalized" else "open / provisional",
            s.filteredAccelLocal.x, s.filteredAccelLocal.y, s.filteredAccelLocal.z,
            s.velocityLocal.x, s.velocityLocal.y, s.velocityLocal.z,
            s.positionLocal.x, s.positionLocal.y, s.positionLocal.z,
            s.elapsedSeconds, s.finalizedSegments, s.correctedTotalDistanceM, closure
        )
        updateRouteInfo(s)
    }

    private fun updateRouteInfo(s: RailEstimator.State = estimator.state()) {
        routeInfo.text = String.format(
            Locale.US,
            "%d finalized segment(s) · %.1f m corrected distance\nBlue = finalized · orange dashed = current provisional segment",
            s.finalizedSegments,
            s.correctedTotalDistanceM
        )
    }

    private fun updateRouteView() {
        val gravity = estimator.calibration()?.measuredGravityVectorBody?.normalized() ?: Vec3(0.0, 0.0, 1.0)
        trajectoryView.setData(estimator.exportSamples(), gravity)
        updateRouteInfo()
    }

    private fun showPage(page: Int) {
        currentPage = page
        runPage.visibility = if (page == PAGE_RUN) View.VISIBLE else View.GONE
        routePage.visibility = if (page == PAGE_ROUTE) View.VISIBLE else View.GONE
        settingsPage.visibility = if (page == PAGE_SETTINGS) View.VISIBLE else View.GONE
        runTab.isEnabled = page != PAGE_RUN
        routeTab.isEnabled = page != PAGE_ROUTE
        settingsTab.isEnabled = page != PAGE_SETTINGS
        if (page == PAGE_ROUTE) updateRouteView()
    }

    private fun buildUi() {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(12), dp(10), dp(12), dp(10))
        }
        root.addView(makeLabel("Rail IMU v0.4", 24f))

        val tabs = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        runTab = Button(this).apply { text = "RUN"; setOnClickListener { showPage(PAGE_RUN) } }
        routeTab = Button(this).apply { text = "ROUTE"; setOnClickListener { showPage(PAGE_ROUTE) } }
        settingsTab = Button(this).apply { text = "SETTINGS"; setOnClickListener { showPage(PAGE_SETTINGS) } }
        tabs.addView(runTab, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        tabs.addView(routeTab, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        tabs.addView(settingsTab, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(tabs)

        val host = FrameLayout(this)
        runPage = buildRunPage()
        routePage = buildRoutePage()
        settingsPage = buildSettingsPage()
        host.addView(runPage, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
        host.addView(routePage, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
        host.addView(settingsPage, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
        root.addView(host, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        setContentView(root)
    }

    private fun buildRunPage(): View {
        val content = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(6), dp(8), dp(6), dp(20))
        }
        status = makeLabel("Initializing…", 14f)
        content.addView(status)
        speedHero = makeLabel("0.0 km/h", 38f).apply {
            gravity = Gravity.CENTER_HORIZONTAL
            setPadding(0, dp(14), 0, dp(14))
        }
        content.addView(speedHero)

        calibrateBtn = Button(this).apply { text = "CALIBRATE 5 S"; setOnClickListener { beginInitialCalibration() } }
        startBtn = Button(this).apply { text = "START"; isEnabled = false; setOnClickListener { startMeasurement() } }
        stopBtn = Button(this).apply { text = "STOP"; isEnabled = false; setOnClickListener { stopMeasurement() } }
        val controls = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        controls.addView(calibrateBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        controls.addView(startBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        controls.addView(stopBtn, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        content.addView(controls)

        stationaryBtn = Button(this).apply {
            text = "STATIONARY UPDATE · KNOWN STOP"
            textSize = 16f
            isEnabled = false
            setOnClickListener { beginStationaryUpdate() }
        }
        content.addView(stationaryBtn)

        live = makeLabel("No measurement", 14f).apply {
            typeface = android.graphics.Typeface.MONOSPACE
            setPadding(0, dp(12), 0, dp(8))
        }
        content.addView(live)
        content.addView(makeLabel("Live values are provisional. A segment is corrected/finalized after a valid Stationary Update.", 11f))
        return ScrollView(this).apply { addView(content) }
    }

    private fun buildRoutePage(): View {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(6), dp(8), dp(6), dp(8))
        }
        routeInfo = makeLabel("No route yet", 13f)
        root.addView(routeInfo)

        trajectoryView = TrajectoryView(this)
        val projections = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        projections.addView(projectionButton("TOP", TrajectoryView.Projection.HORIZONTAL), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        projections.addView(projectionButton("XY", TrajectoryView.Projection.XY), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        projections.addView(projectionButton("XZ", TrajectoryView.Projection.XZ), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        projections.addView(projectionButton("YZ", TrajectoryView.Projection.YZ), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        root.addView(projections)
        root.addView(trajectoryView, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        root.addView(
            makeLabel(
                "TOP projects the 3D trajectory onto the plane perpendicular to calibrated gravity. Phone mounting tilt therefore does not matter. The route is relative, not north-referenced; IMU-only yaw drift can slowly rotate or distort the plan view.",
                11f
            )
        )
        return root
    }

    private fun projectionButton(textValue: String, mode: TrajectoryView.Projection): Button {
        return Button(this).apply {
            text = textValue
            textSize = 11f
            setOnClickListener {
                trajectoryView.projection = mode
                updateRouteView()
            }
        }
    }

    private fun buildSettingsPage(): View {
        val content = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(6), dp(8), dp(6), dp(24))
        }
        content.addView(makeLabel("Estimator parameters", 19f))

        gEdit = numberField("9.80665").apply { isEnabled = false }
        cutoffEdit = numberField("1.0")
        stopWindowEdit = numberField("2.0")
        accelStdEdit = numberField("0.08")
        gyroStdEdit = numberField("0.006")
        biasBridgeEdit = numberField("1.0")
        tiltClosureEdit = numberField("1.0")
        endpointVelEdit = numberField("1.0")

        addParamRow(content, "g used (m/s²)", gEdit, "Filled by calibration; editable.")
        addParamRow(content, "LPF cutoff (Hz)", cutoffEdit, "2nd-order Butterworth, 0.05–20 Hz.")
        addParamRow(content, "Stop window (s)", stopWindowEdit, "Static capture duration after Stationary Update.")
        addParamRow(content, "Stop accel std max", accelStdEdit, "Reject stop update above this 3D accel std (m/s²).")
        addParamRow(content, "Stop gyro std max", gyroStdEdit, "Reject stop update above this 3D gyro std (rad/s).")
        addParamRow(content, "Gyro bias bridge", biasBridgeEdit, "0 = off; 1 = linear start-stop → end-stop bias bridge.")
        addParamRow(content, "Tilt closure strength", tiltClosureEdit, "0 = off; 1 = full endpoint gravity roll/pitch closure.")
        addParamRow(content, "Endpoint v correction", endpointVelEdit, "0 = off; 1 = full v(T)=0 drift closure.")

        content.addView(makeLabel("Calibration", 19f))
        calibrationInfo = makeLabel("Not calibrated", 13f)
        content.addView(calibrationInfo)
        exportBtn = Button(this).apply { text = "EXPORT CSV"; isEnabled = false; setOnClickListener { exportCsv() } }
        content.addView(exportBtn)
        content.addView(makeLabel("Parameters are locked while recording so the complete run uses one estimator configuration.", 11f))
        return ScrollView(this).apply { addView(content) }
    }

    private fun addParamRow(root: LinearLayout, name: String, field: EditText, hint: String) {
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(makeLabel(name, 13f), LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.35f))
        row.addView(field, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 0.65f))
        root.addView(row)
        root.addView(makeLabel(hint, 11f))
    }

    private fun numberField(defaultValue: String): EditText {
        return EditText(this).apply {
            setText(defaultValue)
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
            isSingleLine = true
        }
    }

    private fun makeLabel(textValue: String, sp: Float = 14f): TextView {
        return TextView(this).apply {
            text = textValue
            textSize = sp
            setPadding(0, dp(4), 0, dp(4))
        }
    }

    private fun dp(v: Int): Int = (v * resources.displayMetrics.density).toInt()

    private fun setSettingsEnabled(enabled: Boolean) {
        gEdit.isEnabled = enabled && estimator.calibration() != null
        cutoffEdit.isEnabled = enabled
        stopWindowEdit.isEnabled = enabled
        accelStdEdit.isEnabled = enabled
        gyroStdEdit.isEnabled = enabled
        biasBridgeEdit.isEnabled = enabled
        tiltClosureEdit.isEnabled = enabled
        endpointVelEdit.isEnabled = enabled
    }

    private fun toast(s: String) = Toast.makeText(this, s, Toast.LENGTH_SHORT).show()

    companion object {
        private const val PAGE_RUN = 0
        private const val PAGE_ROUTE = 1
        private const val PAGE_SETTINGS = 2
    }
}
