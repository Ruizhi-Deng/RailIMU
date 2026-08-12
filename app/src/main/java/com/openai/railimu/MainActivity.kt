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
import android.widget.*
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

    private lateinit var status: TextView
    private lateinit var calibrationInfo: TextView
    private lateinit var live: TextView
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
        if (accel == null || gyro == null) {
            status.text = "Raw accelerometer or gyroscope unavailable."
            calibrateBtn.isEnabled = false
        } else status.text = "Fix phone rigidly to train. Initial calibration must be fully stationary."
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
                latestA = v; initialAcc?.addAccel(v); stopAcc?.addAccel(v)
                if (estimator.isMeasuring()) {
                    val s = estimator.onAccelerometer(e.timestamp, v)
                    if (e.timestamp - lastUiNs >= 100_000_000L) { lastUiNs = e.timestamp; refreshUi(s) }
                }
            }
            Sensor.TYPE_GYROSCOPE -> {
                latestW = v; initialAcc?.addGyro(v); stopAcc?.addGyro(v)
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
        initialAcc = CalibrationAccumulator(); calibrateBtn.isEnabled = false; startBtn.isEnabled = false
        status.text = "Initial calibration: keep train and phone stationary for 5 s."
        handler.postDelayed({ finishInitialCalibration() }, 5_000)
    }

    private fun finishInitialCalibration() {
        val a = initialAcc ?: return; initialAcc = null; calibrateBtn.isEnabled = true
        val c = runCatching { a.result() }.getOrElse { status.text = "Calibration failed: ${it.message}"; return }
        estimator.setCalibration(c); gEdit.setText(String.format(Locale.US, "%.6f", c.measuredG)); startBtn.isEnabled = true; setSettingsEnabled(true)
        calibrationInfo.text = String.format(Locale.US,
            "g=%.6f m/s²   accel 3D std=%.5f m/s²\ngyro bias=[%+.6f,%+.6f,%+.6f] rad/s   gyro 3D std=%.6f rad/s",
            c.measuredG,c.accelStdMps2,c.gyroBiasBody.x,c.gyroBiasBody.y,c.gyroBiasBody.z,c.gyroStdRadS)
        status.text = "Calibrated. Edit parameters if desired, then Start."
    }

    private fun startMeasurement() {
        val g = gEdit.text.toString().toDoubleOrNull(); val p = readParams()
        if (estimator.calibration() == null) { toast("Calibrate first"); return }
        if (g == null || g !in 5.0..15.0 || p == null) { toast("Invalid parameter"); return }
        estimator.start(g,p); completedCsv=null; lastUiNs=0L; window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        calibrateBtn.isEnabled=false; startBtn.isEnabled=false; stopBtn.isEnabled=true; stationaryBtn.isEnabled=true; exportBtn.isEnabled=false; setSettingsEnabled(false)
        status.text = "RUNNING: live values are provisional. At a known full stop, press STATIONARY UPDATE to close and reprocess the segment."
        refreshUi(estimator.state())
    }

    private fun beginStationaryUpdate() {
        if (!estimator.isMeasuring() || stopCapture) return
        val p=estimator.parameters(); stopCapture=true; stopAcc=CalibrationAccumulator(); stationaryBtn.isEnabled=false; stopBtn.isEnabled=false
        status.text = String.format(Locale.US,"Stationary update: hold fully stopped for %.1f s…",p.stationaryWindowS)
        handler.postDelayed({ finishStationaryUpdate() }, (p.stationaryWindowS*1000.0).toLong())
    }

    private fun finishStationaryUpdate() {
        if (!stopCapture) return
        val a=stopAcc; stopAcc=null; stopCapture=false; stationaryBtn.isEnabled=true; stopBtn.isEnabled=true
        val c=runCatching { requireNotNull(a).result() }.getOrElse { status.text="Stationary capture failed: ${it.message}"; return }
        val p=estimator.parameters()
        if (c.accelStdMps2 > p.stationaryAccelStdMaxMps2 || c.gyroStdRadS > p.stationaryGyroStdMaxRadS) {
            status.text=String.format(Locale.US,"Rejected: not stationary enough. accel std %.4f (max %.4f), gyro std %.5f (max %.5f). Segment NOT closed.",c.accelStdMps2,p.stationaryAccelStdMaxMps2,c.gyroStdRadS,p.stationaryGyroStdMaxRadS)
            return
        }
        val m=runCatching { estimator.finalizeStationary(c) }.getOrElse { status.text="Segment closure failed: ${it.message}"; return }
        status.text=String.format(Locale.US,"Segment %d finalized: raw end speed %.3f m/s → constrained toward 0; tilt closure %.3f°; corrected segment distance %.1f m. New segment started.",m.segmentId,m.rawEndpointSpeed,m.tiltClosureDeg,m.correctedDistanceM)
        refreshUi(estimator.state())
    }

    private fun cancelStopCapture(){ stopCapture=false; stopAcc=null; handler.removeCallbacksAndMessages(null) }

    private fun stopMeasurement(reason:String="Stopped") {
        if (!estimator.isMeasuring()) return
        cancelStopCapture(); estimator.stop(); completedCsv=CsvLogger.write(this,estimator.exportSamples(),estimator.segmentSummaries(),estimator.gUsed(),estimator.parameters())
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); calibrateBtn.isEnabled=true; startBtn.isEnabled=true; stopBtn.isEnabled=false; stationaryBtn.isEnabled=false; exportBtn.isEnabled=true; setSettingsEnabled(true)
        status.text="$reason. CSV contains finalized corrected segments plus any unfinished provisional segment."
    }

    private fun exportCsv() {
        val src=completedCsv?:run{toast("No CSV");return}
        val i=Intent(Intent.ACTION_CREATE_DOCUMENT).apply{addCategory(Intent.CATEGORY_OPENABLE);type="text/csv";putExtra(Intent.EXTRA_TITLE,"railimu_v03_${System.currentTimeMillis()}.csv")}
        @Suppress("DEPRECATION") startActivityForResult(i,1001)
    }

    @Deprecated("Deprecated in Java")
    override fun onActivityResult(requestCode:Int,resultCode:Int,data:Intent?){super.onActivityResult(requestCode,resultCode,data);if(requestCode!=1001||resultCode!=RESULT_OK)return;val u:Uri=data?.data?:return;val f=completedCsv?:return;runCatching{contentResolver.openOutputStream(u)!!.use{out->f.inputStream().use{it.copyTo(out)}}}.onSuccess{toast("CSV exported")}.onFailure{toast("Export failed: ${it.message}")}}

    private fun refreshUi(s: RailEstimator.State) {
        val m=s.lastSegmentSummary
        live.text=String.format(Locale.US,
            "segment %d   %s\nraw accel [%+.4f,%+.4f,%+.4f] m/s²\nraw gyro  [%+.5f,%+.5f,%+.5f] rad/s\n\nLIVE provisional:\na filt [%+.4f,%+.4f,%+.4f]\nv      [%+.4f,%+.4f,%+.4f] m/s\np      [%+.2f,%+.2f,%+.2f] m\nspeed  %.3f m/s (%.2f km/h)\ntime   %.1f s\n\nfinalized segments %d\ncorrected total distance %.1f m%s",
            s.segmentId,if(s.currentSegmentFinalized)"just finalized" else "open",
            latestA.x,latestA.y,latestA.z,latestW.x,latestW.y,latestW.z,
            s.filteredAccelLocal.x,s.filteredAccelLocal.y,s.filteredAccelLocal.z,
            s.velocityLocal.x,s.velocityLocal.y,s.velocityLocal.z,
            s.positionLocal.x,s.positionLocal.y,s.positionLocal.z,
            s.speedMps,s.speedMps*3.6,s.elapsedSeconds,s.finalizedSegments,s.correctedTotalDistanceM,
            if(m==null)"" else String.format(Locale.US,"\nlast closure: end %.3f m/s, tilt %.3f°, dist %.1f m",m.rawEndpointSpeed,m.tiltClosureDeg,m.correctedDistanceM))
    }

    private fun buildUi(){
        val d=resources.displayMetrics.density; fun dp(v:Int)=(v*d).toInt(); fun t(s:String,sp:Float=14f)=TextView(this).apply{text=s;textSize=sp;setPadding(0,dp(4),0,dp(4))}
        fun nf(v:String)=EditText(this).apply{setText(v);inputType=InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED;isSingleLine=true}
        fun row(root:LinearLayout,label:String,f:EditText,hint:String){val r=LinearLayout(this).apply{orientation=LinearLayout.HORIZONTAL;gravity=Gravity.CENTER_VERTICAL};r.addView(t(label,13f),LinearLayout.LayoutParams(0,ViewGroup.LayoutParams.WRAP_CONTENT,1.35f));r.addView(f,LinearLayout.LayoutParams(0,ViewGroup.LayoutParams.WRAP_CONTENT,0.65f));root.addView(r);root.addView(t(hint,11f))}
        val root=LinearLayout(this).apply{orientation=LinearLayout.VERTICAL;setPadding(dp(18),dp(18),dp(18),dp(28))};root.addView(t("Rail IMU v0.3",28f));root.addView(t("Stop-to-stop inertial estimation with retrospective drift closure",14f));status=t("Initializing…",15f);root.addView(status)
        calibrateBtn=Button(this).apply{text="CALIBRATE 5 S";setOnClickListener{beginInitialCalibration()}};startBtn=Button(this).apply{text="START";isEnabled=false;setOnClickListener{startMeasurement()}};stopBtn=Button(this).apply{text="STOP";isEnabled=false;setOnClickListener{stopMeasurement()}}
        val rr=LinearLayout(this).apply{orientation=LinearLayout.HORIZONTAL};rr.addView(calibrateBtn,LinearLayout.LayoutParams(0,ViewGroup.LayoutParams.WRAP_CONTENT,1f));rr.addView(startBtn,LinearLayout.LayoutParams(0,ViewGroup.LayoutParams.WRAP_CONTENT,1f));rr.addView(stopBtn,LinearLayout.LayoutParams(0,ViewGroup.LayoutParams.WRAP_CONTENT,1f));root.addView(rr)
        root.addView(t("Tuning parameters",18f));gEdit=nf("9.80665").apply{isEnabled=false};cutoffEdit=nf("1.0");stopWindowEdit=nf("2.0");accelStdEdit=nf("0.08");gyroStdEdit=nf("0.006");biasBridgeEdit=nf("1.0");tiltClosureEdit=nf("1.0");endpointVelEdit=nf("1.0")
        row(root,"g used (m/s²)",gEdit,"Filled by calibration; editable.");row(root,"LPF cutoff (Hz)",cutoffEdit,"2nd-order Butterworth, 0.05–20.");row(root,"Stop window (s)",stopWindowEdit,"Stationary data collected after pressing update, 0.5–10.");row(root,"Stop accel std max",accelStdEdit,"Reject stop update if 3D accel noise/motion exceeds this m/s².");row(root,"Stop gyro std max",gyroStdEdit,"Reject stop update if 3D gyro variation exceeds this rad/s.");row(root,"Gyro bias bridge",biasBridgeEdit,"0=no end-stop bias correction; 1=linear start→end bias drift model.");row(root,"Tilt closure strength",tiltClosureEdit,"0=no endpoint gravity alignment; 1=full roll/pitch closure. Yaw is not forced.");row(root,"Endpoint v correction",endpointVelEdit,"0=no v(T)=0 correction; 1=full linear velocity-drift closure.")
        calibrationInfo=t("Not calibrated",13f);root.addView(calibrationInfo);live=t("No measurement",15f).apply{typeface=android.graphics.Typeface.MONOSPACE};root.addView(live)
        stationaryBtn=Button(this).apply{text="STATIONARY UPDATE (KNOWN STOP)";isEnabled=false;setOnClickListener{beginStationaryUpdate()}};root.addView(stationaryBtn);exportBtn=Button(this).apply{text="EXPORT CSV";isEnabled=false;setOnClickListener{exportCsv()}};root.addView(exportBtn)
        root.addView(t("v0.3 principle: during motion, accelerometer is NOT used to continuously correct tilt, because train acceleration and tilt are not separable from accelerometer alone. At a known stop, the app gets a trustworthy endpoint: v=0, gravity direction, and gyro bias. It then replays the whole segment, bridges gyro bias from start stop to end stop, closes roll/pitch using endpoint gravity, and applies the v(T)=0 constraint. Live values can drift; finalized stop-to-stop results are the intended output.",12f));setContentView(ScrollView(this).apply{addView(root)})
    }

    private fun setSettingsEnabled(x:Boolean){gEdit.isEnabled=x&&estimator.calibration()!=null;cutoffEdit.isEnabled=x;stopWindowEdit.isEnabled=x;accelStdEdit.isEnabled=x;gyroStdEdit.isEnabled=x;biasBridgeEdit.isEnabled=x;tiltClosureEdit.isEnabled=x;endpointVelEdit.isEnabled=x}
    private fun toast(s:String)=Toast.makeText(this,s,Toast.LENGTH_SHORT).show()
}
