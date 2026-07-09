package com.example.flutglove

import android.content.Context
import android.net.wifi.WifiManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel
import java.io.File

/**
 * flutglove Android host.
 *
 * ROS 2 needs three things wired up on Android that a desktop gets for free:
 *  1. The getifaddrs shim must be loaded EARLY (before rcl) so it can interpose
 *     for DDS interface discovery — done in the static initializer below.
 *  2. A Wi-Fi multicast lock, or DDS default (multicast) discovery is dropped
 *     by the OS.
 *  3. Dart needs the app's nativeLibraryDir (bundled ROS .so) and a writable
 *     ament index dir — provided over a MethodChannel.
 */
class MainActivity : FlutterActivity() {

    companion object {
        init {
            // Load the getifaddrs shim first; if the ROS closure isn't bundled
            // yet (dev build) this throws — swallow so the app still starts.
            try {
                System.loadLibrary("rcldart_ifaddrs")
            } catch (_: Throwable) {
            }
        }
    }

    private var multicastLock: WifiManager.MulticastLock? = null

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)

        // Hold a multicast lock so DDS discovery packets are delivered.
        val wifi = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        multicastLock = wifi.createMulticastLock("rcldart").apply {
            setReferenceCounted(true)
            acquire()
        }

        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, "rcldart/android")
            .setMethodCallHandler { call, result ->
                when (call.method) {
                    // Where Android extracted the bundled ROS .so closure.
                    "nativeLibDir" -> result.success(applicationInfo.nativeLibraryDir)
                    // Copy the bundled ament index (assets/ros_ament/share) to a
                    // writable dir once, and return its path for AMENT_PREFIX_PATH.
                    "amentPrefixPath" -> result.success(prepareAment())
                    else -> result.notImplemented()
                }
            }
    }

    /** Copies assets/ros_ament/share -> filesDir/ros_ament (once) and returns it. */
    private fun prepareAment(): String {
        val dest = File(filesDir, "ros_ament")
        val marker = File(dest, ".ready")
        if (!marker.exists()) {
            copyAsset("ros_ament", filesDir)
            marker.parentFile?.mkdirs()
            marker.writeText("1")
        }
        return dest.absolutePath
    }

    private fun copyAsset(path: String, outParent: File) {
        val entries = assets.list(path) ?: emptyArray()
        if (entries.isEmpty()) {
            // It's a file.
            assets.open(path).use { input ->
                File(outParent, path).apply { parentFile?.mkdirs() }
                    .outputStream().use { input.copyTo(it) }
            }
        } else {
            File(outParent, path).mkdirs()
            for (e in entries) copyAsset("$path/$e", outParent)
        }
    }

    override fun onDestroy() {
        multicastLock?.release()
        super.onDestroy()
    }
}
