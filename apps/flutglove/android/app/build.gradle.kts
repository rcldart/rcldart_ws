plugins {
    id("com.android.application")
    // The Flutter Gradle Plugin must be applied after the Android and Kotlin Gradle plugins.
    id("dev.flutter.flutter-gradle-plugin")
}

android {
    namespace = "com.example.flutglove"
    compileSdk = flutter.compileSdkVersion
    ndkVersion = flutter.ndkVersion

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    defaultConfig {
        // TODO: Specify your own unique Application ID (https://developer.android.com/studio/build/application-id.html).
        applicationId = "com.example.flutglove"
        // You can update the following values to match your application needs.
        // For more information, see: https://flutter.dev/to/review-gradle-config.
        // ROS 2 runtime requires modern bionic (dlopen/RTLD_GLOBAL/getifaddrs).
        minSdk = maxOf(24, flutter.minSdkVersion)
        targetSdk = flutter.targetSdkVersion
        versionCode = flutter.versionCode
        versionName = flutter.versionName

        // Only bundle ABIs the ROS 2 closure was cross-compiled for (see
        // rcldart/android/src/main/jniLibs). arm64-v8a = real devices.
        ndk {
            // arm64-v8a = real devices; x86_64 = emulator.
            abiFilters += listOf("arm64-v8a", "x86_64")
        }
    }

    // Keep the bundled ROS .so uncompressed & extractable so rcl can dlopen
    // typesupport/rmw plugins by name at runtime.
    packaging {
        jniLibs.useLegacyPackaging = true
    }

    buildTypes {
        release {
            // TODO: Add your own signing config for the release build.
            // Signing with the debug keys for now, so `flutter run --release` works.
            signingConfig = signingConfigs.getByName("debug")
        }
    }
}

kotlin {
    compilerOptions {
        jvmTarget = org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17
    }
}

flutter {
    source = "../.."
}
