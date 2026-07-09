import Flutter
import UIKit

@main
@objc class AppDelegate: FlutterAppDelegate, FlutterImplicitEngineDelegate {
  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }

  func didInitializeImplicitFlutterEngine(_ engineBridge: FlutterImplicitEngineBridge) {
    GeneratedPluginRegistrant.register(with: engineBridge.pluginRegistry)
    if let messenger = engineBridge.pluginRegistry.registrar(forPlugin: "rcldart")?.messenger() {
      RcldartApple.register(messenger)
    }
  }
}

// Bridges the bundled ROS ament index path to Dart. The rcldart podspec ships
// the ament `share/` tree as the `rcldart_ros` resource bundle; AppleRosBootstrap
// sets AMENT_PREFIX_PATH to it before RclDart().init().
enum RcldartApple {
  static func register(_ messenger: FlutterBinaryMessenger) {
    let channel = FlutterMethodChannel(name: "rcldart/apple", binaryMessenger: messenger)
    channel.setMethodCallHandler { call, result in
      switch call.method {
      case "amentPrefixPath":
        if let b = Bundle.main.url(forResource: "rcldart_ros", withExtension: "bundle"),
           let rp = Bundle(url: b)?.resourcePath {
          result(rp + "/share")
        } else {
          result(nil)
        }
      default:
        result(FlutterMethodNotImplemented)
      }
    }
  }
}
