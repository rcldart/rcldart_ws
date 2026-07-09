import Cocoa
import FlutterMacOS

class MainFlutterWindow: NSWindow {
  override func awakeFromNib() {
    let flutterViewController = FlutterViewController()
    let windowFrame = self.frame
    self.contentViewController = flutterViewController
    self.setFrame(windowFrame, display: true)

    RegisterGeneratedPlugins(registry: flutterViewController)

    // Bridge the bundled ROS ament index path to Dart (see AppleRosBootstrap).
    let channel = FlutterMethodChannel(
      name: "rcldart/apple",
      binaryMessenger: flutterViewController.engine.binaryMessenger)
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

    super.awakeFromNib()
  }
}
