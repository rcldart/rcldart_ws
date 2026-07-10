// Forwarder so CocoaPods (which can't use relative source paths) compiles the
// shared C shim under ../src. The actual build of CycloneDDS + libdds_direct is
// driven by the podspec's CMake prepare_command; this file just keeps the pod
// non-empty and its symbols available.
#include "../../src/ddsros.h"
