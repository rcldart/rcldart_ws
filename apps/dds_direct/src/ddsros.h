// ddsros.h — minimal C FFI surface over CycloneDDS for a ROS 2-compatible,
// raw-CDR pub/sub. No rosidl, no ROS: message bytes are produced/consumed by
// pure-Dart `ros2_cdr`. The shim only moves opaque CDR blobs on ROS-named
// topics with a ROS-compatible sertype so real ROS 2 nodes match.
#ifndef DDSROS_H
#define DDSROS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handles (dds_entity_t under the hood).
typedef int32_t ddsros_entity;

// Create a DDS participant on [domain] (ROS_DOMAIN_ID). Returns <0 on error.
ddsros_entity ddsros_participant(uint32_t domain);

// Create a writer on ROS topic [topic] (e.g. "chatter") for ROS type
// [type_name] (e.g. "std_msgs::msg::dds_::String_"). Reliable + volatile by
// default (ROS default for most topics). Returns <0 on error.
ddsros_entity ddsros_writer(ddsros_entity participant,
                            const char *topic, const char *type_name);

// Publish one message: [cdr]/[len] is the full CDR payload INCLUDING the 4-byte
// encapsulation header (exactly what ros2_cdr's CdrWriter.toBytes() returns).
int ddsros_write(ddsros_entity writer, const uint8_t *cdr, size_t len);

// Create a reader on ROS topic [topic] for ROS type [type_name].
ddsros_entity ddsros_reader(ddsros_entity participant,
                            const char *topic, const char *type_name);

// Take one sample if available. Copies the CDR payload (with encapsulation
// header) into [buf] (capacity [cap]); returns the byte count, 0 if no data,
// <0 on error. Poll this, or use ddsros_waitset for blocking.
int ddsros_take(ddsros_entity reader, uint8_t *buf, size_t cap);

// Block up to [timeout_ms] until [reader] has data (or timeout). >0 = ready.
int ddsros_wait(ddsros_entity reader, int timeout_ms);

// --- graph discovery over DDS (no bridge, no WS) ----------------------------
// Create a reader on the DCPSPublication builtin topic — it discovers every
// publisher on the graph (topic name + type name) purely over DDS. Returns <0
// on error. Create once, poll with ddsros_discover.
ddsros_entity ddsros_disco_reader(ddsros_entity participant);

// Read the currently-known publications into [buf] as newline-separated
// "topic<TAB>type" lines (DDS names, e.g. "rt/tb1/scan\tsensor_msgs::msg::dds_::LaserScan_").
// Returns bytes written, <0 on error. Poll this to populate a live topic list.
int ddsros_discover(ddsros_entity disco_reader, char *buf, size_t cap);

// Tear down a participant (and its children).
void ddsros_delete(ddsros_entity entity);

#ifdef __cplusplus
}
#endif
#endif // DDSROS_H
