// ddsros.c — CycloneDDS shim carrying raw CDR on ROS 2-named topics.
//
// ROS 2 does not put generated C types on the wire — rmw_cyclonedds registers
// ONE generic sertype whose samples are just a CDR blob + the ROS type name.
// We do the same, so real ROS 2 nodes discover and exchange data with us while
// all (de)serialization happens in Dart (ros2_cdr).
//
// The sertype/serdata ops below are the full set CycloneDDS 0.10 requires; the
// app-facing "sample" is a {cdr, len} view (struct ros_sample). What is NOT yet
// done: advertising XTypes TypeInformation / the ROS 2 type hash (RIHS01) —
// type_id/type_map/type_info are left NULL. Basic same-domain pub/sub works;
// strict Humble/Jazzy type-hash matching is the remaining piece.
#include "ddsros.h"

#include "dds/dds.h"
#include "dds/ddsi/ddsi_sertype.h"
#include "dds/ddsi/ddsi_serdata.h"
#include "dds/ddsi/ddsi_keyhash.h"
#include "dds/ddsi/q_radmin.h" // fragment chain (struct nn_rdata) for from_ser

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// App-facing sample: a view onto a CDR blob (incl. encapsulation header).
struct ros_sample { const uint8_t *cdr; size_t len; };

// ---- serdata: one CDR sample ------------------------------------------------
struct ros_serdata {
  struct ddsi_serdata c;
  uint32_t size;   // CDR payload size (incl. 4-byte encapsulation header)
  void *data;      // owned CDR bytes
};

struct ros_sertype {
  struct ddsi_sertype c;
  char *type_name;
};

static struct ros_serdata *ros_serdata_new(const struct ddsi_sertype *tp,
                                           enum ddsi_serdata_kind kind,
                                           const void *cdr, uint32_t size) {
  struct ros_serdata *d = malloc(sizeof(*d));
  ddsi_serdata_init(&d->c, tp, kind);
  d->size = size;
  d->data = malloc(size ? size : 1);
  if (cdr && size) memcpy(d->data, cdr, size);
  return d;
}

static void ros_free(struct ddsi_serdata *dcmn) {
  struct ros_serdata *d = (struct ros_serdata *)dcmn;
  free(d->data);
  free(d);
}

static uint32_t ros_get_size(const struct ddsi_serdata *dcmn) {
  return ((const struct ros_serdata *)dcmn)->size;
}

static bool ros_eqkey(const struct ddsi_serdata *a, const struct ddsi_serdata *b) {
  (void)a; (void)b; return true; // keyless
}

static struct ddsi_serdata *ros_from_ser_iov(
    const struct ddsi_sertype *tp, enum ddsi_serdata_kind kind,
    ddsrt_msg_iovlen_t niov, const ddsrt_iovec_t *iov, size_t size) {
  struct ros_serdata *d = ros_serdata_new(tp, kind, NULL, (uint32_t)size);
  size_t off = 0;
  for (ddsrt_msg_iovlen_t i = 0; i < niov; i++) {
    memcpy((char *)d->data + off, iov[i].iov_base, iov[i].iov_len);
    off += iov[i].iov_len;
  }
  return &d->c;
}

static struct ddsi_serdata *ros_from_ser(
    const struct ddsi_sertype *tp, enum ddsi_serdata_kind kind,
    const struct nn_rdata *fragchain, size_t size) {
  struct ros_serdata *d = ros_serdata_new(tp, kind, NULL, (uint32_t)size);
  uint32_t off = 0;
  while (fragchain) {
    if (fragchain->maxp1 > off) {
      const unsigned char *payload =
          NN_RMSG_PAYLOADOFF(fragchain->rmsg, NN_RDATA_PAYLOAD_OFF(fragchain));
      uint32_t n = fragchain->maxp1 - off;
      memcpy((char *)d->data + off, payload + (off - fragchain->min), n);
      off = fragchain->maxp1;
    }
    fragchain = fragchain->nextfrag;
  }
  return &d->c;
}

static struct ddsi_serdata *ros_from_keyhash(
    const struct ddsi_sertype *tp, const struct ddsi_keyhash *kh) {
  (void)kh; return &ros_serdata_new(tp, SDK_KEY, NULL, 0)->c;
}

static struct ddsi_serdata *ros_from_sample(
    const struct ddsi_sertype *tp, enum ddsi_serdata_kind kind, const void *sample) {
  const struct ros_sample *s = sample;
  return &ros_serdata_new(tp, kind, s->cdr, (uint32_t)s->len)->c;
}

static void ros_to_ser(const struct ddsi_serdata *dcmn, size_t off, size_t sz, void *buf) {
  const struct ros_serdata *d = (const struct ros_serdata *)dcmn;
  memcpy(buf, (const char *)d->data + off, sz);
}

static struct ddsi_serdata *ros_to_ser_ref(
    const struct ddsi_serdata *dcmn, size_t off, size_t sz, ddsrt_iovec_t *ref) {
  struct ros_serdata *d = (struct ros_serdata *)dcmn;
  ref->iov_base = (char *)d->data + off;
  ref->iov_len = (ddsrt_iov_len_t)sz;
  return ddsi_serdata_ref(dcmn);
}

static void ros_to_ser_unref(struct ddsi_serdata *dcmn, const ddsrt_iovec_t *ref) {
  (void)ref; ddsi_serdata_unref(dcmn);
}

static bool ros_to_sample(const struct ddsi_serdata *dcmn, void *sample,
                          void **bufptr, void *buflim) {
  (void)bufptr; (void)buflim;
  const struct ros_serdata *d = (const struct ros_serdata *)dcmn;
  struct ros_sample *s = sample;
  s->cdr = d->data;
  s->len = d->size;
  return true;
}

static struct ddsi_serdata *ros_to_untyped(const struct ddsi_serdata *dcmn) {
  const struct ros_serdata *d = (const struct ros_serdata *)dcmn;
  struct ros_serdata *u = ros_serdata_new(d->c.type, SDK_KEY, d->data, d->size);
  return &u->c;
}

static bool ros_untyped_to_sample(
    const struct ddsi_sertype *tp, const struct ddsi_serdata *dcmn,
    void *sample, void **bufptr, void *buflim) {
  (void)tp; return ros_to_sample(dcmn, sample, bufptr, buflim);
}

static size_t ros_print(const struct ddsi_sertype *tp, const struct ddsi_serdata *d,
                        char *buf, size_t size) {
  (void)tp; (void)d;
  return (size_t)snprintf(buf, size, "ros_cdr");
}

static void ros_get_keyhash(const struct ddsi_serdata *d, struct ddsi_keyhash *buf,
                            bool force_md5) {
  (void)d; (void)force_md5;
  memset(buf, 0, sizeof(*buf));
}

static const struct ddsi_serdata_ops ros_serdata_ops = {
  .eqkey = ros_eqkey,
  .get_size = ros_get_size,
  .from_ser = ros_from_ser,
  .from_ser_iov = ros_from_ser_iov,
  .from_keyhash = ros_from_keyhash,
  .from_sample = ros_from_sample,
  .to_ser = ros_to_ser,
  .to_ser_ref = ros_to_ser_ref,
  .to_ser_unref = ros_to_ser_unref,
  .to_sample = ros_to_sample,
  .to_untyped = ros_to_untyped,
  .untyped_to_sample = ros_untyped_to_sample,
  .free = ros_free,
  .print = ros_print,
  .get_keyhash = ros_get_keyhash,
};

// ---- sertype ----------------------------------------------------------------
static void ros_sertype_free(struct ddsi_sertype *tpcmn) {
  struct ros_sertype *tp = (struct ros_sertype *)tpcmn;
  ddsi_sertype_fini(&tp->c);
  free(tp->type_name);
  free(tp);
}

static bool ros_sertype_equal(const struct ddsi_sertype *a, const struct ddsi_sertype *b) {
  return strcmp(((const struct ros_sertype *)a)->type_name,
                ((const struct ros_sertype *)b)->type_name) == 0;
}

static uint32_t ros_sertype_hash(const struct ddsi_sertype *tp) {
  const char *s = ((const struct ros_sertype *)tp)->type_name;
  uint32_t h = 2166136261u;
  for (; *s; s++) { h ^= (uint8_t)*s; h *= 16777619u; }
  return h;
}

static void ros_zero_samples(const struct ddsi_sertype *d, void *samples, size_t count) {
  (void)d;
  memset(samples, 0, count * sizeof(struct ros_sample));
}

static void ros_realloc_samples(void **ptrs, const struct ddsi_sertype *d,
                                void *old, size_t oldcount, size_t count) {
  (void)d; (void)oldcount;
  char *base = realloc(old, count * sizeof(struct ros_sample));
  for (size_t i = 0; i < count; i++) ptrs[i] = base + i * sizeof(struct ros_sample);
}

static void ros_free_samples(const struct ddsi_sertype *d, void **ptrs, size_t count,
                             dds_free_op_t op) {
  // Our sample is a {cdr,len} VIEW that owns nothing — the CDR bytes belong to
  // the serdata (freed via ros_free on unref). So there is nothing to free here;
  // in particular we must NOT free the caller's sample array (it may be stack).
  (void)d; (void)ptrs; (void)count; (void)op;
}

static size_t ros_get_serialized_size(const struct ddsi_sertype *d, const void *sample) {
  (void)d; return ((const struct ros_sample *)sample)->len;
}

static bool ros_serialize_into(const struct ddsi_sertype *d, const void *sample,
                               void *dst, size_t dst_size) {
  (void)d;
  const struct ros_sample *s = sample;
  if (dst_size < s->len) return false;
  memcpy(dst, s->cdr, s->len);
  return true;
}

static const struct ddsi_sertype_ops ros_sertype_ops = {
  .version = ddsi_sertype_v0,
  .arg = NULL,
  .free = ros_sertype_free,
  .zero_samples = ros_zero_samples,
  .realloc_samples = ros_realloc_samples,
  .free_samples = ros_free_samples,
  .equal = ros_sertype_equal,
  .hash = ros_sertype_hash,
  .type_id = NULL,   // XTypes TypeInformation / ROS type hash: TODO
  .type_map = NULL,
  .type_info = NULL,
  .derive_sertype = NULL,
  .get_serialized_size = ros_get_serialized_size,
  .serialize_into = ros_serialize_into,
};

static struct ddsi_sertype *make_sertype(const char *type_name) {
  struct ros_sertype *tp = malloc(sizeof(*tp));
  tp->type_name = strdup(type_name);
  ddsi_sertype_init(&tp->c, type_name, &ros_sertype_ops, &ros_serdata_ops,
                    /*topickind_no_key=*/true);
  return &tp->c;
}

// ROS 2 maps a topic "chatter" to DDS topic "rt/chatter".
static char *ros_topic_name(const char *topic) {
  size_t n = strlen(topic) + 4;
  char *s = malloc(n);
  snprintf(s, n, "rt/%s", topic);
  return s;
}

// ---- public API -------------------------------------------------------------
ddsros_entity ddsros_participant(uint32_t domain) {
  return dds_create_participant(domain, NULL, NULL);
}

static ddsros_entity make_topic(ddsros_entity pp, const char *topic,
                                const char *type_name) {
  struct ddsi_sertype *st = make_sertype(type_name);
  char *tn = ros_topic_name(topic);
  dds_entity_t t = dds_create_topic_sertype(pp, tn, &st, NULL, NULL, NULL);
  free(tn);
  return t;
}

ddsros_entity ddsros_writer(ddsros_entity pp, const char *topic, const char *type_name) {
  dds_entity_t t = make_topic(pp, topic, type_name);
  if (t < 0) return t;
  dds_qos_t *q = dds_create_qos();
  dds_qset_reliability(q, DDS_RELIABILITY_RELIABLE, DDS_MSECS(100));
  dds_qset_durability(q, DDS_DURABILITY_VOLATILE);
  dds_entity_t w = dds_create_writer(pp, t, q, NULL);
  dds_delete_qos(q);
  return w;
}

int ddsros_write(ddsros_entity writer, const uint8_t *cdr, size_t len) {
  struct ros_sample s = { cdr, len };
  return dds_write(writer, &s);
}

ddsros_entity ddsros_reader(ddsros_entity pp, const char *topic, const char *type_name) {
  dds_entity_t t = make_topic(pp, topic, type_name);
  if (t < 0) return t;
  dds_qos_t *q = dds_create_qos();
  dds_qset_reliability(q, DDS_RELIABILITY_RELIABLE, DDS_MSECS(100));
  dds_qset_durability(q, DDS_DURABILITY_VOLATILE);
  dds_entity_t r = dds_create_reader(pp, t, q, NULL);
  dds_delete_qos(q);
  return r;
}

int ddsros_take(ddsros_entity reader, uint8_t *buf, size_t cap) {
  // Take the SERDATA (not a converted sample): the sample-copy path would
  // release the serdata before we copy, leaving s.cdr dangling. With the cdr
  // path we hold the ref, copy the raw CDR, then unref — no use-after-free.
  struct ddsi_serdata *sd = NULL;
  dds_sample_info_t info;
  int n = dds_takecdr(reader, &sd, 1, &info, DDS_ANY_STATE);
  if (n <= 0) return n;
  int copied = 0;
  if (info.valid_data && sd != NULL) {
    struct ros_serdata *d = (struct ros_serdata *)sd;
    copied = (int)(d->size > cap ? cap : d->size);
    memcpy(buf, d->data, copied);
  }
  if (sd != NULL) ddsi_serdata_unref(sd);
  return copied;
}

int ddsros_wait(ddsros_entity reader, int timeout_ms) {
  dds_entity_t ws = dds_create_waitset(DDS_CYCLONEDDS_HANDLE);
  dds_entity_t rc = dds_create_readcondition(reader, DDS_ANY_STATE);
  dds_waitset_attach(ws, rc, reader);
  dds_attach_t xs[1];
  int n = dds_waitset_wait(ws, xs, 1, DDS_MSECS(timeout_ms));
  dds_delete(ws);
  return n;
}

// --- graph discovery over DDS ------------------------------------------------
ddsros_entity ddsros_disco_reader(ddsros_entity pp) {
  return dds_create_reader(pp, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, NULL, NULL);
}

int ddsros_discover(ddsros_entity rd, char *buf, size_t cap) {
  enum { MAXN = 512 };
  void *samples[MAXN];
  dds_sample_info_t si[MAXN];
  for (int i = 0; i < MAXN; i++) samples[i] = NULL;
  // dds_read (not take) so the built-in history stays for the next poll.
  int n = dds_read(rd, samples, si, MAXN, MAXN);
  if (n < 0) return n;
  size_t off = 0;
  for (int i = 0; i < n && off + 1 < cap; i++) {
    if (!si[i].valid_data) continue;
    const dds_builtintopic_endpoint_t *e = samples[i];
    if (e->topic_name == NULL) continue;
    int w = snprintf(buf + off, cap - off, "%s\t%s\n",
                     e->topic_name, e->type_name ? e->type_name : "");
    if (w > 0 && (size_t)w < cap - off) off += (size_t)w;
  }
  dds_return_loan(rd, samples, n);
  return (int)off;
}

void ddsros_delete(ddsros_entity entity) { dds_delete(entity); }
