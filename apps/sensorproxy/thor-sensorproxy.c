/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <gio/gio.h>
#include <glib-unix.h>
#include <gmodule.h>
#include <math.h>
#include <libssc-sensor-accelerometer.h>

#define BUS "net.hadess.SensorProxy"
#define PATH "/net/hadess/SensorProxy"
#define PRIVATE_SSC "/usr/lib/thorch-motion/ssc/libssc.so.2"

/* 3.9 orientation-compatible subset; Unit is a documented extension. */
static const char xml[] =
 "<node><interface name='" BUS "'>"
 "<method name='ClaimAccelerometer'/><method name='ReleaseAccelerometer'/>"
 "<property name='HasAccelerometer' type='b' access='read'/>"
 "<property name='AccelerometerOrientation' type='s' access='read'/>"
 "<property name='AccelerometerUnit' type='s' access='read'/>"
 "<property name='AccelerometerTilt' type='s' access='read'/>"
 "<property name='HasAmbientLight' type='b' access='read'/>"
 "<property name='HasProximity' type='b' access='read'/>"
 "</interface></node>";

typedef struct {
 void (*create)(GCancellable *, GAsyncReadyCallback, gpointer);
 SSCSensorAccelerometer *(*created)(GAsyncResult *, GError **);
 void (*rate)(SSCSensor *, gfloat);
 void (*open)(SSCSensorAccelerometer *, GCancellable *, GAsyncReadyCallback, gpointer);
 gboolean (*opened)(SSCSensorAccelerometer *, GAsyncResult *, GError **);
 void (*close)(SSCSensorAccelerometer *, GCancellable *, GAsyncReadyCallback, gpointer);
 gboolean (*closed)(SSCSensorAccelerometer *, GAsyncResult *, GError **);
} API;
static API api;
static GMainLoop *loop;
static GDBusConnection *bus;
static GHashTable *claims;
static SSCSensorAccelerometer *sensor;
static gboolean busy, streaming, available, stopping, owns_name;
static gint exit_code;
static gint axis[3], sign_axis[3], debounce_ms = 450, process_ms = 100;
static gdouble sample_hz = 100;
static gint64 last_sample, last_processed, candidate_since;
static const char *orientation = "undefined", *candidate = "undefined";

static void changed(const char *name, GVariant *value)
{
 if (!bus) return;
 GVariantBuilder props, invalid;
 g_variant_builder_init(&props, G_VARIANT_TYPE("a{sv}"));
 g_variant_builder_add(&props, "{sv}", name, value);
 g_variant_builder_init(&invalid, G_VARIANT_TYPE("as"));
 g_dbus_connection_emit_signal(bus, NULL, PATH,
  "org.freedesktop.DBus.Properties", "PropertiesChanged",
  g_variant_new("(sa{sv}as)", BUS, &props, &invalid), NULL);
}
static void set_available(gboolean value)
{
 if (available != value) {
  available = value;
  changed("HasAccelerometer", g_variant_new_boolean(value));
 }
}
static void set_orientation(const char *value)
{
 if (g_strcmp0(orientation, value)) {
  orientation = value;
  changed("AccelerometerOrientation", g_variant_new_string(value));
 }
}
/* Actual upstream signal: measurement(self, gfloat x, gfloat y, gfloat z, data).
 * libssc has already applied its corrected placement matrix. Axis mapping here
 * maps that result into the physically validated display frame, not raw SSC. */
static void measurement(SSCSensorAccelerometer *self, gfloat x, gfloat y,
                        gfloat z, gpointer data)
{
 (void)self; (void)data;
 if (!streaming || !g_hash_table_size(claims)) return;
 double raw[] = {x, y, z};
 for (int i = 0; i < 3; i++) if (!isfinite(raw[i])) return;
 gint64 now = g_get_monotonic_time();
 last_sample = now;
 set_available(TRUE);
 if (now - last_processed < (gint64)process_ms * 1000) return;
 last_processed = now;
 double v[3];
 for (int i = 0; i < 3; i++) v[i] = sign_axis[i] * raw[axis[i]];
 double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
 /* Reject motion/free fall and near-flat positions; preserve last stable
  * orientation. Dominant-axis margin is angular hysteresis (~10 degrees). */
 if (norm < 7.0 || norm > 12.5 || MAX(fabs(v[0]), fabs(v[1])) < 0.65*norm ||
     fabs(fabs(v[0]) - fabs(v[1])) < 0.18*norm) {
  candidate = "undefined"; candidate_since = 0; return;
 }
 const char *next = fabs(v[1]) >= fabs(v[0])
  ? (v[1] < 0 ? "normal" : "bottom-up")
  : (v[0] > 0 ? "left-up" : "right-up");
 if (g_strcmp0(next, candidate)) {
  candidate = next; candidate_since = now;
 } else if (now - candidate_since >= (gint64)debounce_ms*1000) {
  set_orientation(next);
 }
}
static void reconcile(void);
static void closed(GObject *source, GAsyncResult *result, gpointer data)
{
 (void)data;
 GError *error = NULL;
 if (!api.closed((SSCSensorAccelerometer *)source, result, &error)) {
  g_warning("SSC close: %s", error->message); g_clear_error(&error);
 }
 busy = FALSE; streaming = FALSE;
 g_clear_object(&sensor);
 set_available(FALSE); set_orientation("undefined");
 if (stopping) g_main_loop_quit(loop); else reconcile();
}
static void opened(GObject *source, GAsyncResult *result, gpointer data)
{
 (void)data;
 GError *error = NULL;
 streaming = api.opened((SSCSensorAccelerometer *)source, result, &error);
 busy = FALSE;
 if (!streaming) {
  g_warning("SSC open: %s", error->message); g_clear_error(&error);
  g_clear_object(&sensor); set_available(FALSE);
  if (stopping) g_main_loop_quit(loop);
  return; /* Retry by maintenance timer, not recursive spinning. */
 }
 last_sample = g_get_monotonic_time();
 reconcile();
}
static void created(GObject *source, GAsyncResult *result, gpointer data)
{
 (void)source; (void)data;
 GError *error = NULL;
 sensor = api.created(result, &error);
 busy = FALSE;
 if (!sensor) {
  g_warning("SSC discovery: %s", error->message); g_clear_error(&error);
  if (stopping) g_main_loop_quit(loop);
  return;
 }
 set_available(TRUE);
 g_signal_connect(sensor, "measurement", G_CALLBACK(measurement), NULL);
 reconcile();
}
static void reconcile(void)
{
 if (busy) return;
 if (!owns_name && !stopping) return;
 gboolean want = !stopping && g_hash_table_size(claims) > 0;
 if (streaming && !want) {
  busy = TRUE;
  api.close(sensor, NULL, closed, NULL);
 } else if (stopping) {
  g_main_loop_quit(loop);
 } else if (!sensor) {
  busy = TRUE; api.create(NULL, created, NULL);
 } else if (want && !streaming) {
  candidate = "undefined"; candidate_since = last_processed = 0;
  api.rate((SSCSensor *)sensor, (gfloat)sample_hz);
  busy = TRUE; api.open(sensor, NULL, opened, NULL);
 }
}
static gboolean maintenance(gpointer data)
{
 (void)data;
 if (streaming && g_get_monotonic_time() - last_sample > 5000000) {
  set_available(FALSE); set_orientation("undefined");
  /* Close a stalled stream, then rediscover while retaining live claims. */
  if (!busy) { busy = TRUE; api.close(sensor, NULL, closed, NULL); }
 } else reconcile();
 return G_SOURCE_CONTINUE;
}
static void unwatch(gpointer value)
{
 g_bus_unwatch_name(GPOINTER_TO_UINT(value));
}
static void vanished(GDBusConnection *connection, const char *name, gpointer data)
{
 (void)connection; (void)data;
 if (g_hash_table_remove(claims, name)) reconcile();
}
static void method(GDBusConnection *connection, const char *sender,
 const char *path, const char *interface, const char *name,
 GVariant *parameters, GDBusMethodInvocation *invocation, gpointer data)
{
 (void)connection; (void)path; (void)interface; (void)parameters; (void)data;
 if (!g_strcmp0(name, "ClaimAccelerometer")) {
  if (!g_hash_table_contains(claims, sender)) {
   guint watch = g_bus_watch_name_on_connection(connection, sender,
    G_BUS_NAME_WATCHER_FLAGS_NONE, NULL, vanished, NULL, NULL);
   g_hash_table_insert(claims, g_strdup(sender), GUINT_TO_POINTER(watch));
  }
 }
 else if (!g_strcmp0(name, "ReleaseAccelerometer"))
  g_hash_table_remove(claims, sender);
 else {
  g_dbus_method_invocation_return_error(invocation, G_DBUS_ERROR,
   G_DBUS_ERROR_UNKNOWN_METHOD, "Unsupported sensor method"); return;
 }
 g_dbus_method_invocation_return_value(invocation, NULL);
 reconcile();
}
static GVariant *property(GDBusConnection *connection, const char *sender,
 const char *path, const char *interface, const char *name, GError **error,
 gpointer data)
{
 (void)connection; (void)sender; (void)path; (void)interface; (void)data;
 if (!g_strcmp0(name, "HasAccelerometer")) return g_variant_new_boolean(available);
 if (!g_strcmp0(name, "AccelerometerOrientation")) return g_variant_new_string(orientation);
 if (!g_strcmp0(name, "AccelerometerUnit")) return g_variant_new_string("m/s^2");
 if (!g_strcmp0(name, "AccelerometerTilt")) return g_variant_new_string("undefined");
 if (!g_strcmp0(name, "HasAmbientLight") || !g_strcmp0(name, "HasProximity"))
  return g_variant_new_boolean(FALSE);
 g_set_error(error, G_DBUS_ERROR, G_DBUS_ERROR_UNKNOWN_PROPERTY, "Unknown property");
 return NULL;
}
static const GDBusInterfaceVTable vtable = {method, property, NULL, {0}};
static gboolean stop(gpointer data)
{
 (void)data; stopping = TRUE; reconcile(); return G_SOURCE_REMOVE;
}
static void name_lost(GDBusConnection *connection, const char *name, gpointer data)
{
 (void)connection; (void)name; (void)data;
 g_warning("Cannot own " BUS "; system proxy may already own it");
 owns_name = FALSE; exit_code = 1; stop(NULL);
}
static void name_acquired(GDBusConnection *connection, const char *name, gpointer data)
{
 (void)connection; (void)name; (void)data;
 owns_name = TRUE; reconcile();
}
static void bus_acquired(GDBusConnection *connection, const char *name, gpointer data)
{
 (void)name; (void)data;
 GError *error = NULL;
 bus = g_object_ref(connection);
 GDBusNodeInfo *info = g_dbus_node_info_new_for_xml(xml, &error);
 if (!info || !g_dbus_connection_register_object(bus, PATH, info->interfaces[0],
                                               &vtable, NULL, NULL, &error)) {
  g_printerr("D-Bus registration: %s\n", error->message);
  g_clear_error(&error); exit_code = 1; stop(NULL);
 }
 if (info) g_dbus_node_info_unref(info);
}
int main(int argc, char **argv)
{
 char *frame = NULL;
 gboolean session_bus = FALSE;
 GOptionEntry options[] = {
  {"session-bus", 0, 0, G_OPTION_ARG_NONE, &session_bus, "Isolated diagnostic bus; does not supply KWin system-bus orientation", NULL},
  {"frame", 0, 0, G_OPTION_ARG_STRING, &frame, "Required signed axis permutation: display X,Y,Z (e.g. +x,+y,+z; NOT a Thor default)", "AXES"},
  {"sample-hz", 0, 0, G_OPTION_ARG_DOUBLE, &sample_hz, "SSC rate; 100 Hz known working, lower rates need validation", "HZ"},
  {"process-ms", 0, 0, G_OPTION_ARG_INT, &process_ms, "Orientation evaluation interval", "MS"},
  {"debounce-ms", 0, 0, G_OPTION_ARG_INT, &debounce_ms, "Stable candidate duration", "MS"},
  {NULL, 0, 0, 0, NULL, NULL, NULL}
 };
 GError *error = NULL;
 GOptionContext *context = g_option_context_new("- SSC SensorProxy compatibility bridge");
 g_option_context_add_main_entries(context, options, NULL);
 if (!g_option_context_parse(context, &argc, &argv, &error)) {
  g_printerr("%s\n", error->message); return 2;
 }
 if (!frame || !isfinite(sample_hz) || sample_hz < 1 || sample_hz > 200 ||
     process_ms < 20 || process_ms > 2000 || debounce_ms < 100 || debounce_ms > 5000) {
  g_printerr("Explicit physically validated --frame and valid rate/timing required; use --help\n"); return 2;
 }
 char **parts = g_strsplit(frame, ",", -1);
 gboolean used[3] = {FALSE, FALSE, FALSE}, valid = g_strv_length(parts) == 3;
 for (int i = 0; valid && i < 3; i++) {
  valid = strlen(parts[i]) == 2 && (parts[i][0] == '+' || parts[i][0] == '-') &&
   parts[i][1] >= 'x' && parts[i][1] <= 'z';
  if (valid) {
   axis[i] = parts[i][1] - 'x'; sign_axis[i] = parts[i][0] == '+' ? 1 : -1;
   valid = !used[axis[i]]; used[axis[i]] = TRUE;
  }
 }
 g_strfreev(parts); g_free(frame); g_option_context_free(context);
 if (!valid) { g_printerr("--frame must be a signed permutation of x,y,z\n"); return 2; }
 GModule *module = g_module_open(PRIVATE_SSC, G_MODULE_BIND_LOCAL);
 if (!module) { g_printerr("Private SSC only: %s\n", g_module_error()); return 1; }
#define LOAD(field, symbol) do { if (!g_module_symbol(module, symbol, (gpointer *)&api.field)) { \
 g_printerr("Private SSC missing %s: %s\n", symbol, g_module_error()); return 1; } } while (0)
 LOAD(create, "ssc_sensor_accelerometer_new");
 LOAD(created, "ssc_sensor_accelerometer_new_finish");
 LOAD(rate, "ssc_sensor_set_sample_rate");
 LOAD(open, "ssc_sensor_accelerometer_open");
 LOAD(opened, "ssc_sensor_accelerometer_open_finish");
 LOAD(close, "ssc_sensor_accelerometer_close");
 LOAD(closed, "ssc_sensor_accelerometer_close_finish");
#undef LOAD
 g_module_make_resident(module); /* GObject types outlive module handles. */
 claims = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, unwatch);
 loop = g_main_loop_new(NULL, FALSE);
 g_unix_signal_add(SIGTERM, stop, NULL); g_unix_signal_add(SIGINT, stop, NULL);
 g_timeout_add_seconds(2, maintenance, NULL);
 guint owner = g_bus_own_name(session_bus ? G_BUS_TYPE_SESSION : G_BUS_TYPE_SYSTEM, BUS,
  G_BUS_NAME_OWNER_FLAGS_DO_NOT_QUEUE, bus_acquired, name_acquired, name_lost, NULL, NULL);
 g_main_loop_run(loop);
 g_bus_unown_name(owner);
 g_clear_object(&sensor); g_clear_object(&bus);
 g_hash_table_unref(claims); g_main_loop_unref(loop);
 return exit_code;
}
