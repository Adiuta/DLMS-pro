#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "dlms_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the DLMS WebSocket endpoint at /ws.
 * The HTTP server itself is owned by wifi_web.c.
 */
esp_err_t dlms_ws_init(void);
esp_err_t dlms_ws_register_uri(httpd_handle_t server);
esp_err_t dlms_ws_attach_server(httpd_handle_t server);

/* Broadcast the complete current state to every connected Studio client. */
esp_err_t dlms_ws_broadcast_state(void);

/* Broadcast six realtime software VU values:
 * IN1, IN2, OUT1, OUT2, OUT3, OUT4.
 */
esp_err_t dlms_ws_broadcast_meter(const float vu_db[6]);

/* Broadcast a single parameter/state change. */
esp_err_t dlms_ws_broadcast_change(const char *path, float value);

/* Broadcast meter data with RTA spectrum. */
esp_err_t dlms_ws_broadcast_meter_rta(const float vu_db[6], const float *rta_db, size_t rta_count);

#ifdef __cplusplus
}
#endif

