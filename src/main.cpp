
/*

MQTT

Connected: 0 - 1
IP: <str>
RSS: <str>

Mode: 0 off 1 auto 2 cool 3 dry 4 fun 5 heat
Temp: 16 - 30
Fun: 0 auto 1 - 5 on
Swing: 0 - 1
Light: 0 - 1

X-HM-Mode: 0 off 1 heat 2 cool

*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "creds.h"

#define HEARTBIT_INTERVAL   5 * 60 * 1000
#define MQTT_RECON_INTERVAL 5 * 1000
#define AIR_MSG_INTERVAL    1 * 1000
#define BLINK_INTERVAL      3 * 1000

#define LED                 2

WiFiClient espClient;
PubSubClient mqtt(espClient);

const char* mqtt_server       = "10.0.0.3";
const uint16_t mqtt_port      = 1884;
const char* mqtt_device       = "air_esp_living";
const char* mqtt_device_name  = "AirESP Living";

int online_status = 0; // 0 all ok, -2 no wifi, -3 no mqtt, -4 no air response
bool msg_received_flag = false;

bool air_error = false;
bool air_needs_update = false;

unsigned long last_mqtt_attempt = 0;
unsigned long last_heartbit = 0;
unsigned long last_led_blink = 0;
unsigned long last_air_message = 0;

const size_t mqtt_topic_len = 100;
char mqtt_topic[100];
const size_t mqtt_msg_len = 50;
char mqtt_msg[50];

void update_online_status();
void blink_led_on_interval();

void talk_to_air_on_interval();

void connect_mqtt_if_needed();
void handle_mqtt_message(char* rcv_topic, byte* payload, unsigned int length);
void send_heartbit_on_interval();
void format_topic(const char* control, const char* meta, bool setter);
void mqtt_publish_state();
void mqtt_publish_heartbit();
void mqtt_publish_meta();
void mqtt_publish_debug(const char* text);

void setup() {
  Serial.begin(4800, SERIAL_8E1);
  pinMode(LED, OUTPUT);

  WiFi.begin(ssid, password);

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(handle_mqtt_message);
}

void loop() {
  mqtt.loop();
  connect_mqtt_if_needed();
  update_online_status();
  talk_to_air_on_interval();
  blink_led_on_interval();
  send_heartbit_on_interval();
  delay(50);
}

// ========== General ==========

void update_online_status() {
  if (!WiFi.status() == WL_CONNECTED) {
    online_status = -2;
  } else if (!mqtt.connected()) {
    online_status = -3;
  } else if (air_error) {
    online_status = -4;
  } else {
    online_status = 0;
  }
}

void blink_led_on_interval() {
  if (!msg_received_flag && millis() - last_led_blink < BLINK_INTERVAL) {
    return;
  }
  if (online_status == 0 && !msg_received_flag) {
    return;
  }
  last_led_blink = millis();
  int blinks = 0;
  if (online_status != 0) {
    blinks = abs(online_status);
  } else if (msg_received_flag) {
    blinks = 1;
    msg_received_flag = 0;
  }
  while (blinks) {
    blinks--;
    digitalWrite(LED, 1);
    delay(100);
    digitalWrite(LED, 0);
    delay(100);
  }
}

// ========== AIR ==========

struct air_state_t {
  char mode; // 0 off, 1 auto, 2 cool, 3 dry, 4 fun, 5 heat
  char temp; // 16...30
  char fun; // 0...5
  bool swing; // 0...1
  bool light; // 0...1
};

struct air_state_t last_air_state;

const char payload_prefix[4] = { 0x1, 0x0, 0x0, 0x0 };
const char payload_postfix[25] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0 };
char config[16] = { 0x0, 0x70, 0xa, 0x2, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
char air_msg[51];

bool is_params_valid(struct air_state_t s) {
  if (s.mode < 0 || s.mode > 5) return false;
  if (s.temp < 16 || s.temp > 30) return false;
  if (s.fun < 0 || s.fun > 5) return false;
  return true;
}

air_state_t decode_config() {
  struct air_state_t s;
  char mode = (config[0] >> 4) & 0x0f;
  s.mode = mode <= 7 ? 0 : mode - 7;
  char temp = ((config[1] >> 4) & 0x0f);
  s.temp = temp + 16;
  s.fun = config[14] & 0x0f;
  s.swing = (config[4] >> 4) > 0 ? 1 : 0;
  s.light = (config[2] & 0x02) >> 1;
  return s;
}

void encode_config(air_state_t s) {
  char mode = 0;
  if (s.mode > 0) mode = s.mode + 7;
  char fun1 = 0;
  if (s.fun < 3) {
    fun1 = s.fun;
  } else if (s.fun == 3) {
    fun1 = 2;
  } else if (s.fun > 3) {
    fun1 = 3;
  }
  config[0] = (mode << 4) | fun1;
  char temp = s.temp - 16;
  config[1] = (temp << 4) | (config[1] & 0x0f);
  config[2] = (config[2] & (~(1 << 1))) | (s.light << 1);
  config[4] = s.swing << 4;
  config[14] = s.fun;
}

void write_config_to_serial(bool setter) { // msg size 50
  air_msg[0] = 0x7e; // head
  air_msg[1] = 0x7e;
  air_msg[2] = 0x2f; // length
  if (setter) {
      air_msg[7] = 0xaf;
  } else {
      air_msg[7] = 0x00;
  }
  
  int i = 3;
  memcpy(air_msg + i, payload_prefix, sizeof(payload_prefix));
  i += sizeof(payload_prefix) + 1;
  memcpy(air_msg + i, config, sizeof(config));
  i += sizeof(config);
  memcpy(air_msg + i, payload_postfix, sizeof(payload_postfix));
  
  int sum = 0;
  for (int i = 2; i < 48; i++) {
      sum += air_msg[i];
  }
  sum %= 256;
  air_msg[49] = sum;

  Serial.write(air_msg, 50);
  Serial.flush();
}

bool read_config_from_serial() {
  char state = 0;
  char length = 0;
  int i = 0;
  while(1) {
    if (!Serial.available()) {
      return false; 
    }
    char c = (char)Serial.read();
    if (state == 0) {
      if (c == 0x7e) {
        state = 1;
      }
    } else if (state == 1) {
      if (c == 0x7e) {
        state = 2;
      } else {
        state = 0;
      }
    } else if (state == 2) {
      length = c;
      i = 3;
      state = 3;
    } else if (state == 3) {
      air_msg[i] = c;
      i++;
      length -= 1;
      if (length == 0) {
        msg_received_flag = true;
        memcpy(config, air_msg + 8, sizeof(config));
        return true;
      }
    } 
  }
}

void talk_to_air_on_interval() {
  if (millis() - last_air_message < AIR_MSG_INTERVAL) {
    return;
  }
  last_air_message = millis();

  write_config_to_serial(air_needs_update);
  air_needs_update = false;
  air_error = !read_config_from_serial();
  if (air_error) return;
  
  struct air_state_t state = decode_config();
  if (last_air_state.mode != state.mode || last_air_state.temp != state.temp ||
      last_air_state.fun != state.fun || last_air_state.swing != state.swing ||
      last_air_state.light != state.light) {
    last_air_state = state;
    mqtt_publish_state();
  }
}

// ========== MQTT ==========

void connect_mqtt_if_needed() {
  if (mqtt.connected() || millis() - last_mqtt_attempt < MQTT_RECON_INTERVAL) {
    return;
  }
  format_topic("Connected", "", false);
  if (!mqtt.connect(mqtt_device, mqtt_login, mqtt_pass, mqtt_topic, 1, true, "0")) {
    last_mqtt_attempt = millis();
    return;
  }

  // handle mqtt connected
  // send heartbit
  mqtt_publish_meta();
  mqtt_publish_heartbit();
  last_heartbit = millis();

  // subscribe for "on" topics
  format_topic("Mode", "", true);
  mqtt.subscribe(mqtt_topic);
  format_topic("Temp", "", true);
  mqtt.subscribe(mqtt_topic);
  format_topic("Fun", "", true);
  mqtt.subscribe(mqtt_topic);
  format_topic("Swing", "", true);
  mqtt.subscribe(mqtt_topic);
  format_topic("Light", "", true);
  mqtt.subscribe(mqtt_topic);
  format_topic("X-HM-Mode", "", true);
  mqtt.subscribe(mqtt_topic);
}

void handle_mqtt_message(char* rcv_topic, byte* payload, unsigned int length) {
  msg_received_flag = true;
  int value = atoi((char*)payload);

  struct air_state_t state = last_air_state;

  format_topic("Mode", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    state.mode = (char)value;
  }
  format_topic("X-HM-Mode", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    if (value == 1) state.mode = 5;
    else if (value == 2) state.mode = 2;
    else state.mode = 0;
  }
  format_topic("Temp", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    state.temp = (char)value;
  }
  format_topic("Fun", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    state.fun = (char)value;
  }
  format_topic("Swing", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    state.swing = (value > 0);
  }
  format_topic("Light", "", true);
  if (strcmp(mqtt_topic, rcv_topic) == 0) {
    state.light = (value > 0);
  }

  // snprintf(mqtt_msg, mqtt_msg_len, "status: m=%d t=%d f=%d sw=%d l=%d", state.mode, state.temp, state.fun, state.swing, state.light);
  // mqtt_publish_debug(mqtt_msg);

  if (!is_params_valid(state)) {
    // mqtt_publish_debug("Incorrect topic value");
    return;
  }  

  encode_config(state);
  air_needs_update = true;
}

void send_heartbit_on_interval() {
  if (millis() - last_heartbit < HEARTBIT_INTERVAL) {
    return;
  }
  mqtt_publish_heartbit();
  last_heartbit = millis();
}

void format_topic(const char* control, const char* meta, bool setter) {
  if (*control) {
    if (*meta) {
      snprintf(mqtt_topic, mqtt_topic_len, "/devices/%s/controls/%s/meta/%s", mqtt_device, control, meta);
    } else {
      if (setter) {
        snprintf(mqtt_topic, mqtt_topic_len, "/devices/%s/controls/%s/on", mqtt_device, control);
      } else {
        snprintf(mqtt_topic, mqtt_topic_len, "/devices/%s/controls/%s", mqtt_device, control);
      }
    }
  } else {
    if (*meta) {
      snprintf(mqtt_topic, mqtt_topic_len, "/devices/%s/meta/%s", mqtt_device, meta);
    } else {
      snprintf(mqtt_topic, mqtt_topic_len, "/devices/%s", mqtt_device);
    }
  }
}

void mqtt_publish_state() {
  format_topic("Mode", "", false);
  snprintf(mqtt_msg, mqtt_msg_len, "%d", (int)last_air_state.mode);
  mqtt.publish(mqtt_topic, mqtt_msg, true);

  format_topic("X-HM-Mode", "", false);
  int hm_mode = 0;
  if (last_air_state.mode == 5) hm_mode = 1;
  else if (last_air_state.mode == 2) hm_mode = 2;
  snprintf(mqtt_msg, mqtt_msg_len, "%d", hm_mode);
  mqtt.publish(mqtt_topic, mqtt_msg, true);

  format_topic("Temp", "", false);
  snprintf(mqtt_msg, mqtt_msg_len, "%d", (int)last_air_state.temp);
  mqtt.publish(mqtt_topic, mqtt_msg, true);

  format_topic("Fun", "", false);
  snprintf(mqtt_msg, mqtt_msg_len, "%d", (int)last_air_state.fun);
  mqtt.publish(mqtt_topic, mqtt_msg, true);

  format_topic("Swing", "", false);
  mqtt.publish(mqtt_topic, last_air_state.swing ? "1" : "0", true);

  format_topic("Light", "", false);
  mqtt.publish(mqtt_topic, last_air_state.light ? "1" : "0", true);
}

void mqtt_publish_heartbit() {
  format_topic("Connected", "", false);
  mqtt.publish(mqtt_topic, "1", true);

  format_topic("IP", "", false);
  mqtt.publish(mqtt_topic, WiFi.localIP().toString().c_str(), true);

  format_topic("RSSI", "", false);
  snprintf(mqtt_msg, mqtt_msg_len, "%d dB", (int)WiFi.RSSI());
  mqtt.publish(mqtt_topic, mqtt_msg, true);
}

void mqtt_publish_meta() {
  // Device name
  format_topic("", "name", false);
  mqtt.publish(mqtt_topic, mqtt_device_name, true);

  // Heartbit
  format_topic("Connected", "type", false);
  mqtt.publish(mqtt_topic, "switch", true);
  format_topic("Connected", "readonly", false);
  mqtt.publish(mqtt_topic, "1", true);
  format_topic("Connected", "order", false);
  mqtt.publish(mqtt_topic, "0", true);

  format_topic("IP", "type", false);
  mqtt.publish(mqtt_topic, "text", true);
  format_topic("IP", "readonly", false);
  mqtt.publish(mqtt_topic, "1", true);
  format_topic("IP", "order", false);
  mqtt.publish(mqtt_topic, "1", true);

  format_topic("RSSI", "type", false);
  mqtt.publish(mqtt_topic, "text", true);
  format_topic("RSSI", "readonly", false);
  mqtt.publish(mqtt_topic, "1", true);
  format_topic("RSSI", "order", false);
  mqtt.publish(mqtt_topic, "2", true);

  // State
  format_topic("Mode", "type", false);
  mqtt.publish(mqtt_topic, "range", true);
  format_topic("Mode", "max", false);
  mqtt.publish(mqtt_topic, "5", true);
  format_topic("Mode", "order", false);
  mqtt.publish(mqtt_topic, "3", true);

  format_topic("Temp", "type", false);
  mqtt.publish(mqtt_topic, "range", true);
  format_topic("Temp", "max", false);
  mqtt.publish(mqtt_topic, "30", true);
  format_topic("Temp", "order", false);
  mqtt.publish(mqtt_topic, "4", true);

  format_topic("Fun", "type", false);
  mqtt.publish(mqtt_topic, "range", true);
  format_topic("Fun", "max", false);
  mqtt.publish(mqtt_topic, "5", true);
  format_topic("Fun", "order", false);
  mqtt.publish(mqtt_topic, "5", true);

  format_topic("Swing", "type", false);
  mqtt.publish(mqtt_topic, "switch", true);
  format_topic("Swing", "order", false);
  mqtt.publish(mqtt_topic, "6", true);

  format_topic("Light", "type", false);
  mqtt.publish(mqtt_topic, "switch", true);
  format_topic("Light", "order", false);
  mqtt.publish(mqtt_topic, "7", true);

  format_topic("X-HM-Mode", "type", false);
  mqtt.publish(mqtt_topic, "range", true);
  format_topic("X-HM-Mode", "max", false);
  mqtt.publish(mqtt_topic, "2", true);
  format_topic("X-HM-Mode", "order", false);
  mqtt.publish(mqtt_topic, "8", true);
}

void mqtt_publish_debug(const char* text) {
  format_topic("Debug", "", false);
  mqtt.publish(mqtt_topic, text, false);
}
