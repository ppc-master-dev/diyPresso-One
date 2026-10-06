#include "dp_mqtt.h"
#include <WiFiNINA.h>
#include "dp_boiler.h"
#include "dp_brew.h"
#include "dp_reservoir.h"
#include "dp_heater.h"
#include "dp_settings.h"
#include "dp_hardware.h"
#include "dp_steam_process.h"
#include "dp_menu.h"


MqttDevice mqttDevice;


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


const char broker[] = "test.mosquitto.org";
int port = 1883;
char state_topic[64] = "";
char set_topic[64] = "";

#define TX_PAYLOAD_SIZE 512         // the state message outgrows the library's 256 byte default
#define ADOPT_DELAY_MSEC (10 * 1000) // queued settings must be unchanged this long before adoption

// Settings that may be changed over MQTT, keyed by their line protocol field name.
// t_set matches the field in the state message, so a client reads back what it sets.
typedef double (DpSettings::*setter_t)(double);
typedef struct { const char *key; setter_t set; } settable_t;

static const settable_t settable[] = {
    { "t_set", &DpSettings::temperature },
    { "P", &DpSettings::P },
    { "I", &DpSettings::I },
    { "D", &DpSettings::D },
};

static int settable_index(const char *key)
{
    for (int i = 0; i < (int)(sizeof(settable) / sizeof(settable[0])); i++)
        if (strcmp(key, settable[i].key) == 0)
            return i;
    return -1;
}

static void on_message(int size)
{
    mqttDevice.receive(size);
}


void mac_to_hex(char *hex, byte *mac)
{
    static const char *hexchar = "0123456789ABCDEF";
    // WiFiNINA's WiFi.macAddress() fills the bytes LSB first; reverse them
    // so the topic matches the MAC as shown by routers and network tools.
    for(int i=0; i<6; i++)
    {
        hex[2*i+0] = hexchar[ (mac[5-i] >> 4) ];
        hex[2*i+1] = hexchar[ (mac[5-i] & 15) ];
    }
    hex[12] = 0;
}

void MqttDevice::init()
{
  byte mac[6]; // Wifi MAC address

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  WiFi.macAddress(mac);
  char mac_hex[13];
  mac_to_hex(mac_hex, mac);
  snprintf(state_topic, sizeof(state_topic), "diyPressoOne/%s/state", mac_hex);
  snprintf(set_topic, sizeof(set_topic), "diyPressoOne/%s/set", mac_hex);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    return;
  }
  on();

  mqttClient.setTxPayloadSize(TX_PAYLOAD_SIZE);
  mqttClient.onMessage(on_message);
  if (!mqttClient.subscribe(set_topic)) {
    Serial.print("MQTT subscribe failed: ");
    Serial.println(set_topic);
  }
  
  Serial.println("You're connected to the MQTT broker! Subscribe using mosquitto with this command:");
  
  Serial.print("mosquitto_sub -h ");
  Serial.print(broker);
  Serial.print(" -t ");
  Serial.println(state_topic);
  Serial.print("Settings are accepted on: ");
  Serial.println(set_topic);
}

/*
 * Read a message from the set topic, e.g. "set t_set=95.0,P=7.0".
 * The measurement name is ignored; only the fields in settable[] are accepted.
 */
void MqttDevice::receive(int size)
{
    char line[128];

    if (size <= 0 || size >= (int)sizeof(line))
    {
        while (mqttClient.available())
            mqttClient.read();
        Serial.println("MQTT set rejected: message too long");
        return;
    }
    int n = mqttClient.read((uint8_t *)line, size);
    line[n] = 0;
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
        line[--n] = 0;

    Serial.print("MQTT set: ");
    Serial.println(line);
    if (!queue_settings(line))
        Serial.println("MQTT set rejected");
}

/*
 * Parse the field list strictly and queue the values: an unknown key, a value that is
 * not a finite number, or a timestamp rejects the whole line, so a set of PID values
 * is never applied in part. Values are clamped when they are adopted, not here.
 */
bool MqttDevice::queue_settings(char *line)
{
    double value[NUM_SETTABLE];
    bool present[NUM_SETTABLE] = { false };
    bool any = false;

    char *fields = strchr(line, ' ');
    if (fields == NULL)
        return false;
    fields++;
    if (strchr(fields, ' ') != NULL)
        return false;

    for (char *field = strtok(fields, ","); field != NULL; field = strtok(NULL, ","))
    {
        char *eq = strchr(field, '=');
        if (eq == NULL)
            return false;
        *eq = 0;
        int i = settable_index(field);
        if (i < 0)
            return false;
        char *end;
        double v = strtod(eq + 1, &end);
        if (end == eq + 1 || *end != 0 || !isfinite(v))
            return false;
        value[i] = v;
        present[i] = true;
        any = true;
    }
    if (!any)
        return false;

    for (int i = 0; i < NUM_SETTABLE; i++)
    {
        if (present[i])
        {
            _pending_value[i] = value[i];
            _pending[i] = true;
        }
    }
    _pending_since = millis();
    return true;
}

int MqttDevice::pending_count()
{
    int count = 0;
    for (int i = 0; i < NUM_SETTABLE; i++)
        if (_pending[i])
            count++;
    return count;
}

/*
 * Adopt queued settings only while the machine is idle: commissioned, no brew, no steam,
 * nobody editing settings at the machine. Saving writes flash, which stalls the CPU, and
 * BrewProcess reloads the setpoint from settings on every return to idle anyway.
 * Settings edited at the machine win: an open settings menu drops what is queued.
 */
void MqttDevice::adopt_pending()
{
    if (pending_count() == 0)
        return;

    if (menu_settings_active())
    {
        for (int i = 0; i < NUM_SETTABLE; i++)
            _pending[i] = false;
        Serial.println("MQTT set dropped: settings menu is open");
        return;
    }

    if (millis() - _pending_since < ADOPT_DELAY_MSEC)
        return;
    if (!settings.commissioningDone() || !brewProcess.is_idle())
        return;
    if (hardware.has_steam_group() && !steamProcess.is_idle())
        return;

    for (int i = 0; i < NUM_SETTABLE; i++)
    {
        if (_pending[i])
        {
            (settings.*settable[i].set)(_pending_value[i]);
            _pending[i] = false;
        }
    }
    settings.save();
    settings.apply();
    Serial.println("MQTT set adopted");
}


void MqttDevice::run() 
{
    if ( !is_on() )
        return;

    // call poll() regularly to allow the library to send MQTT keep alive which avoids being disconnected by the broker
    mqttClient.poll();
    adopt_pending();
}

void MqttDevice::prepare(const char *measurement)
{
    if (_state == MSG_START)
    {
        mqttClient.beginMessage(state_topic);
        mqttClient.print("measurement ");
    }
    if (_state == MSG_NEXT)
        mqttClient.print(",");
    mqttClient.print(measurement);
    mqttClient.print("=");
    _state = MSG_NEXT;
}

void MqttDevice::write(const char *measurement, double value)
{
    if ( !is_on() )  return;
    prepare(measurement);
    mqttClient.print(value);
}

void MqttDevice::write(const char *measurement, const char *value)
{
    if ( !is_on() ) return;
    prepare(measurement);
    mqttClient.print("\"");
    mqttClient.print(value);
    mqttClient.print("\"");
}

void MqttDevice::write(const char *measurement, long value)
{
    if ( !is_on() ) return;
    prepare(measurement);
    mqttClient.print(value);
}

void MqttDevice::send()
{
    if ( !is_on() ) return;
    mqttClient.endMessage();
    _state = MSG_START;
}

void MqttDevice::send_state()
{
  static unsigned long prev_time = millis();
  if (millis() - prev_time > 5000)
  {
    write("t_set", boilerController.set_temp());
    write("t_act", boilerController.act_temp());
    write("h_pwr", heaterDevice.power());
    write("h_avg", heaterDevice.average());
    write("r_lvl", reservoir.level());
    write("r_wgt", reservoir.weight());
    write("w_cur", brewProcess.weight());
    write("w_end", brewProcess.end_weight());
    write("shots", (long)settings.shotCounter());
    write("P", settings.P());
    write("I", settings.I());
    write("D", settings.D());
    int pending = pending_count();
    if (pending > 0)
      write("pend", (long)pending);

    write("boil", boilerController.get_state_name());
    if (boilerController.is_error())
      write("boil_err", boilerController.get_error_text());

    write("brew", brewProcess.get_state_name());
    if (brewProcess.is_error())
      write("brew_err", brewProcess.get_error_text());

    if (reservoir.is_error())
      write("res_err", reservoir.get_error_text());

    write("msec", (long)millis());
    send();

    prev_time = millis();
  }
}