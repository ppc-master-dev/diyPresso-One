/*
 * MQTT client (using ArduinoMqttClient) with message formatting in influxDB line format
 * This allows easy forwarding of measurements to influxDB, using telegraf or another client.
 */
#ifndef DP_MQTT_H
#define DP_MQTT_H

#include "dp.h"
#include <ArduinoMqttClient.h>

class MqttDevice
{
    public:
      static const int NUM_SETTABLE = 4; // entries in settable[] (dp_mqtt.cpp), checked there
    private:
      bool _on=false;
      enum mqtt_state_t { MSG_START, MSG_NEXT };
      mqtt_state_t _state = MSG_START;
      double _pending_value[NUM_SETTABLE];
      bool _pending[NUM_SETTABLE] = { false };
      unsigned long _pending_since = 0;
      void prepare(const char *measurement);
      bool queue_settings(char *line);
      int pending_count();
      void adopt_pending();
    public:
      MqttDevice() { off(); }
      void on(void) { _on=true;  }
      void off() { _on = false; }
      bool is_on(void) { return _on; }
      void init();
      void run();
      void write(const char *measurement, long value);
      void write(const char *measurement, double value);
      void write(const char *measurement, const char *value);
      void send();
      void send_state();
      void receive(int size);

};

extern MqttDevice mqttDevice;

#endif // DP_MQTT_H
