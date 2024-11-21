#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

const int timer_div_us = 80; 
const int dht11_pin = 21;
const int la1_pin = 36;
const int la2_pin = 37;
const int btn_pin = 0;

const std::string service_uuid = "554cb655-dc08-433f-a42c-cfb8eaa363e1";
const std::string characteristic_uuid_temperature = "25ed47ec-2800-46f0-903d-c75afdd6cc45";
const std::string characteristic_uuid_humidity = "9b5c4bed-5ffb-4c38-af8f-c52f01bd7c63";

const uint8_t flag_id = (1 << 0);
const uint8_t flag_dht11 = (1 << 1);

typedef struct {
  int16_t temperature;
  int16_t humidity;
} sensor_data_t;

class server_callbacks: public BLEServerCallbacks {
  private:
    bool *device_connected;
  public:
    server_callbacks(bool *device_connected) : device_connected(device_connected) {
    }

    void onConnect(BLEServer* pServer) {
      *device_connected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      *device_connected = false;
    }
};

hw_timer_t *timer0_cfg = nullptr;
QueueHandle_t queue_uart, queue_ble, queue_dht11_timestamp;
EventGroupHandle_t event_group;
TimerHandle_t timer;

void task_heartbeat(void *args);
void task_fastbeat(void *args);
void task_dht11(void *args);
void task_uart(void *args);
void timer_callback(TimerHandle_t t);

void dht11_measurement(uint16_t *temperature, uint16_t *humidity);
void handle_error();

void IRAM_ATTR dht11_pin_isr() {
  uint64_t value = timerRead(timer0_cfg);
  BaseType_t higher_priority_task_woken = pdFALSE;
  if (xQueueSendFromISR(queue_dht11_timestamp, &value, &higher_priority_task_woken) != pdPASS)
    handle_error();
}

void IRAM_ATTR timer0_isr() {
  pinMode(dht11_pin, INPUT_PULLUP);
  timerAlarmDisable(timer0_cfg);
  BaseType_t higher_priority_task_woken = pdFALSE;
}

void dht11_measurement(int16_t *temperature, int16_t *humidity) {
  uint64_t value, last = 0;
  bool started = false;
  uint32_t low_portion = 0;
  uint8_t val = 0, humidity1, humidity2, temperature1, temperature2;
  uint32_t count = 0;
  uint8_t retry = 0;

  while (xQueueReceive(queue_dht11_timestamp, &value, 0) == pdPASS); //Drain timestamp queue

  pinMode(dht11_pin, OUTPUT);
  digitalWrite(dht11_pin, LOW);
  timerWrite(timer0_cfg, 0);
  timerAlarmWrite(timer0_cfg, 18000, true);
  timerAlarmEnable(timer0_cfg);

  while(count < 40) {
    if (xQueueReceive(queue_dht11_timestamp, &value, portMAX_DELAY) != pdPASS) {
      handle_error();
      return;
    }

    uint32_t duration = value - last;
    if (duration == 0) {
      handle_error();
      return;
    }
    last = value;

    if (!started) {
      if (low_portion > 60 && low_portion < 100 && duration > 60 && duration < 100) {        
        started = true;
        low_portion = 0;
      } else {        
        low_portion = duration;
        retry++;
        if (retry == 20) {
          handle_error();
          return;
        }
      }
    } else {
      if (low_portion == 0)
        low_portion = duration;
      else {
        bool bit;
        if (low_portion > 30 && low_portion < 70 && duration > 10 && duration < 45) {
          val <<= 1;
          count ++;
        } else if (low_portion > 30 && low_portion < 70 && duration > 50 && duration < 90) {
          val <<= 1;
          val |= 1;
          count ++;
        } else {
          handle_error();
          return;
        }
        low_portion = 0;

        if (count == 8) {
          humidity1 = val;
          val = 0;
        } else if (count == 16) {
          humidity2 = val;
          val = 0;
        } else if (count == 24) {
          temperature1 = val;
          val = 0;
        } else if (count == 32) {
          temperature2 = val;
          val = 0;
        } else if (count == 40) {
          if (humidity1 + humidity2 + temperature1 + temperature2 != val) {
            handle_error();
            return;
          }
          else {
            *humidity = (humidity1 << 8) | humidity2;
            *temperature = (temperature1 << 8) | temperature2;
          }
        }
      }
    }
  }
}

void handle_error() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
}

void task_heartbeat(void *args) {
  pinMode(la1_pin, OUTPUT);
  for (;;) {
    digitalWrite(la1_pin, !digitalRead(la1_pin));
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void task_id(void *args) {
  uint8_t btn_last = HIGH;
  pinMode(btn_pin, INPUT_PULLUP);

  for (;;) {
    uint8_t btn = digitalRead(btn_pin);
    if (btn == LOW && btn_last == HIGH) {
        vTaskDelay(10L / portTICK_PERIOD_MS);
        btn = digitalRead(btn_pin);
        if (btn == LOW)
          xEventGroupSetBits(event_group, flag_id);
    }
    btn_last = btn;
    vTaskDelay(10L / portTICK_PERIOD_MS);
  }
}

void task_fastbeat(void *args) {
  pinMode(la2_pin, OUTPUT);
  for (;;) {
    vTaskDelay(1L / portTICK_PERIOD_MS);
    digitalWrite(la2_pin, !digitalRead(la2_pin));
  }
}

void task_dht11(void *args) {
  pinMode(dht11_pin, INPUT_PULLUP);
  attachInterrupt(dht11_pin, dht11_pin_isr, CHANGE);  
  timer0_cfg = timerBegin(0, timer_div_us, true);  
  timerAttachInterrupt(timer0_cfg, &timer0_isr, true);
  
  bool led_status = false;
  while(true) {
    int16_t temperature = INT16_MIN, humidity = INT16_MIN;
    EventBits_t bits = xEventGroupWaitBits(event_group, flag_dht11, pdTRUE, pdFALSE, portMAX_DELAY);
    if ((bits & flag_dht11) == flag_dht11) {
      if (!led_status)
        neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
      else
        neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
      led_status = !led_status;

      dht11_measurement(&temperature, &humidity);
      if (temperature > INT16_MIN && humidity  > INT16_MIN) {
        sensor_data_t data;
        data.temperature = temperature;
        data.humidity = humidity;
        if (xQueueSend(queue_uart, &data, portMAX_DELAY) != pdPASS)
          handle_error();
        if (xQueueSend(queue_ble, &data, portMAX_DELAY) != pdPASS)
          handle_error();
      }
    }
  }
}

void task_uart(void *args) {
  sensor_data_t data;
  uint32_t idx = 0;

  Serial.begin(115200);
  while(!Serial);

  while (true) {
    while (xQueueReceive(queue_uart, &data, 0) == pdPASS) {
      Serial.printf("Sensor Node - FreeRTOS: %3u: %u.%01u%% %u.%01uC\n", idx, data.humidity >> 8, data.humidity & 0xff, data.temperature >> 8, data.temperature & 0xff);
      idx++;
    }
    EventBits_t bits = xEventGroupClearBits(event_group, flag_id);      
    if (bits & flag_id == flag_id) {
      Serial.println("Hello from polzert!");
    }
    vTaskDelay(10L / portTICK_PERIOD_MS);
  }
}

void task_ble(void *args) {
  sensor_data_t data;
  BLEServer *server = nullptr;
  BLECharacteristic *buttonCharacteristic = nullptr;
  bool device_connected = false;
  bool last_connected = false;

  BLEDevice::init("Sensor Node (polzert)");
  server = BLEDevice::createServer();
  server->setCallbacks(new server_callbacks(&device_connected));
  BLEService *service = server->createService(service_uuid);
  BLECharacteristic *characteristic_temperature = service->createCharacteristic(
                                         characteristic_uuid_temperature,
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_READ
                                       );
  characteristic_temperature->addDescriptor(new BLE2902());  
  BLECharacteristic *characteristic_humidity = service->createCharacteristic(
                                         characteristic_uuid_humidity,
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_READ
                                       );
  characteristic_humidity->addDescriptor(new BLE2902());

  service->start();    
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(service_uuid);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  BLEDevice::startAdvertising();

  while (true) {
    // connecting
    if (device_connected && !last_connected) {
      last_connected = device_connected;
      if(xTimerStart(timer, 0) != pdPASS) {
        handle_error();
        return;
      }
    }

    if (device_connected && last_connected) {
      if (xQueueReceive(queue_ble, &data, 0) == pdPASS) {
        uint16_t value = data.temperature;
        characteristic_temperature->setValue(value);
        characteristic_temperature->notify();
        value = data.humidity;
        characteristic_humidity->setValue(value);
        characteristic_humidity->notify();
      }
    }

    if (!device_connected && last_connected) {
      neopixelWrite(RGB_BUILTIN, 0, 0, 0);
      if(xTimerStop(timer, 0) != pdPASS) {
        handle_error();
        return;
      }
      last_connected = device_connected;      
      vTaskDelay(500 / portTICK_PERIOD_MS); // give the bluetooth stack the chance to get things ready
      server->startAdvertising(); // restart advertising
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void timer_callback(TimerHandle_t t) {  
  if (t == timer) {
    xEventGroupSetBits(event_group, flag_dht11);    
  }
}

void setup() {
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  if ((queue_uart = xQueueCreate(8, sizeof(sensor_data_t))) == NULL) {
    handle_error();
    return;
  }
  if ((queue_ble = xQueueCreate(8, sizeof(sensor_data_t))) == NULL) {
    handle_error();
    return;
  }
  if ((queue_dht11_timestamp = xQueueCreate(128, sizeof(uint64_t))) == NULL) {
    handle_error();
    return;
  }
  if ((event_group = xEventGroupCreate()) == NULL) {
     handle_error();
     return;
  }
  if(xTaskCreatePinnedToCore(task_heartbeat, "Task Heartbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if(xTaskCreatePinnedToCore(task_fastbeat, "Task Fastbeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if(xTaskCreatePinnedToCore(task_dht11, "Task DHT11", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if(xTaskCreatePinnedToCore(task_uart, "Task UART", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if(xTaskCreatePinnedToCore(task_id, "Task ID", configMINIMAL_STACK_SIZE + 512, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if(xTaskCreatePinnedToCore(task_ble, "Task BLE", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS) {
    handle_error();
    return;
  }
  if((timer = xTimerCreate("Timer", 5000UL / portTICK_RATE_MS, pdTRUE,  nullptr,  timer_callback)) == NULL) {
    handle_error();
    return;
  }
}

void loop() {
}
