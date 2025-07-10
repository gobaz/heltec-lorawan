#include <Arduino.h>
// #include "heltec.h"

// #define RADIOLIB_GODMODE 1
#define HELTEC_NO_DISPLAY               // Disabling default Heltec lib OLED display
// #define HELTEC_NO_RADIOLIB
#define BUTTON 21                       // Redefine button pin 
#define ARDUINO_heltec_wifi_32_lora_V3  // Without this line, Lora Radio doesn't work with Heltec lib
// #define RADIOLIB_DEBUG_PROTOCOL 1

#include "heltec_unofficial.h"
#include <LoRaWAN_ESP32.h>
#include <heltec-eink-modules.h>
#include <SimpleKalmanFilter.h>

#define MINIMUM_DELAY 10

typedef struct {
  uint8_t size;
  float rssi;
  float snr;
  uint8_t num1;
  uint8_t num2;
  uint8_t num3;
} LoraData_t;


void goToSleep( void );
portTASK_FUNCTION_PROTO( vTaskLed, pvParameters );
portTASK_FUNCTION_PROTO( vTaskEink, pvParameters );
portTASK_FUNCTION_PROTO( vTaskADC, pvParameters );
portTASK_FUNCTION_PROTO( vTaskLora, pvParameters );

EInkDisplay_VisionMasterE290 display;
LoRaWANNode* node;
RTC_DATA_ATTR uint8_t count = 0;
float CalculatedVoltage = 0.0;
LoraData_t  lora_data;

SemaphoreHandle_t sem_CalculatedVoltage;
SemaphoreHandle_t sem_LoraData;

/************ SETUP *****************/

void setup() {
  // Serial.begin(115200);
  heltec_setup();
  delay( 5000 );
  Serial.println( "Starting..." );
  // analogReadResolution( 12 );
  // analogSetPinAttenuation( VBAT_ADC, ADC_0db );
  // pinMode( 46, INPUT_PULLUP );
  // pinMode( 46, OUTPUT );
  // digitalWrite( 46, HIGH );
  float mv = analogReadMilliVolts( 12 ); // dummy read

  display.landscape();
  display.fastmodeOn();
  display.clear();
  // display.printCenter( "Hello, World!" );
  display.update();
  DRAW( display ) {
    display.setTextSize( 2 );
    display.printCenter( "Starting..." );
  }
  vTaskDelay( 3000 );
  Serial.println( "Started." );

  persist.setConsole( Serial );
  // persist.wipe();
  Serial.printf( "band: '%s'\n", persist.getBand() );
  if ( persist.isProvisioned() ) {
    Serial.println( "provisioned" );
  }
  else {
    Serial.println( "NOT provisioned" );
    persist.provision();
  }

  sem_CalculatedVoltage = xSemaphoreCreateBinary();
  xSemaphoreGive( sem_CalculatedVoltage );
  sem_LoraData = xSemaphoreCreateBinary();
  xSemaphoreGive( sem_LoraData );
  // xTaskCreate( vTaskLed, "LED", 1024 * 2, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskEink, "EINK", 1024 * 3, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskADC, "ADC", 1024 * 3, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskLora, "LORA", 1024 * 8, (void*)1, tskIDLE_PRIORITY, NULL );

  // log_i( "Init." );
  Serial.println( "Init." );

  // lora();
}

portTASK_FUNCTION( vTaskLed, pvParameters ) {
  const TickType_t xDelay = 5 / portTICK_PERIOD_MS;

  for ( ;; ) {
    for ( int i = 0;i <= 100;i++ ) {
      vTaskDelay( xDelay );
      heltec_led( i );
    }
    for ( int i = 0;i <= 100;i++ ) {
      vTaskDelay( xDelay );
      heltec_led( 100 - i );
    }
    // log_i( "stack %d", uxTaskGetStackHighWaterMark( NULL ) );
  }
  vTaskDelete( NULL );
}

portTASK_FUNCTION( vTaskEink, pvParameters ) {
  const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
  const TickType_t xFrequency = 1000; //delay for mS
  TickType_t xLastWakeTime = xTaskGetTickCount();
  BaseType_t xWasDelayed;
  // float vbat;
  int vbatp;
  float temp;

  for ( ;; ) {
    // vbat = analogReadMilliVolts( VBAT_ADC ) / 200.58;
    vbatp = heltec_battery_percent( CalculatedVoltage );
    temp = heltec_temperature();

    // Serial.printf( "Temperature: %.1f °C\n", temp );

    unsigned long t1 = millis();
    DRAW( display ) {
      display.setTextSize( 2 );
      display.setCursor( 160, 0 );
      display.printf( "%3d%% %.3fV", vbatp, CalculatedVoltage );
      display.setCursor( 220, 20 );
      display.printf( "%.1f^C", temp );
      display.setCursor( 0, 40 );
      display.printf( "cnt: %u  millis: %ld", count, pdTICKS_TO_MS( xTaskGetTickCount() ) );
      // display.drawLine( 0, 20 - 3, 295, 20 - 3, BLACK );
      // display.drawLine( 0, 40 - 3, 295, 40 - 3, BLACK );
      display.drawLine( 0, 60 - 3, 295, 60 - 3, BLACK );
      display.setCursor( 0, 60 );
      display.printf( "RSSI: %.2f  SNR: %.2f", lora_data.rssi, lora_data.snr );

      if ( lora_data.size > 0 ) {
        display.setTextSize( 3 );
        display.setCursor( 0, 80 );
        display.printf( "%c %c %c", lora_data.num1, lora_data.num2, lora_data.num3 );
      }
    }
    unsigned long t2 = millis();
    // Serial.printf( "draw: %u\n", t2 - t1 );

    vTaskDelay( 1000 );
    // xLastWakeTime = xTaskGetTickCount();
    // xWasDelayed = xTaskDelayUntil( &xLastWakeTime, /* pdMS_TO_TICKS( xFrequency */ 2000 );
    // log_i( "stack: %d", uxTaskGetStackHighWaterMark( NULL ) );

  }
  vTaskDelete( NULL );
}

portTASK_FUNCTION( vTaskADC, pvParameters ) {
  // const float r1 = 390000.0f; // R1 in ohm, 50K
  // const float r2 = 100000.0f; // R2 in ohm, 10k potentiometer
  // float    vRefScale = (3.3f / 4096.0f) * ((r1 + r2) / r2);
  const TickType_t xFrequency = 1000 / 16; //delay for mS
  static TickType_t xLastWakeTime = xTaskGetTickCount();

  float    adcValue = 0.0f;
  float    Vbatt = 0.0f;
  int      printCount = 0;
  SimpleKalmanFilter KF_ADC_b( 1.0f, 1.0f, .01f );
  uint64_t TimePastKalman = esp_timer_get_time(); // used by the Kalman filter UpdateProcessNoise, time since last kalman calculation

  for ( ;;) {
    // analogReadRaw( VBAT_ADC ); //read and discard
    // analogRead( 7 ); //read and discard
    // adcValue = float( analogReadMilliVolts( 7 ) ); //take a raw ADC reading
    adcValue = heltec_vbat();
    KF_ADC_b.setProcessNoise( (esp_timer_get_time() - TimePastKalman) / 1000000.0f ); //get time, in microsecods, since last readings
    adcValue = KF_ADC_b.updateEstimate( adcValue ); // apply simple Kalman filter
    Vbatt = adcValue;// / 204.5;//vRefScale;
    xSemaphoreTake( sem_CalculatedVoltage, portMAX_DELAY );
    CalculatedVoltage = Vbatt;
    xSemaphoreGive( sem_CalculatedVoltage );
    // log_i( "mV: %f vbat: %.4f", adcValue, Vbatt );
    // Serial.printf( "mV: %f vbat: %.4f\n", adcValue, Vbatt );
    vTaskDelay( 100 );
    // xTaskDelayUntil( &xLastWakeTime, xFrequency );

#if 0
    printCount++;
    if ( printCount >= 16 ) {
      log_d( "%.4f vbat: %.4f", adcValue, Vbatt );
      printCount = 0;
    }
#endif

    TimePastKalman = esp_timer_get_time(); // time of update complete
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // log_i( "stack: %d",  uxTaskGetStackHighWaterMark( NULL ) );
  }
  vTaskDelete( NULL );
}

/************ LOOP  *****************/

void loop() {
  heltec_loop();
  delay( 1 ); // IMPORTANT because of wdt reset
}


/*************************************/

portTASK_FUNCTION( vTaskLora, pvParameters ) {

  const TickType_t xFrequency = 15000;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // bool initialized = false;
  int16_t state;

  for ( ;;) {
    state = radio.begin();
    if ( state != RADIOLIB_ERR_NONE ) {
      log_e( "Radio did not initialize. We'll try again later." );
      // goToSleep();
    }
    else {
      log_i( "Radio initialized." );
      // initialized = true;

      node = persist.manage( &radio, true );

      if ( !node->isActivated() ) {
        log_e( "Could not join network. We'll try again later." );
        // goToSleep();
      }
      else {

        // If we're still here, it means we joined, and we can send something

          // Manages uplink intervals to the TTN Fair Use Policy
        node->setDutyCycle( true, 250 );

        uint8_t uplinkData[2];
        LoRaWANEvent_t evUp, evDown;
        uplinkData[0] = count++;
        float temp = heltec_temperature();
        uplinkData[1] = temp;

        uint8_t downlinkData[256];
        size_t lenDown = sizeof( downlinkData );

        state = node->sendReceive( uplinkData, sizeof( uplinkData ), 1, downlinkData, &lenDown, false, &evUp, &evDown );


        if ( state == RADIOLIB_ERR_NONE ) {
          log_i( "Message sent, no downlink received." );
          xSemaphoreTake( sem_LoraData, portMAX_DELAY );
          lora_data.size = 0;
          lora_data.rssi = node->phyLayer->getRSSI();
          lora_data.snr = node->phyLayer->getSNR();
          // lora_data.rssi = node->phy->getRSSI();
          // lora_data.snr = node->phy->getSNR();
          xSemaphoreGive( sem_LoraData );
        }
        else if ( state > 0 ) {

          log_i( "Message sent, downlink received." );
          log_i( "donw (%d): ", lenDown );
          for ( int i = 0; i < lenDown; i++ ) log_i( "0x%02X ", downlinkData[i] );
          // log_i( "" );
          if ( lenDown >= 2 && downlinkData[0] == 0x4C && downlinkData[1] <= 100 ) {
            heltec_led( downlinkData[1] );
          }
          xSemaphoreTake( sem_LoraData, portMAX_DELAY );
          if ( lenDown > 0 ) {
            lora_data.size = lenDown;
            lora_data.num1 = downlinkData[0];
            lora_data.num2 = downlinkData[1];
            lora_data.num3 = downlinkData[2];
          }
          else {
            lora_data.size = 0;
          }
          lora_data.rssi = node->phyLayer->getRSSI();
          lora_data.snr = node->phyLayer->getSNR();
          // lora_data.rssi = node->phy->getRSSI();
          // lora_data.snr = node->phy->getSNR();
          xSemaphoreGive( sem_LoraData );
        }
        else {
          log_e( "sendReceive returned error %d, we'll try again later.\n", state );
        }

      }
    }
    persist.saveSession( node );
    log_w( "stack: %d", uxTaskGetStackHighWaterMark( NULL ) );
    xTaskDelayUntil( &xLastWakeTime, xFrequency );
    // goToSleep();  // Does not return, program starts over next round
  }
  vTaskDelete( NULL );
}


void goToSleep() {
  Serial.println( "Going to deep sleep now" );
  // allows recall of the session after deepsleep
  persist.saveSession( node );
  // Calculate minimum duty cycle delay (per FUP & law!)
  uint32_t interval = node->timeUntilUplink();
  // And then pick it or our MINIMUM_DELAY, whichever is greater
  uint32_t delayMs = max( interval, (uint32_t)MINIMUM_DELAY * 1000 );
  // Serial.printf("Next TX in %i s\n", delayMs/1000);
  Serial.printf( "Next TX in %i s\n", MINIMUM_DELAY );
  delay( 100 );  // So message prints
  // and off to bed we go
  // heltec_deep_sleep(delayMs/1000);
  heltec_deep_sleep( MINIMUM_DELAY );
}
