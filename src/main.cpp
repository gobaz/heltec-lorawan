#include <Arduino.h>
// #include "heltec.h"

#define HELTEC_NO_DISPLAY               // Disabling default Heltec lib OLED display
// #define HELTEC_NO_RADIOLIB
#define BUTTON 21                       // Redefine button pin 
#define ARDUINO_heltec_wifi_32_lora_V3  // Without this line, Lora Radio doesn't work with Heltec lib

#include "heltec_unofficial.h"
#include <LoRaWAN_ESP32.h>
#include <heltec-eink-modules.h>
#include <SimpleKalmanFilter.h>

EInkDisplay_VisionMasterE290 display;


#define MINIMUM_DELAY 10


void goToSleep( void );
void lora( void );
portTASK_FUNCTION_PROTO( vTaskLed, pvParameters );
portTASK_FUNCTION_PROTO( vTaskEink, pvParameters );
portTASK_FUNCTION_PROTO( vTaskADC, pvParameters );
portTASK_FUNCTION_PROTO( vTaskLora, pvParameters );

LoRaWANNode* node;
RTC_DATA_ATTR uint8_t count = 0;
float CalculatedVoltage = 0.0;


SemaphoreHandle_t sema_CalculatedVoltage;

/************ SETUP *****************/

void setup() {
  heltec_setup();
  analogReadResolution( 12 );
  analogSetPinAttenuation( VBAT_ADC, ADC_0db );
  // pinMode( 46, INPUT_PULLUP );
  pinMode( VBAT_CTRL, OUTPUT );
  digitalWrite( VBAT_CTRL, HIGH );
  float mv = analogReadMilliVolts( 7 ); // dummy read

  display.landscape();
  display.fastmodeOn();
  display.clear();
  // display.printCenter( "Hello, World!" );
  // display.update();
  DRAW( display ) {
    display.printCenter( "Hello, World!" );
  }

  sema_CalculatedVoltage = xSemaphoreCreateBinary();
  xSemaphoreGive( sema_CalculatedVoltage );
  xTaskCreate( vTaskLed, "LED", 1024 * 2, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskEink, "EINK", 1024 * 3, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskADC, "ADC", 1024 * 3, (void*)1, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTaskLora, "LORA", 1024 * 5, (void*)1, tskIDLE_PRIORITY, NULL );

  log_i( "Init." );

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
    temp = 0; //heltec_temperature();

    // Serial.printf( "Temperature: %.1f °C\n", temp );

    unsigned long t1 = millis();
    DRAW( display ) {
      display.setTextSize( 2 );
      display.setCursor( 160, 0 );
      display.printf( "%3d%% %.3fV", vbatp, CalculatedVoltage );
      display.setCursor( 230, 20 );
      display.printf( "%.1fC", temp );
      display.setCursor( 0, 40 );
      display.printf( "millis: %ld", pdTICKS_TO_MS( xTaskGetTickCount() ) );
      display.drawLine( 0, 20 - 3, 295, 20 - 3, BLACK );
      display.drawLine( 0, 40 - 3, 295, 40 - 3, BLACK );
      display.drawLine( 0, 60 - 3, 295, 60 - 3, BLACK );
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
  float    adcValue = 0.0f;
  float    Vbatt = 0.0f;
  int      printCount = 0;
  SimpleKalmanFilter KF_ADC_b( 1.0f, 1.0f, .01f );
  uint64_t TimePastKalman = esp_timer_get_time(); // used by the Kalman filter UpdateProcessNoise, time since last kalman calculation
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;;) {
    analogReadRaw( VBAT_ADC ); //read and discard
    adcValue = float( analogReadMilliVolts( VBAT_ADC ) ); //take a raw ADC reading
    KF_ADC_b.setProcessNoise( (esp_timer_get_time() - TimePastKalman) / 1000000.0f ); //get time, in microsecods, since last readings
    adcValue = KF_ADC_b.updateEstimate( adcValue ); // apply simple Kalman filter
    Vbatt = adcValue / 204.5;//vRefScale;
    xSemaphoreTake( sema_CalculatedVoltage, portMAX_DELAY );
    CalculatedVoltage = Vbatt;
    xSemaphoreGive( sema_CalculatedVoltage );

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

}


/*************************************/

portTASK_FUNCTION( vTaskLora, pvParameters ) {

  const TickType_t xFrequency = 10000;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool initialized = false;
  int16_t state;

  for ( ;;) {
    // radio.
    // initialize radio
    // if ( initialized ) {
    //   state = RADIOLIB_ERR_NONE;
    // }
    // else {
      state = radio.begin();
    // }
    if ( state != RADIOLIB_ERR_NONE ) {
      log_e( "Radio did not initialize. We'll try again later." );
      // goToSleep();
      // goto wait1;
    }
    else {
      if ( initialized ) {
        log_i( "Radio already initialized." );
      }
      else {
        log_i( "Radio initialized." );
      }
      initialized = true;

      node = persist.manage( &radio );

      if ( !node->isActivated() ) {
        log_e( "Could not join network. We'll try again later." );
        // goToSleep();
        // goto wait1;
      }
      else {

        // If we're still here, it means we joined, and we can send something

          // Manages uplink intervals to the TTN Fair Use Policy
        node->setDutyCycle( true, 1250 );

        uint8_t uplinkData[2];
        LoRaWANEvent_t evUp, evDown;
        uplinkData[0] = count++;
        float temp = heltec_temperature();
        uplinkData[1] = temp + 100;

        uint8_t downlinkData[256];
        size_t lenDown = sizeof( downlinkData );

        state = node->sendReceive( uplinkData, sizeof( uplinkData ), 1, downlinkData, &lenDown, false, &evUp, &evDown );


        if ( state == RADIOLIB_ERR_NONE ) {
          log_i( "Message sent, no downlink received." );
          // Serial.printf( "confirmed: %x\nconfirming: %x\ndatarate: %u\nfreq: %u\npower %u\nfCnt: %lu\nfport: %u\nnbtrans: %u\n",
          //   evUp.confirmed, evUp.confirming, evUp.datarate, evUp.freq, evUp.power, evUp.fCnt, evUp.fPort, evUp.nbTrans );
        }
        else if ( state > 0 ) {
          // Serial.printf( "confirmed: %x\nconfirming: %x\ndatarate: %u\nfreq: %u\npower %u\nfCnt: %lu\nfport: %u\nnbtrans: %u\n",
          //   evUp.confirmed, evUp.confirming, evUp.datarate, evUp.freq, evUp.power, evUp.fCnt, evUp.fPort, evUp.nbTrans );
          // Serial.printf( "confirmed: %x\nconfirming: %x\ndatarate: %u\nfreq: %u\npower %u\nfCnt: %lu\nfport: %u\nnbtrans: %u\n",
          //   evDown.confirmed, evDown.confirming, evDown.datarate, evDown.freq, evDown.power, evDown.fCnt, evDown.fPort, evDown.nbTrans );

          log_i( "Message sent, downlink received." );
          log_i( "donw (%d): ", lenDown );
          for ( int i = 0; i < lenDown; i++ ) log_i( "0x%02X ", downlinkData[i] );
          // log_i( "" );
          if ( lenDown >= 2 && downlinkData[0] == 0x4C && downlinkData[1] <= 100 ) {
            heltec_led( downlinkData[1] );
          }
        }
        else {
          log_e( "sendReceive returned error %d, we'll try again later.\n", state );
        }

      }
    }
    persist.saveSession( node );
    log_i( "stack: %d", uxTaskGetStackHighWaterMark( NULL ) );
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
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
