#include "esp_camera.h"
#include "CTBot.h"
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BME280.h> 

#define I2C_SDA 14
#define I2C_SCL 15
//BOT
CTBot myBot;

Adafruit_MPU6050 mpu;
TwoWire I2CSensors = TwoWire(0);

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model

#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"
const int sensorPIR=33;
const int Buzzer=12;
int PIR=0;
String IP="";


void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  //----------------------------------------------------------------------------------------------
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);


 Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68,&I2CSensors)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
//------------------------------------------------------------------------------

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t1 * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
//-----------------------------------------------------------------------------------------------
 pinMode(sensorPIR, INPUT_PULLUP);
 pinMode(Buzzer, OUTPUT);
 myBot.wifiConnect("POCO X3 Pro", "HoliHolita"); // conectarse a la red
 myBot.setTelegramToken("6210484336:AAF7KFAePDCmvYkvKEiXn9q3zYoSszCY8nw"); // Token de Bot

 if(myBot.testConnection())
      Serial.println("Connection OK");
  else
     Serial.println("Connectionk NOK");

 IP=WiFi.localIP().toString();
 
 startCameraServer();
 

}

void loop() {
  TBMessage msg; // a variable to store telegram message data

  if (myBot.getNewMessage(msg)){
    myBot.sendMessage(msg.sender.id, "Sistema listo para usar");
     Serial.printf("ID:%i ", msg.sender.id);
    }

    // ...forward it to the sender
    
    PIR = digitalRead(sensorPIR);
    if(PIR==1 && mpu.getMotionInterruptStatus() ){
      myBot.sendMessage(msg.sender.id, "Se detecto un movimiento cerca del vehiculo, revisalo: http://"+ IP +"");
       for( int i=0; i<10; i++){
         digitalWrite(Buzzer,HIGH);
      delay (500);
      digitalWrite(Buzzer,LOW);
      delay (500);
    }
   
    
  
    }
   delay(500); // wait 500 milliseconds
}
