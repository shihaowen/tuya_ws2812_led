/*
 * @FileName: start.ino
 * @Author: Tuya
 * @Email: 
 * @LastEditors: Tuya
 * @Date: 2021-04-10 11:24:27
 * @LastEditTime: 2021-04-28 19:48:31
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Description: This demo is based on the Arduino UNO, and the LEDs on the UNO board are controlled by the Tuya Smart App. 
 *               Enter network connection mode when Pin7 to GND.
 * @Github:https://github.com/tuya/tuya-wifi-mcu-sdk-arduino-library
 */

#include <TuyaWifi.h>        //涂鸦库
#include <SoftwareSerial.h>  //软串口库
#include "FastLED.h"            // 此示例程序需要使用FastLED库

SoftwareSerial mySerial(9, 10); // RX, TX
TuyaWifi my_device(&mySerial);  //使用软串口
//TuyaWifi my_device;           //使用硬件串口

#define NUM_LEDS 30             // LED灯珠数量             或板载6
#define DATA_PIN 11             // Arduino输出控制信号引脚  或板载A1引脚
#define LED_TYPE WS2812         // LED灯带型号
#define COLOR_ORDER GRB         // RGB灯珠中红色、绿色、蓝色LED的排列顺序
uint8_t max_bright = 128;       // LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];            // 建立光带leds

uint8_t beginHue = 0;  //beginHue参数为起始色调数值
uint8_t deltaHue = 1;  //deltaHue为相邻LED灯珠色调差

/* Current LED status */
unsigned char dp_led_state = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
unsigned char dp_string_value[21] = {"0"};

uint16_t Hue=0; //HSV
uint8_t Sat=0;
uint8_t Val=0;
uint8_t scene_mode=0;

/* Connect network button pin */
int key1_pin = 6;  //LED 绿灯
int key2_pin = 5;  //LED 红灯
int led1_pin = 8;  //KEY1  按键长按进入配网模式
int led2_pin = 7;  //KEY2 

/* Data point define */
#define DPID_SWITCH_LED 20   //开关(可下发可上报)
#define DPID_WORK_MODE 21    //模式(可下发可上报)
#define DPID_MUSIC_DATA 27   //音乐灯(只下发)
//备注:类型：字符串；
//Value: 011112222333344445555；
//0：   变化方式，0表示直接输出，1表示渐变；
//1111：H（色度：0-360，0X0000-0X0168）；
//2222：S (饱和：0-1000, 0X0000-0X03E8）；
//3333：V (明度：0-1000，0X0000-0X03E8）；
//4444：白光亮度（0-1000）；
//5555：色温值（0-1000）
#define DPID_DREAMLIGHT_SCENE_MODE 51   //炫彩情景(可下发可上报)
#define DPID_LIGHTPIXEL_NUMBER_SET 53   //点数/长度设置(可下发可上报)


/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] =
{
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_WORK_MODE, DP_TYPE_ENUM},
  {DPID_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
  {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
};

unsigned char pid[] = {"6tpg2eivatjbhzkx"};   //已替换为幻彩灯带PID
unsigned char mcu_ver[] = {"1.0.0"};

/* last time */
unsigned long last_time = 0;  //记录上次时间

void setup() 
{
  Serial.begin(115200);     //DEBUG串口
  mySerial.begin(9600);     //软件串口初始化

  //Initialize led port, turn off led.
  pinMode(led1_pin, OUTPUT);
  digitalWrite(led1_pin, HIGH);
  pinMode(led2_pin, OUTPUT);
  digitalWrite(led2_pin, HIGH);

  //Initialize networking keys.
  pinMode(key1_pin, INPUT_PULLUP);
  pinMode(key2_pin, INPUT_PULLUP);

  LEDS.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);   // 初始化光带 
  FastLED.setBrightness(max_bright);                               // 设置光带亮度

  my_device.init(pid, mcu_ver);                //Enter the PID and MCU software version
  my_device.set_dp_cmd_total(dp_array, 5);     //incoming all DPs and their types array, DP numbers
  my_device.dp_process_func_register(dp_process);         //register DP download processing callback function
  my_device.dp_update_all_func_register(dp_update_all);   //register upload all DP callback function

  last_time = millis();
}

void loop() 
{
  my_device.uart_service();

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(key1_pin) == LOW)
  {
    delay(200);
    if (digitalRead(key1_pin) == LOW) 
    {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);  //配网
    }
  }
  /* LED blinks when network is being connected */    //配网模式 灯闪
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) 
  {
    if (millis()- last_time >= 500)
    {
        last_time = millis();
  
        if (dp_led_state == LOW) 
        {
          dp_led_state = HIGH;
        } 
        else 
        {
          dp_led_state = LOW;
        }
        digitalWrite(led1_pin, dp_led_state);
    }
  }

  delay(10);
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)   //DP点数据处理
{
  Serial.print("dpid:");
  Serial.println(dpid);
  switch(dpid) 
  {
      case DPID_SWITCH_LED:   //开/关灯
          dp_led_state = my_device.mcu_get_dp_download_data(dpid, value, length);   /* Get the value of the down DP command */
          if (dp_led_state) 
          {
              //digitalWrite(led1_pin, LOW);        //Turn on
              fill_solid(leds, NUM_LEDS, CRGB::White);  //以上语句将leds光带的从头数30个灯珠设置为白颜色。
              FastLED.show();                     // 更新LED色彩
          }
          else
          {
              //digitalWrite(led1_pin, HIGH);      //Turn off
              fill_solid(leds, NUM_LEDS, CRGB::Black);  //以上语句将leds光带的从头数30个灯珠设置为白颜色。
              FastLED.show();                     // 更新LED色彩
          }
          //Status changes should be reported.
          my_device.mcu_dp_update(dpid, value, length);
      break;

      case DPID_WORK_MODE:   //开/关灯
          dp_enum_value = my_device.mcu_get_dp_download_data(dpid, value, length);   /* Get the value of the down DP command */
          switch(dp_enum_value)
          {
                case 0: // white mode
                          //leds[0] = CRGB::Red;              // 设置光带中第一个灯珠颜色为红色，leds[0]为第一个灯珠，leds[1]为第二个灯珠
                          fill_solid(leds, NUM_LEDS, CRGB::White);  //以上语句将leds光带的从头数30个灯珠设置为白颜色。
                          FastLED.show();                     // 更新LED色彩
                          //delay(5);                           // 等待5毫秒
                          Serial.print("white mode");
                break;
                case 1: // colour mode 
                          fill_rainbow(leds, NUM_LEDS, beginHue, deltaHue);
                          //以上语句将leds光带的从头数30个灯珠设置为渐变彩虹色。beginHue参数为起始色调数值。deltaHue为相邻LED灯珠色调差。 
                          FastLED.show();                    // 更新LED色彩
                          //delay(5);                          // 等待5毫秒
                          Serial.print("colour mode");
                break;
                case 2: // scene mode
                break;
                case 3:  // music mode
                break;
          }
          //Status changes should be reported.
          my_device.mcu_dp_update(dpid, value, length);
      break;

      case DPID_MUSIC_DATA: //音乐律动  
          my_device.mcu_dp_update(dpid, value, length);
          //colour_data_control(value, length);

      break;

       case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
           my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, value, length);
           scene_mode=value[1]; 
           switch(scene_mode)
           {
             case 0:
                fill_gradient_RGB(leds, 0, CRGB::Red, NUM_LEDS-1, CRGB::Blue);
                FastLED.show();                     // 更新LED色彩
                //delay(5);                           // 等待5毫秒
                Serial.print("scene_mode0");
                //以上语句将leds光带的从头数30个灯珠设置为渐变色。灯带头部起始颜色为红色。灯带尾部结束颜色为蓝色。灯带中间为由红色到蓝色的渐变色。
                //colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
                break;
              case 1:
                //colorWipe(strip.Color(255,   0,   0), 50);    // Red
                //fadeToBlackBy( leds, 30, 10);
                fill_solid(leds, NUM_LEDS, CRGB::GreenYellow);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode1");
                //delay(5);                           // 等待5毫秒
                break;
              case 2:
                //colorWipe(strip.Color(  0, 255,   0), 50);    // Green
                fill_solid(leds, NUM_LEDS, CRGB::Red);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode2");
                //delay(5);                           // 等待5毫秒
                break;
              case 3:
                //colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
                //fill_solid(leds, NUM_LEDS, CRGB::MediumSeaGreen);
                fill_gradient_RGB(leds, 0, CRGB::DeepPink, NUM_LEDS-1, CRGB::Green);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode3");
                break;
              case 4:
                //theaterChase(strip.Color(127, 127, 127), 50); // White
                //fill_solid(leds, NUM_LEDS, CRGB::Blue);
                fill_rainbow(leds, 30, beginHue, deltaHue);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode4");
                //delay(5);                           // 等待5毫秒
                break;
              case 5:
                //theaterChase(strip.Color(127,   0,   0), 50); // Red
                fill_solid(leds, NUM_LEDS, CRGB::DeepPink);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode5");
                //delay(5);                           // 等待5毫秒
                break;
              case 6:
                //theaterChase(strip.Color(  0,   0, 127), 50); // Blue
                fill_solid(leds, NUM_LEDS, CRGB::Green);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode6");
                //delay(5);                           // 等待5毫秒
                break;
              case 7:
                //rainbow(10);
                fill_solid(leds, NUM_LEDS, CRGB::Pink);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode7");
                //delay(5);                           // 等待5毫秒
                break;
              case 8:
                //theaterChaseRainbow(50);
                fill_solid(leds, NUM_LEDS, CRGB::MediumBlue);
                FastLED.show();                     // 更新LED色彩
                Serial.print("scene_mode8");
                //delay(5);                           // 等待5毫秒
                break;   
           }
        break;

      case DPID_LIGHTPIXEL_NUMBER_SET: //长度设置
           my_device.mcu_dp_update(dpid, value, length);
      break;
      
      default:   break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_LED, dp_led_state, 1);
}
