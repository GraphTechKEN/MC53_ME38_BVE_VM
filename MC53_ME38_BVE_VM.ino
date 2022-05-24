//Adafruit MCP23017 Arduino Library を導入してください
//Arduino Micro または Leonard を使用してください

//簡単な説明
//コマンドに対応しました。デリミタ:CR、Baud:115200、DataBits:8、StopBit:1
//マスコン5段(固定)、ブレーキX段+EBです。 ブレーキ段数は"SET BRK_NUM:X"で段数指定できます。
//ノッチの移動量に応じて、各キーボードコマンドを打ち込んでいることになりますので、BVEのキーアサイン設定はデフォルトに戻してください。
//マスコン側は真理値に基づいてノッチ(N,P1～P5)を指示します。レバーサも対応します。
//ブレーキ側はポテンショの値を一旦角度換算し、ブレーキノッチ(N,B1～EB)を指示します。

//BVEゲーム開始時は、一旦ブレーキハンドルをN→B8(or EB)、マスコンノッチはN、レバーサハンドルをB→N→Fと動かす等してリセットします。

//更新履歴
//MC53_ME38_BVE_VM_V2   ブレーキ弁角度と段数を計算で処理するようにした
//MC53_ME38_BVE_VM_V3   速度計を調整可能にした
//MC53_ME38_BVE_VM_V3.5 電圧計を電流に応じて動かすようにした
//MC53_ME38_BVE_VM_V3.6 ブレーキ弁段数を変更できるようにした
//MC53_ME38_BVE_VM_V3.6.1 電流計を絶対値表示にした、レバーサ不具合修正、ブレーキ角度をPOT_NとPOT_EB間の範囲とした

#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP4725.h>
#include <EEPROM.h>
#include <Wire.h>

#include <Keyboard.h>

//マスコン入力ピンアサイン
#define PIN_MC_1 0
#define PIN_MC_2 1
#define PIN_MC_3 2
#define PIN_MC_4 3
#define PIN_MC_5 4
#define PIN_MC_DEC 5

//レバーサ入力ピンアサイン
#define PIN_MC_DIR_F 6
#define PIN_MC_DIR_B 7

//ホーン入力　※不要な場合はコメントアウト
#define PIN_HORN_1 8
#define PIN_HORN_2 9

//ATS警報持続入力　※不要な場合はコメントアウト
#define PIN_ATS_CONT 10

//スイッチボックス入力　※不要な場合はコメントアウト
#define PIN_ATS_CONF 11
#define PIN_ROOM_LIGHT 12
#define PIN_LIGFT_DEF 13
#define PIN_LIGHT_ON 14
#define PIN_PANTO 15

#define SS_Brk 4   //MCP3008_Brk
#define SS_Mc SS //MCP23S17_MC

//↓デバッグのコメント(//)を解除するとシリアルモニタでデバッグできます
//#define DEBUG

Adafruit_MCP23X17 mcp;
Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac2;

SPISettings settings = SPISettings( 1000000 , MSBFIRST , SPI_MODE0 );

uint16_t ioexp_1_AB = 0;

String strbve = "";
uint16_t bve_speed = 0;
bool bve_door = 0;

uint8_t mcBit = 0;
uint8_t mcBit_latch = 0;
uint8_t notch_mc = 0;
uint8_t notch_mc_latch = 0;
uint8_t notch_mc_H = 0;
uint8_t notch_mc_H_latch = 0;
uint8_t mc_DEC_latch = 0;
String notch_name = "";
uint8_t notch_brk = 0;
uint8_t notch_brk_latch = 0;
String notch_brk_name = "";
//以下ブレーキ設定値
uint16_t notch_brk_angl_max = 80;//直通帯の角度
uint8_t notch_brk_num = 8;//常用ブレーキ段数
uint16_t brk_full_angl = 165;//緩め位置から非常最大までの全体角度
uint16_t brk_eb_angl = 150;//緩め位置に対して非常位置の角度
//以上ブレーキ設定値
//以下速度計補正値
uint16_t spd_adj_010 = 150;
uint16_t spd_adj_020 = 400;
uint16_t spd_adj_030 = 680;
uint16_t spd_adj_040 = 1010;
uint16_t spd_adj_050 = 1330;
uint16_t spd_adj_060 = 1650;
uint16_t spd_adj_070 = 2000;
uint16_t spd_adj_080 = 2340;
uint16_t spd_adj_090 = 2680;
uint16_t spd_adj_100 = 3020;
uint16_t spd_adj_110 = 3340;
uint16_t spd_adj_120 = 3650;
uint16_t spd_adj_130 = 4000;
uint16_t spd_adj_140 = 4095;
uint16_t spd_adj_150 = 4095;
uint16_t spd_adj_160 = 4095;

uint16_t spd_limit = 120;
//以上速度計補正値

int16_t bve_current = 0;

int16_t curr_limit = 750;

uint16_t vehicle_res = 500;

//回生モード
bool curr_kaisei = true; //true:有効　false:無効
//計器モード
bool curr_mode = true; //true:電圧計 false:電流計

int8_t iDir = 0;
int8_t iDir_latch = 0;
String strDir = "";
String strDir_N = "  ";
bool Horn_1 = 0;
bool Horn_1_latch = 0;
bool Horn_2 = 0;
bool Horn_2_latch = 0;
bool Ats_Cont = 0;
bool Ats_Cont_latch = 0;
bool Ats_Conf = 0;
bool Ats_Conf_latch = 0;
bool Panto = 0;
bool Panto_latch = 0;
bool Light_Def = 0;
bool Light_Def_latch = 0;
bool Light_On = 0;
bool Light_On_latch = 0;
bool Room_Light = 0;
bool Room_Light_latch = 0;
float adj_N = 0.0;
float adj_EB = 0.0;

uint16_t POT_N = 0;
uint16_t POT_EB = 0;

unsigned long iniMillis_N = 0;
unsigned long iniMillis_EB = 0;
uint8_t setMode_N = 0;
uint8_t setMode_EB = 0;

//ボタン設定
bool btnSelect = false;
bool btnStart = false;
bool btnA = false;
bool btnB = false;
bool btnC = false;
bool btnD = false;
bool btnSelect_latch = false;
bool btnStart_latch = false;
bool btnA_latch = false;
bool btnB_latch = false;
bool btnC_latch = false;
bool btnD_latch = false;

//運転モード
bool modeBVE = true;
bool modeN = false;

void setup() {
  pinMode(SS_Brk, OUTPUT); //MCP3008
  pinMode(SS_Mc, OUTPUT); //MCP23S17_MC

  pinMode(8, OUTPUT); //BVE_Door
  digitalWrite(8, 0);

  Serial.begin(115200);
  Serial1.begin(115200);
  dac.begin(0x60);
  dac2.begin(0x61);
  Keyboard.begin();

  if (!mcp.begin_SPI(SS_Mc)) {
    Serial.println("Error.");
  } else {
    // マスコンスイッチを全てプルアップ
    for (uint8_t i = 0 ; i < 16 ; i++ ) {
      mcp.pinMode(i, INPUT_PULLUP);
    }
  }

  if (EEPROM.get(0, POT_N) < 0)EEPROM.put(0, 0);
  if (EEPROM.get(2, POT_EB) < 0)EEPROM.put(2, 512);
  if (EEPROM.get(4, notch_brk_num) < 0)EEPROM.put(4, 8);
  if (EEPROM.get(6, notch_brk_angl_max) < 0)EEPROM.put(6, 80);
  if (EEPROM.get(8, brk_eb_angl) < 0)EEPROM.put(8, 150);
  if (EEPROM.get(10, brk_full_angl) < 0)EEPROM.put(10, 165);
  if (EEPROM.get(12, spd_adj_010) < 0)EEPROM.put(12, 150);
  if (EEPROM.get(14, spd_adj_020) < 0)EEPROM.put(14, 400);
  if (EEPROM.get(16, spd_adj_030) < 0)EEPROM.put(16, 680);
  if (EEPROM.get(18, spd_adj_040) < 0)EEPROM.put(18, 1010);
  if (EEPROM.get(20, spd_adj_050) < 0)EEPROM.put(20, 1330);
  if (EEPROM.get(22, spd_adj_060) < 0)EEPROM.put(22, 1650);
  if (EEPROM.get(24, spd_adj_070) < 0)EEPROM.put(24, 2000);
  if (EEPROM.get(26, spd_adj_080) < 0)EEPROM.put(26, 2340);
  if (EEPROM.get(28, spd_adj_090) < 0)EEPROM.put(28, 2680);
  if (EEPROM.get(30, spd_adj_100) < 0)EEPROM.put(30, 3020);
  if (EEPROM.get(32, spd_adj_110) < 0)EEPROM.put(32, 3340);
  if (EEPROM.get(34, spd_adj_120) < 0)EEPROM.put(34, 3650);
  if (EEPROM.get(36, spd_adj_130) < 0)EEPROM.put(36, 4000);
  if (EEPROM.get(38, spd_adj_140) < 0)EEPROM.put(38, 4095);
  if (EEPROM.get(40, spd_adj_150) < 0)EEPROM.put(40, 4095);
  if (EEPROM.get(42, spd_adj_160) < 0)EEPROM.put(42, 4095);
  if (EEPROM.get(44, spd_limit) < 0)EEPROM.put(44, 120);
  if (EEPROM.get(46, curr_kaisei) == -1)EEPROM.put(46, 1);
  if (EEPROM.get(48, curr_mode) == -1)EEPROM.put(48, 1);
  if (EEPROM.get(50, curr_limit) == -1)EEPROM.put(50, 750);
  if (EEPROM.get(52, vehicle_res) == -1)EEPROM.put(52, 500);
}

void loop() {
  //  Serial.println(bve_speed);
  // LOW = pressed, HIGH = not pressed
  //シリアルモニタが止まるのを防止するおまじない
  //BVEモードの時のみシリアル入力を受け付ける
  if (Serial.available() && modeBVE) {
    strbve = Serial.readStringUntil('\r');
    //シリアル設定モード
    if (strbve.startsWith("SET")) {
      //ブレーキ設定モード
      //ブレーキ段数設定
      int8_t i = 0;
      i = strbve.indexOf("BRK_NUM:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 10).toInt();
        if (num == 0 || num > 50) {
          Serial.println("SET NG:BRK_NUM");
        } else {
          notch_brk_num = num;
          Serial.print("SET OK:BRK_NUM=");
          Serial.println(notch_brk_num);
          EEPROM.put(4, notch_brk_num);
        }
      }
      //直通帯範囲
      i = strbve.indexOf("BRK_ANGL:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 9, i + 12).toInt();
        if (num == 0 || num > brk_eb_angl) {
          Serial.println("SET NG:BRK_ANGL");
        } else {
          notch_brk_angl_max = num;
          Serial.print("SET OK:BRK_ANGL=");
          Serial.println(notch_brk_angl_max);
          EEPROM.put(6, notch_brk_angl_max);
        }
      }
      //非常位置
      i = strbve.indexOf("EB_ANGL:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 11).toInt();
        if (num == 0 || num > brk_full_angl) {
          Serial.println("SET NG:EB_ANGL");
        } else {
          brk_eb_angl = num;
          Serial.print("SET OK:EB_ANGL=");
          Serial.println(brk_eb_angl);
          EEPROM.put(8, brk_eb_angl);
        }
      }
      //ブレーキ最大角度
      i = strbve.indexOf("BRK_FULL_ANGL:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 14, i + 17).toInt();
        if (num == 0 || num > 270) {
          Serial.println("SET NG:BRK_FULL_ANGL");
        } else {
          brk_full_angl = num;
          Serial.print("SET OK:BRK_FULL_ANGL=");
          Serial.println(brk_full_angl);
          EEPROM.put(10, brk_full_angl);
        }
      }
      //速度計調整10km/h
      i = strbve.indexOf("SPD_010:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_010");
        } else {
          spd_adj_010 = num;
          Serial.print("SET OK:SPD_010=");
          Serial.println(spd_adj_010);
          EEPROM.put(12, spd_adj_010);
          bve_speed = 100;
        }
      }
      //速度計調整20km/h
      i = strbve.indexOf("SPD_020:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_020");
        } else {
          spd_adj_020 = num;
          Serial.print("SET OK:SPD_020=");
          Serial.println(spd_adj_020);
          EEPROM.put(14, spd_adj_020);
          bve_speed = 200;
        }
      }
      //速度計調整30km/h
      i = strbve.indexOf("SPD_030:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_030");
        } else {
          spd_adj_030 = num;
          Serial.print("SET OK:SPD_030=");
          Serial.println(spd_adj_030);
          EEPROM.put(16, spd_adj_030);
          bve_speed = 300;
        }
      }
      //速度計調整40km/h
      i = strbve.indexOf("SPD_040:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_040");
        } else {
          spd_adj_040 = num;
          Serial.print("SET OK:SPD_040=");
          Serial.println(spd_adj_040);
          EEPROM.put(18, spd_adj_040);
          bve_speed = 400;
        }
      }
      //速度計調整50km/h
      i = strbve.indexOf("SPD_050:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_050");
        } else {
          spd_adj_050 = num;
          Serial.print("SET OK:SPD_050=");
          Serial.println(spd_adj_050);
          EEPROM.put(20, spd_adj_050);
          bve_speed = 500;
        }
      }
      //速度計調整60km/h
      i = strbve.indexOf("SPD_060:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_060");
        } else {
          spd_adj_060 = num;
          Serial.print("SET OK:SPD_060=");
          Serial.println(spd_adj_060);
          EEPROM.put(22, spd_adj_060);
          bve_speed = 600;
        }
      }
      //速度計調整70km/h
      i = strbve.indexOf("SPD_070:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_070");
        } else {
          spd_adj_070 = num;
          Serial.print("SET OK:SPD_070=");
          Serial.println(spd_adj_070);
          EEPROM.put(24, spd_adj_070);
          bve_speed = 700;
        }
      }
      //速度計調整80km/h
      i = strbve.indexOf("SPD_080:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_080");
        } else {
          spd_adj_080 = num;
          Serial.print("SET OK:SPD_080=");
          Serial.println(spd_adj_080);
          EEPROM.put(26, spd_adj_080);
          bve_speed = 800;
        }
      }
      //速度計調整90km/h
      i = strbve.indexOf("SPD_090:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_090");
        } else {
          spd_adj_090 = num;
          Serial.print("SET OK:SPD_090=");
          Serial.println(spd_adj_090);
          EEPROM.put(28, spd_adj_090);
          bve_speed = 900;
        }
      }
      //速度計調整100km/h
      i = strbve.indexOf("SPD_100:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_100");
        } else {
          spd_adj_100 = num;
          Serial.print("SET OK:SPD_100=");
          Serial.println(spd_adj_100);
          EEPROM.put(30, spd_adj_100);
          bve_speed = 1000;
        }
      }
      //速度計調整110km/h
      i = strbve.indexOf("SPD_110:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_110");
        } else {
          spd_adj_110 = num;
          Serial.print("SET OK:SPD_110=");
          Serial.println(spd_adj_110);
          EEPROM.put(32, spd_adj_110);
          bve_speed = 1100;
        }
      }

      //速度計調整120km/h
      i = strbve.indexOf("SPD_120:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_120");
        } else {
          spd_adj_120 = num;
          Serial.print("SET OK:SPD_120=");
          Serial.println(spd_adj_120);
          EEPROM.put(34, spd_adj_120);
          bve_speed = 1200;
        }
      }
      //速度計調整130km/h
      i = strbve.indexOf("SPD_130:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_130");
        } else {
          spd_adj_130 = num;
          Serial.print("SET OK:SPD_130=");
          Serial.println(spd_adj_130);
          EEPROM.put(36, spd_adj_130);
          bve_speed = 1300;
        }
      }
      //速度計調整140km/h
      i = strbve.indexOf("SPD_140:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_140");
        } else {
          spd_adj_140 = num;
          Serial.print("SET OK:SPD_140=");
          Serial.println(spd_adj_140);
          EEPROM.put(38, spd_adj_140);
          bve_speed = 1400;
        }
      }
      //速度計調整150km/h
      i = strbve.indexOf("SPD_150:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_150");
        } else {
          spd_adj_150 = num;
          Serial.print("SET OK:SPD_150=");
          Serial.println(spd_adj_150);
          EEPROM.put(40, spd_adj_150);
          bve_speed = 1500;
        }
      }
      //速度計調整160km/h
      i = strbve.indexOf("SPD_160:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 8, i + 12).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_160");
        } else {
          spd_adj_160 = num;
          Serial.print("SET OK:SPD_160=");
          Serial.println(spd_adj_160);
          EEPROM.put(42, spd_adj_160);
          bve_speed = 1600;
        }
      }

      //回生モード
      i = strbve.indexOf("CURR_KAISEI:");
      if (i > 0) {
        int8_t on = strbve.indexOf("ON");
        int8_t off = strbve.indexOf("OFF");
        if ( on < 0 && off < 0 ) {
          Serial.println("SET NG:CURR_KAISEI");
        } else {
          if ( on > 0 ) {
            curr_kaisei = 1;
            Serial.println("SET OK:CURR_KAISEI:ON");
          } else {
            curr_kaisei = 0;
            Serial.println("SET OK:CURR_KAISEI:OFF");
          }
          EEPROM.put(46, curr_kaisei);
        }
      }
      //計器モード
      i = strbve.indexOf("CURR_MODE:");
      if (i > 0) {
        int8_t V = strbve.indexOf("V");
        int8_t I = strbve.indexOf("I");
        if (V < 0 && I < 0) {
          Serial.println("SET NG:CURR_MODE");
        } else {
          if (V > 0) {
            curr_mode = 1;
            Serial.println("SET OK:CURR_MODE:V");
          } else {
            curr_mode = 0;
            Serial.println("SET OK:CURR_MODE:I");
          }
          EEPROM.put(48, curr_mode);
        }
      }

      //列車抵抗
      i = strbve.indexOf("VEHICLE_RES:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 12, i + 16).toInt();
        if (num == 0) {
          Serial.println("SET NG:VEHICLE_RES");
        } else {
          vehicle_res = num;
          Serial.print("SET OK:VEHICLE_RES=");
          Serial.println(vehicle_res);
          EEPROM.put(52, vehicle_res);
        }
      }
      //最高速度設定
      i = strbve.indexOf("SPD_LIMIT:");
      if (i > 0) {
        uint16_t num = strbve.substring(i + 10, i + 13).toInt();
        if (num == 0) {
          Serial.println("SET NG:SPD_LIMIT");
        } else {
          spd_limit = num;
          Serial.print("SET OK:SPD_LIMIT=");
          Serial.println(spd_limit);
          EEPROM.put(44, spd_limit);
          bve_speed = spd_limit * 10;
        }
      }
      //ブレーキ設定読み出し
      i = strbve.indexOf("BRK_READ:");
      if (i > 0) {
        Serial.print("SET READ:BRK_NUM=");
        Serial.println(notch_brk_num);
        Serial.print("SET READ:BRK_ANGL=");
        Serial.println(notch_brk_angl_max);
        Serial.print("SET READ:EB_ANGL=");
        Serial.println(brk_eb_angl);
        Serial.print("SET READ:BRK_FULL_ANGL=");
        Serial.println(brk_full_angl);
      }
      //速度計設定読み出し
      i = strbve.indexOf("SPD_READ:");
      if (i > 0) {
        Serial.println("SET READ:SPD");
        Serial.print("010=");
        Serial.println(spd_adj_010);
        Serial.print("020=");
        Serial.println(spd_adj_020);
        Serial.print("030=");
        Serial.println(spd_adj_030);
        Serial.print("040=");
        Serial.println(spd_adj_040);
        Serial.print("050=");
        Serial.println(spd_adj_050);
        Serial.print("060=");
        Serial.println(spd_adj_060);
        Serial.print("070=");
        Serial.println(spd_adj_070);
        Serial.print("080=");
        Serial.println(spd_adj_080);
        Serial.print("090=");
        Serial.println(spd_adj_090);
        Serial.print("100=");
        Serial.println(spd_adj_100);
        Serial.print("110=");
        Serial.println(spd_adj_110);
        Serial.print("120=");
        Serial.println(spd_adj_120);
        Serial.print("130=");
        Serial.println(spd_adj_130);
        Serial.print("140=");
        Serial.println(spd_adj_140);
        Serial.print("150=");
        Serial.println(spd_adj_150);
        Serial.print("160=");
        Serial.println(spd_adj_160);
        Serial.print("Limit=");
        Serial.println(spd_limit);
      }
      i = strbve.indexOf("MODE_N:");
      if (i > 0) {
        int8_t on = strbve.indexOf("ON");
        int8_t off = strbve.indexOf("OFF");
        if (on < 0 && off < 0) {
          Serial.println("SET NG");
        } else if (on > 0 && off < 0) {
          modeN = true;
          notch_brk_num = 8;
          Serial.println("SET OK:MODE_N:ON");
        } else {
          modeN = false;
          Serial.println("SET OK:MODE_N:OFF");
        }
      }
    } else {
      //通常モード：速度抽出
      bve_speed = strbve.substring(0, 4).toInt();
      bve_door = strbve.substring(5, 6).toInt();
      bve_current = strbve.substring(7, 12).toInt();
    }

    //速度計のリミットを適用
    if (bve_speed > ( spd_limit * 10 )) {
      bve_speed = ( spd_limit * 10 );
    }

    //速度計補正
    if (bve_speed < 100) {
      dac.setVoltage(map(bve_speed, 0 , 100 , 0, spd_adj_010), false);
    } else if (bve_speed < 200) {
      dac.setVoltage(map(bve_speed, 100 , 200 , spd_adj_010, spd_adj_020), false);
    } else if (bve_speed < 300) {
      dac.setVoltage(map(bve_speed, 200 , 300 , spd_adj_020, spd_adj_030), false);
    } else if (bve_speed < 400) {
      dac.setVoltage(map(bve_speed, 300 , 400 , spd_adj_030, spd_adj_040), false);
    } else if (bve_speed < 500) {
      dac.setVoltage(map(bve_speed, 400 , 500 , spd_adj_040, spd_adj_050), false);
    } else if (bve_speed < 600) {
      dac.setVoltage(map(bve_speed, 500 , 600 , spd_adj_050, spd_adj_060), false);
    } else if (bve_speed < 700) {
      dac.setVoltage(map(bve_speed, 600 , 700 , spd_adj_060, spd_adj_070), false);
    } else if (bve_speed < 800) {
      dac.setVoltage(map(bve_speed, 700 , 800 , spd_adj_070, spd_adj_080), false);
    } else if (bve_speed < 900) {
      dac.setVoltage(map(bve_speed, 800 , 900 , spd_adj_080, spd_adj_090), false);
    } else if (bve_speed < 1000) {
      dac.setVoltage(map(bve_speed, 900 , 1000 , spd_adj_090, spd_adj_100), false);
    } else if (bve_speed < 1100) {
      dac.setVoltage(map(bve_speed, 1000 , 1100 , spd_adj_100, spd_adj_110), false);
    } else if (bve_speed < 1200) {
      dac.setVoltage(map(bve_speed, 1100 , 1200 , spd_adj_110, spd_adj_120), false);
    } else if (bve_speed < 1300) {
      dac.setVoltage(map(bve_speed, 1200 , 1300 , spd_adj_120, spd_adj_130), false);
    } else if (bve_speed < 1400) {
      dac.setVoltage(map(bve_speed, 1300 , 1400 , spd_adj_130, spd_adj_140), false);
    } else if (bve_speed < 1500) {
      dac.setVoltage(map(bve_speed, 1400 , 1500 , spd_adj_140, spd_adj_150), false);
    } else if (bve_speed < 1600) {
      dac.setVoltage(map(bve_speed, 1500 , 1600 , spd_adj_150, spd_adj_160), false);
    } else {
      dac.setVoltage(map(bve_speed, 1600 , 1700 , spd_adj_160, 4095), false);
    }

    //電流計
    if (!curr_kaisei && (bve_current < 0) ) {
      bve_current = 0;
    }
    if (curr_mode) {
      uint16_t v = 1500 - ( bve_current * (vehicle_res / 1000.0 ) ) ;
      if (v > 2000) {
        v = 2000;
      }
      dac2.setVoltage(map(v, 0 , 2000 , 0, 4095), false);
    } else {
      int current = abs(bve_current);
      if (current > curr_limit) {
        current = curr_limit;
      }
      dac2.setVoltage(map(current, 0 , curr_limit , 0, 4095), false);
    }

    //戸閉灯指示
    digitalWrite(8, !bve_door);

    //Serial1デバッグ用
    Serial1.print("BVE Speed:");
    Serial1.println(bve_speed);
    Serial1.print(" Door:");
    Serial1.println(bve_door);

  }

  /*//読み残しは捨てる
    if (Serial.available() && modeBVE) {
    Serial.readStringUntil('\n');
    }*/
  read_IOexp();         //IOエキスパンダ読込ルーチン
  read_modeBVE();       //BVEモード読込ルーチン
  read_MC();            //マスコンノッチ読込ルーチン
  read_Dir();           //マスコンレバーサ読込ルーチン
  read_Break();//ブレーキハンドル読込ルーチン
  read_Break_Setting(); //ブレーキハンドル読込ルーチン(未実装)
  read_Horn();          //ホーンペダル読込ルーチン
  read_Ats();           //ATS確認・警報持続読込ルーチン
  read_Panto();         //強制終了ルーチン
  read_Light_Def();     //減光ライト読込ルーチン
  keyboard_control();   //キーボード(HID)アウトプットルーチン

  delay(10);


#ifdef DEBUG
  Serial.print(" ");
  Serial.print(notch_name);
  Serial.print(" ");
  Serial.print(notch_brk_name);
  Serial.print(" Dir:");
  Serial.print(strDir);
  Serial.println();
#endif
  if (!modeBVE) {
    delay(10);
  }
}

//MCP23S17読込
//値が0でないとき　かつ n 回連続で同じ値を出力したときに値を変更する
void read_IOexp() {
  uint16_t temp_ioexp1_ini = mcp.readGPIOAB();
  uint8_t n = 3;
  if (temp_ioexp1_ini != 0) {
    for (uint8_t i = 0 ; i < n ; i++ ) {
      uint16_t temp_ioexp1 = mcp.readGPIOAB();
      if (temp_ioexp1 != 0) {
        if (temp_ioexp1_ini == temp_ioexp1 ) {
          if (i == n - 1) {
            ioexp_1_AB = temp_ioexp1;
          }
        } else {
          break;
        }
      } else {
        break;
      }
    }
  }
}

//MCP3008ADコンバータ読取
uint16_t adcRead(uint8_t ch) { // 0 .. 7
  byte channelData = (ch + 8 ) << 4;
  // Serial.println(String(channelData,BIN));
  SPI.beginTransaction(settings);
  digitalWrite(SS_Brk, LOW);
  delayMicroseconds(100);
  SPI.transfer(0b00000001); // Start bit 1
  byte highByte = SPI.transfer(channelData); // singleEnd
  byte lowByte = SPI.transfer(0x00); // dummy
  delayMicroseconds(100);
  digitalWrite(SS_Brk, HIGH);
  SPI.endTransaction();
  return ((highByte & 0x03) << 8) + lowByte ;
}

//MCP23S17マスコンノッチ状態読込 (MC53抑速ブレーキ対応)
void read_MC(void) {
  mcBit = ioexp_1_AB;
  if (mcBit != mcBit_latch) {

#ifdef DEBUG
    Serial.print(mcBit | 0x100, DEC);
#endif

    if (ioexp_1_AB & (1 << PIN_MC_DEC)) {
      if (~ioexp_1_AB & (1 << PIN_MC_5)) {
        notch_mc = 55;
        notch_name = "P5";
      } else if (~ioexp_1_AB & (1 << PIN_MC_4)) {
        notch_mc = 54;
        notch_name = "P4";
      } else if (~ioexp_1_AB & (1 << PIN_MC_3)) {
        notch_mc = 53;
        notch_name = "P3";
      } else if (~ioexp_1_AB & (1 << PIN_MC_2)) {
        notch_mc = 52;
        notch_name = "P2";
      } else if (~ioexp_1_AB & (1 << PIN_MC_1)) {
        notch_mc = 51;
        notch_name = "P1";
      } else {
        notch_mc = 50;
        notch_mc_H = 100;
        notch_name = "N ";
      }
    } else {
      if (~ioexp_1_AB & (1 << PIN_MC_5)) {
        notch_mc_H = 101;
        notch_name = "H1";
      } else if (~ioexp_1_AB & (1 << PIN_MC_3) && ioexp_1_AB & (1 << PIN_MC_4)) {
        notch_mc_H = 102;
        notch_name = "H2";
      } else if (~ioexp_1_AB & (1 << PIN_MC_2) && ioexp_1_AB & (1 << PIN_MC_4)) {
        notch_mc_H = 103;
        notch_name = "H3";
      } else if (~ioexp_1_AB & (1 << PIN_MC_2) && ~ioexp_1_AB & (1 << PIN_MC_4)) {
        notch_mc_H = 104;
        notch_name = "H4";
      } else if (~ioexp_1_AB & (1 << PIN_MC_3) && ~ioexp_1_AB & (1 << PIN_MC_4)) {
        notch_mc_H = 105;
        notch_name = "H5";
      }
    }
#ifdef DEBUG
    Serial.print(" mc:");
    Serial.print(notch_mc);
    Serial.print(" mc_latch:");
    Serial.println(notch_mc_latch);
    Serial.print(" mc_H:");
    Serial.print(notch_mc_H);
    Serial.print(" mc_H_latch:");
    Serial.println(notch_mc_H_latch);
#endif

  }
  mcBit_latch = mcBit;
}

//マスコンレバーサ読取
void read_Dir(void) {
  if (~ioexp_1_AB & (1 << PIN_MC_DIR_F)) {
    iDir = 1;
    strDir = "F";
    strDir_N = "L ";
  } else if (~ioexp_1_AB & (1 << PIN_MC_DIR_B)) {
    iDir = -1;
    strDir = "B";
    strDir_N = "R ";
  } else {
    iDir = 0;
    strDir = "N ";
  }
}

//ブレーキ角度読取
void read_Break(void) {
  uint16_t adc = adcRead(0);
  if(adc < POT_N){
    adc = POT_N;
  }else if ( adc > POT_EB){
    adc = POT_EB;
  }
  int16_t deg = map(adc, POT_N , POT_EB , 0, brk_full_angl);
#ifdef DEBUG
  Serial.print(" Pot1:");
  Serial.print(10000 + adcRead(0));
  Serial.print(" Deg:");
  Serial.println(deg);
#endif

  if (deg < ( (float) notch_brk_angl_max / ( notch_brk_num * 2 ))) {
    notch_brk = notch_brk_num + 1;
    notch_brk_name = "N ";
  } else if ( deg < notch_brk_angl_max) {
    uint8_t temp_notch_brk = round( ((float) deg / (float)notch_brk_angl_max) * notch_brk_num );
    notch_brk = notch_brk_num + 1 - temp_notch_brk;
    String s = String( temp_notch_brk );
    notch_brk_name = "B" + s;
  } else if ( deg < brk_eb_angl ) {
    notch_brk = 1;
    notch_brk_name = "B" + String( notch_brk_num );
  } else {
    notch_brk = 0;
    notch_brk_name = "EB";
  }

#ifdef DEBUG
  bool sw = 0;
  if (adcRead(1) < 512)sw = 0; else sw = 1;
  Serial.print(" SW1:");
  Serial.print(sw);

  if (adcRead(2) < 512)sw = 0; else sw = 1;
  Serial.print(" SW2:");
  Serial.print(sw);

  if (adcRead(3) < 512)sw = 0; else sw = 1;
  Serial.print(" SW3:");
  Serial.print(sw);

  if (adcRead(4) < 512)sw = 0; else sw = 1;
  Serial.print(" SW4:");
  Serial.print(sw);
#endif
}

//キーボード(HID)出力
void keyboard_control(void) {
  //マスコンノッチが前回と異なるとき
  if (notch_mc != notch_mc_latch ) {
    if (modeBVE) {
      uint8_t d = abs(notch_mc - notch_mc_latch);
#ifdef DEBUG
      Serial.print(" notch_mc:");
      Serial.print(notch_mc);
      Serial.print(" notch_mc-notch_mc_latch:");
      Serial.print(d);
      Serial.print(" Key:");
#endif
      //力行ノッチ
      if ( notch_mc >= 50 && notch_mc_latch >= 50 && notch_mc <= 55 && notch_mc_latch <= 55 ) {
        //進段
        if ((notch_mc - notch_mc_latch) > 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x5A);//"/"
#ifdef DEBUG
            Serial.println("Z");
#endif
          }
        }
        //戻し
        if ((notch_mc - notch_mc_latch) < 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x41);//"/"
#ifdef DEBUG
            Serial.println("A");
#endif
          }
        }
      }
      if (modeN) {
        if (notch_brk == notch_brk_num + 1) {
          Serial.println(notch_name);
        }
      }
    }
  }

  //抑速ノッチが前回と異なるとき
  if (notch_mc_H != notch_mc_H_latch ) {
    if (modeBVE) {
      uint8_t d = abs(notch_mc_H - notch_mc_H_latch);
      //抑速ノッチ
      if ( notch_mc_H >= 100 && notch_mc_H_latch >= 100 && notch_mc_H <= 105 && notch_mc_H_latch <= 105 ) {
        //進段
        if ((notch_mc_H - notch_mc_H_latch) > 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x51);//"/"
#ifdef DEBUG
            Serial.println("Q");
#endif
          }
        }
        //戻し
        if ((notch_mc_H - notch_mc_H_latch) < 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x41);//"/"
#ifdef DEBUG
            Serial.println("A");
#endif
          }
        }
      }

    }
    if (modeN) {
      if (notch_brk == notch_brk_num + 1) {
        if (ioexp_1_AB & (1 << PIN_MC_DEC) != mc_DEC_latch) {
          if (~ioexp_1_AB & (1 << PIN_MC_DEC)) {
            Serial.println("Co");
          } else {
            Serial.println("N ");
          }
        }
      }
    }
  }
  //mc_DEC_latch = ioexp_1_AB & (1 << PIN_MC_DEC);//ここいる？？

  //ブレーキノッチ(角度)が前回と異なるとき
  if (notch_brk != notch_brk_latch) {
    if (modeBVE) {
      uint8_t d = abs(notch_brk - notch_brk_latch);
#ifdef DEBUG
      Serial.print(" notch_brk:");
      Serial.print(notch_brk);
      Serial.print(" notch_brk-notch_brk_latch:");
      Serial.print(d);
      Serial.print(" Key:");
#endif
      //ブレーキノッチ
      if ( notch_brk <= (notch_brk_num + 1) && notch_brk_latch <= (notch_brk_num + 1) && notch_brk > 0) {
        //戻し
        if ((notch_brk - notch_brk_latch) > 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x2C);//","
#ifdef DEBUG
            Serial.println(",");
#endif
          }
        }
        //ブレーキ
        if ((notch_brk - notch_brk_latch) < 0) {
          for (uint8_t i = 0; i < d ; i ++) {
            Keyboard.write(0x2E);//"."
#ifdef DEBUG
            Serial.println(".");
#endif
          }
        }
      }
      if (notch_brk == 0) {
        Keyboard.write(0x2F);//"/"
#ifdef DEBUG
        Serial.println("/");
#endif
      }
      if (modeN) {
        Serial.println(notch_brk_name);
      }
    }
  }

  //レバーサが前回と異なるとき
  if (iDir != iDir_latch) {
    if (modeBVE) {
      uint8_t d = abs(iDir - iDir_latch);
#ifdef DEBUG
      Serial.print(" iDir:");
      Serial.print(iDir);
#endif
      //前進
      if ((iDir - iDir_latch) > 0) {
        for (uint8_t i = 0; i < d ; i ++) {
          Keyboard.write(0xDA);//"↑"
#ifdef DEBUG
          Serial.println("↑");
#endif
        }
      }
      //後退
      if ((iDir - iDir_latch) < 0) {
        for (uint8_t i = 0; i < d ; i ++) {
          Keyboard.write(0xD9);//"↓"
#ifdef DEBUG
          Serial.println("↓");
#endif
        }
      }
      if (modeN) {
        if (iDir != 0) {
          Serial.println(strDir_N);
        }
      }
    }
  }

  notch_mc_latch = notch_mc;
  notch_mc_H_latch = notch_mc_H;
  notch_brk_latch = notch_brk;
  iDir_latch = iDir;
}

//ブレーキ角度調整
void read_Break_Setting(void) {
  uint16_t value = 0;
  bool n = (adcRead(5) < 512);
  bool eb = (adcRead(6) < 512);
  if (n) {
    if (setMode_N == 0) {
      adj_N = adcRead(0);
      setMode_N = 1;
      iniMillis_N = millis();
      Serial.print("POT_N=");
      Serial.print(EEPROM.get(0, value));
      Serial.print(" ADC=");
      Serial.println(adj_N);
    } else if (setMode_N == 1) {
      if (millis() - iniMillis_N > 3000) {
        setMode_N = 2;
        //Serial.println("Mode_1");
      }
      adj_N = adj_N * 0.9 + adcRead(0) * 0.1;
      //Serial.println("Mode_1");
    } else if (setMode_N == 2) {
      setMode_N = 0;
      POT_N = (uint16_t) adj_N;
      EEPROM.put(0, POT_N );
      Serial.print("NEW POT_N=");
      Serial.println(POT_N);
    }
  } else {
    setMode_N = 0;
  }

  if (eb) {
    if (setMode_EB == 0) {
      adj_EB = adcRead(0);
      setMode_EB = 1;
      iniMillis_EB = millis();
      Serial.print("POT_EB=");
      Serial.print(EEPROM.get(2, value));
      Serial.print(" ADC=");
      Serial.println(adj_EB);
    } else if (setMode_EB == 1) {
      if (millis() - iniMillis_EB > 3000) {
        setMode_EB = 2;
        //Serial.println("Mode_1");
      }
      adj_EB = adj_EB * 0.9 + adcRead(0) * 0.1;
      //Serial.println("Mode_1");
    } else if (setMode_EB == 2) {
      setMode_EB = 0;
      POT_EB = (uint16_t) adj_EB;
      EEPROM.put(2, POT_EB);
      Serial.print("NEW POT_EB=");
      Serial.println(POT_EB);
    }
  } else {
    setMode_EB = 0;
  }
}

void read_Horn(void) {
  Horn_1 = ~ioexp_1_AB & (1 << PIN_HORN_1);
  if ( Horn_1 != Horn_1_latch )
  {
    if (Horn_1) {
      if (modeBVE) {
        Keyboard.press(0xB0);//"Enter"
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xB0);
      }
    }
  }
  Horn_1_latch = Horn_1;

  Horn_2 = ~ioexp_1_AB & (1 << PIN_HORN_2);
  if ( Horn_2 != Horn_2_latch )
  {
    if (Horn_2 ) {
      if (modeBVE) {
        Keyboard.press(0xDF);//"Enter"
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xDF);
      }
    }
  }
  Horn_2_latch = Horn_2;
}

void read_Ats(void) {
  Ats_Cont = ~ioexp_1_AB & (1 << PIN_ATS_CONT);
  if ( Ats_Cont != Ats_Cont_latch )
  {
    if (Ats_Cont) {
      if (modeBVE) {
        if (adcRead(1) < 1) {
          Keyboard.press(0xD1);//"Insert"
        }
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xD1);
      }
    }
  }
  Ats_Cont_latch = Ats_Cont;

  Ats_Conf = ~ioexp_1_AB & (1 << PIN_ATS_CONF);
  if ( Ats_Conf != Ats_Conf_latch )
  {
    if (Ats_Conf ) {
      if (modeBVE) {
        if (adcRead(1) < 1) {
          Keyboard.press(0xD2);//"Home"
        }
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xD2);
      }
    }
  }
  Ats_Conf_latch = Ats_Conf;
}

void read_Panto(void) {

  Panto = ~ioexp_1_AB & (1 << PIN_PANTO);
  if ( Panto != Panto_latch )
  {
    if (Panto) {
      if (modeBVE) {
        Keyboard.press(0x82);//"Alt"
        Keyboard.press(0xC5);//"F4"
      }
    } else {
      if (modeBVE) {
        Keyboard.releaseAll();
      }
    }
  }
  Panto_latch = Panto;
}

void read_Light_Def(void) {
  if (!modeBVE) {
    Light_Def = ~ioexp_1_AB & (1 << PIN_LIGFT_DEF);
    Light_Def_latch = Light_Def;
  }
}

void read_modeBVE(void) {
  Light_On = ~ioexp_1_AB & (1 << PIN_LIGHT_ON);
  if ( Light_On != Light_On_latch )
  {
    modeBVE = Light_On;
    if (!Light_On) {
      notch_brk_num = 8;
    }
  }
  Light_On_latch = Light_On;
}
