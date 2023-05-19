//Arduino Micro または Leonard を使用してください

//Adafruit MCP23017 Arduino Library を導入してください
//Adafruit_MCP4725 Arduino Library を導入してください


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
//MC53_ME38_BVE_VM_V3.6.2 Arduino Pin9 に ATS復帰(内房線用)　"Homeキー" 追加
//MC53_ME38_BVE_VM_V3.6.3 他基板連動対応(Serial1送信対応)
//MC53_ME38_BVE_VM_V3.6.3.2 他基板からキーボードコマンド受付対応、ATS確認をSpacebar(0x20)に変更、Pin6をEBに、チャタリング防止機能テスト
//MC53_ME38_BVE_VM_V3.6.3.3 速度調整修正、常用最大位置を設定可能とする(デフォルト67°)
//MC53_ME38_BVE_VM_V3.6.3.4 速度計修正、微修正
//MC53_ME38_BVE_VM_V3.6.3.5 ブレーキ弁調整モード
//MC53_ME38_BVE_VM_V3.6.3.8 ブレーキ弁調整モード修正
//MC53_ME38_BVE_VM_V3.6.3.9 直通帯最小位置を設定可能とする(デフォルト3°)
//MC53_ME38_BVE_VM_V3.7 簡易自動帯再現
//MC53_ME38_BVE_VM_V4.0.0.0 コマンド番号化
//MC53_ME38_BVE_VM_V4.1.0.0 自動帯を追加
//MC53_ME38_BVE_VM_V4.1.0.1 速度計調整時速度計が動かないバグ修正

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

#define SS_Brk 4  //MCP3008_Brk
#define SS_Mc SS  //MCP23S17_MC

//↓デバッグのコメント(//)を解除するとシリアルモニタでデバッグできます
//#define DEBUG

Adafruit_MCP23X17 mcp;
Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac2;

SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE0);

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
uint8_t notch_brk = 0;  //ブレーキノッチ
uint8_t autoair_notch_brk = 0;
uint8_t notch_brk_latch = 0;
String notch_brk_name = "";
//以下ブレーキ設定値
uint16_t adc = 0;
uint16_t adc_latch = 0;
uint16_t brk_sap_angl = 80;         //直通帯の角度
uint8_t notch_brk_num = 8;          //常用ブレーキ段数
uint16_t brk_full_angl = 165;       //ブレーキ幅範囲
uint16_t brk_eb_angl = 150;         //緩め位置に対して非常位置の角度
uint16_t brk_sap_max_angl = 67;     //直通帯最大角度
uint16_t brk_sap_min_angl = 3;      //直通帯最小角度
uint16_t brk_keep_angl = 130;       //重なり全開位置
uint16_t brk_keep_full_angl = 135;  //重なり開始位置
uint16_t brk_bp_press = 490;        //BP管圧力
float brk_angl = 0;
float brk_angl_latch = 0;
//以上ブレーキ設定値
//以下速度計補正値
uint16_t spd_adj_010 = 150;   //12
uint16_t spd_adj_020 = 400;   //14
uint16_t spd_adj_030 = 680;   //16
uint16_t spd_adj_040 = 1010;  //18
uint16_t spd_adj_050 = 1330;  //20
uint16_t spd_adj_060 = 1650;  //22
uint16_t spd_adj_070 = 2000;  //24
uint16_t spd_adj_080 = 2340;  //26
uint16_t spd_adj_090 = 2680;  //28
uint16_t spd_adj_100 = 3020;  //30
uint16_t spd_adj_110 = 3340;  //32
uint16_t spd_adj_120 = 3650;  //34
uint16_t spd_adj_130 = 4000;  //36
uint16_t spd_adj_140 = 4095;  //38
uint16_t spd_adj_150 = 4095;  //40
uint16_t spd_adj_160 = 4095;  //42

uint16_t spd_limit = 120;  //44
//以上速度計補正値

int16_t bve_current = 0;

int16_t curr_limit = 750;

uint16_t vehicle_res = 500;

uint8_t chat_filter = 0;

//回生モード
bool curr_kaisei = true;  //true:有効　false:無効
//計器モード
bool curr_mode = true;  //true:電圧計 false:電流計

int8_t iDir = 0;
int8_t iDir_latch = 0;
String strDir = "";
String strDir_N = "  ";
bool Horn_1 = 0;
bool Horn_1_latch = 0;
bool Horn_2 = 0;
bool Horn_2_latch = 0;
bool Ats_Cont = 0;
bool Ats_Conf = 0;
bool Ats_Cont_latch = 0;
bool Ats_Conf_latch = 0;
bool Ats_Rec = 0;
bool Ats_Rec_latch = 0;
bool Panto = 0;
bool Panto_latch = 0;
bool Light_Def = 0;
bool Light_Def_latch = 0;
bool Light_On = 0;
bool Light_On_latch = 0;
bool Room_Light = 0;
bool Room_Light_latch = 0;
bool EB_SW = 0;
bool EB_SW_latch = 0;
float adj_N = 0.0;
float adj_EB = 0.0;

//以下ブレーキ位置調整用
uint16_t POT_N = 0;   //00
uint16_t POT_EB = 0;  //02
//以上ブレーキ位置調整用

unsigned long iniMillis_N = 0;
unsigned long iniMillis_EB = 0;
uint8_t setMode_N = 0;
uint8_t setMode_EB = 0;

bool mode_POT = false;

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
bool modeADJ = false;

//自動ブレーキ帯
unsigned long bp_millis = 0;
uint8_t bp_span = 20;
uint8_t autoair_notch_brk_latch = 0;

void setup() {
  pinMode(SS_Brk, OUTPUT);  //MCP3008
  pinMode(SS_Mc, OUTPUT);   //MCP23S17_MC

  pinMode(5, INPUT_PULLUP);  //EBスイッチ

  pinMode(8, OUTPUT);  //BVE_Door
  digitalWrite(8, 0);

  pinMode(9, INPUT_PULLUP);   //ATS
  pinMode(10, INPUT_PULLUP);  //予備
  pinMode(11, INPUT_PULLUP);  //予備SW1
  pinMode(12, INPUT_PULLUP);  //予備SW2

  Serial.begin(115200);
  Serial1.begin(115200);
  //Serial.setTimeout(10);
  //Serial1.setTimeout(10);
  dac.begin(0x60);
  dac2.begin(0x61);
  Keyboard.begin();

  if (!mcp.begin_SPI(SS_Mc)) {
    Serial.println("Error.");
  } else {
    // マスコンスイッチを全てプルアップ
    for (uint8_t i = 0; i < 16; i++) {
      mcp.pinMode(i, INPUT_PULLUP);
    }
  }

  //初回書き込みチェック
  bool b = 0;
  if (EEPROM.get(100, b) == 0) {
    EEPROM.put(0, 0);      //POT_N
    EEPROM.put(2, 512);    //POT_EB
    EEPROM.put(4, 8);      //ブレーキ段数設定
    EEPROM.put(6, 80);     //直通帯範囲
    EEPROM.put(8, 150);    //非常位置
    EEPROM.put(10, 165);   //ブレーキ最大角度
    EEPROM.put(12, 150);   //10km/h
    EEPROM.put(14, 400);   //20km/h
    EEPROM.put(16, 680);   //30km/h
    EEPROM.put(18, 1010);  //40km/h
    EEPROM.put(20, 1330);  //50km/h
    EEPROM.put(22, 1650);  //60km/h
    EEPROM.put(24, 2000);  //70km/h
    EEPROM.put(26, 2340);  //80km/h
    EEPROM.put(28, 2680);  //90km/h
    EEPROM.put(30, 3020);  //100km/h
    EEPROM.put(32, 3340);  //110km/h
    EEPROM.put(34, 3650);  //120km/h
    EEPROM.put(36, 4000);  //130km/h
    EEPROM.put(38, 4095);  //140km/h
    EEPROM.put(40, 4095);  //150km/h
    EEPROM.put(42, 4095);  //160km/h
    EEPROM.put(44, 120);   //速度計上限
    EEPROM.put(46, 1);     //回生モード
    EEPROM.put(48, 1);     //計器モード
    EEPROM.put(50, 750);   //電流値上限
    EEPROM.put(52, 500);   //列車抵抗
    EEPROM.put(54, 1);     //チャタリング
    EEPROM.put(56, 67);    //直通帯最大角度
    EEPROM.put(58, 3);     //直通帯最小角度
    EEPROM.put(60, 130);   //自動帯重なり開始位置
    EEPROM.put(62, 135);   //自動帯重なり全開位置
    //初回書き込みフラグセット
    EEPROM.put(100, b);
  } else {
    EEPROM.get(0, POT_N);
    EEPROM.get(2, POT_EB);
    EEPROM.get(4, notch_brk_num);
    EEPROM.get(6, brk_sap_angl);
    EEPROM.get(8, brk_eb_angl);
    EEPROM.get(10, brk_full_angl);
    EEPROM.get(12, spd_adj_010);
    EEPROM.get(14, spd_adj_020);
    EEPROM.get(16, spd_adj_030);
    EEPROM.get(18, spd_adj_040);
    EEPROM.get(20, spd_adj_050);
    EEPROM.get(22, spd_adj_060);
    EEPROM.get(24, spd_adj_070);
    EEPROM.get(26, spd_adj_080);
    EEPROM.get(28, spd_adj_090);
    EEPROM.get(30, spd_adj_100);
    EEPROM.get(32, spd_adj_110);
    EEPROM.get(34, spd_adj_120);
    EEPROM.get(36, spd_adj_130);
    EEPROM.get(38, spd_adj_140);
    EEPROM.get(40, spd_adj_150);
    EEPROM.get(42, spd_adj_160);
    EEPROM.get(44, spd_limit);
    EEPROM.get(46, curr_kaisei);
    EEPROM.get(48, curr_mode);
    EEPROM.get(50, curr_limit);
    EEPROM.get(52, vehicle_res);
    EEPROM.get(54, chat_filter);
    EEPROM.get(56, brk_sap_max_angl);
    EEPROM.get(58, brk_sap_min_angl);
    EEPROM.get(60, brk_keep_angl);
    EEPROM.get(62, brk_keep_full_angl);
  }

  //速度計テスト
  disp_SpeedMeter(spd_limit * 10, spd_limit);
  delay(1500);
  disp_SpeedMeter(0, spd_limit);
}

void loop() {
  //  Serial.println(bve_speed);
  // LOW = pressed, HIGH = not pressed
  //シリアルモニタが止まるのを防止するおまじない
  //BVEモードの時のみシリアル入力を受け付ける

  /*if (Serial1.available() && modeBVE) {
    String str1 = Serial1.readStringUntil('\r');
    if (str1 == "KEY:KEY_HOME") {
      Keyboard.write(0xD2);
    } else if (str1.startsWith("KEY:")) {
      Keyboard.print(str1.substring(4));
    }
    }*/
  if (Serial.available()) {
    strbve = Serial.readStringUntil('\r');
    uint8_t i = 0;
    String s = "";
    uint8_t device = strbve.substring(i + 4, i + 6).toInt();
    uint16_t num = strbve.substring(i + 7, i + 12).toInt();

    if (strbve.startsWith("WR ")) {
      //ブレーキ設定モード

      switch (device) {

        //ブレーキ段数設定
        case 4:
          if (num == 0 || num > 255) {
            s = "E1 004";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&notch_brk_num, true);
          }
          break;

        //直通帯範囲
        case 6:
          if (num == 0 || num > brk_eb_angl) {
            s = "E1 006";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_sap_angl, true);
          }
          break;

        //非常位置
        case 8:
          if (num == 0 || num > brk_full_angl) {
            s = "E1 008";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_eb_angl, true);
          }
          break;

        //ブレーキ最大角度
        case 10:
          if (num == 0 || num > 255) {
            s = "E1 010";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_full_angl, true);
          }
          break;

        //速度計調整
        case 12:
          s = rw_eeprom(device, &num, &spd_adj_010, true);
          bve_speed = 100;
          break;
        case 14:
          s = rw_eeprom(device, &num, &spd_adj_020, true);
          bve_speed = 200;
          break;
        case 16:
          s = rw_eeprom(device, &num, &spd_adj_030, true);
          bve_speed = 300;
          break;
        case 18:
          s = rw_eeprom(device, &num, &spd_adj_040, true);
          bve_speed = 400;
          break;
        case 20:
          s = rw_eeprom(device, &num, &spd_adj_050, true);
          bve_speed = 500;
          break;
        case 22:
          s = rw_eeprom(device, &num, &spd_adj_060, true);
          bve_speed = 600;
          break;
        case 24:
          s = rw_eeprom(device, &num, &spd_adj_070, true);
          bve_speed = 700;
          break;
        case 26:
          s = rw_eeprom(device, &num, &spd_adj_080, true);
          bve_speed = 800;
          break;
        case 28:
          s = rw_eeprom(device, &num, &spd_adj_090, true);
          bve_speed = 900;
          break;
        case 30:
          s = rw_eeprom(device, &num, &spd_adj_100, true);
          bve_speed = 1000;
          break;
        case 32:
          s = rw_eeprom(device, &num, &spd_adj_110, true);
          bve_speed = 1100;
          break;
        case 34:
          s = rw_eeprom(device, &num, &spd_adj_120, true);
          bve_speed = 1200;
          break;
        case 36:
          s = rw_eeprom(device, &num, &spd_adj_130, true);
          bve_speed = 1300;
          break;
        case 38:
          s = rw_eeprom(device, &num, &spd_adj_140, true);
          bve_speed = 1400;
          break;
        case 40:
          s = rw_eeprom(device, &num, &spd_adj_150, true);
          bve_speed = 1500;
          break;
        case 42:
          s = rw_eeprom(device, &num, &spd_adj_160, true);
          bve_speed = 1600;
          break;

        //最高速度設定
        case 44:
          if (num == 0) {
            s = "E1 044";
          } else {
            bve_speed = num * 10;
            s = rw_eeprom(device, &num, &spd_limit, true);
          }
          break;

        //回生モード
        case 46:
          if (num > 1) {
            s = "E1 046";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&curr_kaisei, true);
          }
          break;

        //計器モード
        case 48:
          if (num > 1) {
            s = "E1 048";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&curr_mode, true);
          }
          break;

        //列車抵抗
        case 52:
          if (num == 0) {
            s = "E1 052";
          } else {
            s = rw_eeprom(device, &num, &vehicle_res, true);
          }
          break;

        //チャタリング
        case 54:
          if (num < 0 || num > 3) {
            s = "E1 054";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&chat_filter, true);
          }

          //常用最大角度
        case 56:
          if (num == 0 || num > brk_sap_angl) {
            s = "E1 056";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_sap_max_angl, true);
          }
          break;

        //直通帯最小角度
        case 58:
          if (num == 0 || num > brk_sap_max_angl) {
            s = "E1 058";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_sap_min_angl, true);
          }
          break;
          //自動帯重なり位置
        case 60:
          if (num < brk_sap_max_angl || num > brk_eb_angl) {
            s = "E1 060";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_keep_angl, true);
          }
          break;

          //自動帯重なり開始位置
        case 62:
          if (num < brk_sap_max_angl || num > brk_eb_angl) {
            s = "E1 062";
          } else {
            s = rw_eeprom(device, &num, (uint16_t)&brk_keep_full_angl, true);
          }
          break;


        default:
          s = "E0";
          break;
      }
      Serial.println(s);

    } else if (strbve.startsWith("RD ")) {
      //ブレーキ設定読み出し
      if (strbve.indexOf("BRK") > 0) {
        Serial.print("SET READ:BRK_NUM=");
        Serial.println(notch_brk_num);
        Serial.print("SET READ:BRK_ANGL=");
        Serial.println(brk_sap_angl);
        Serial.print("SET READ:EB_ANGL=");
        Serial.println(brk_eb_angl);
        Serial.print("SET READ:BRK_FULL_ANGL=");
        Serial.println(brk_full_angl);
        Serial.print("SET READ:BRK_SAP_MAX_ANGL=");
        Serial.println(brk_sap_max_angl);
        Serial.print("SET READ:BRK_SAP_MIN_ANGL=");
        Serial.println(brk_sap_min_angl);
        Serial.print("SET READ:CHAT_FILTER=");
        Serial.println(chat_filter);
        Serial.print("SET READ:BRK_KEEP_ANGL=");
        Serial.println(brk_keep_angl);
        Serial.print("SET READ:BRK_KEEP_FULL_ANGL=");
        Serial.println(brk_keep_full_angl);

        //速度計設定読み出し
      } else if (strbve.indexOf("SPD") > 0) {
        Serial.println("SET READ: SPD");
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
      } else {
        uint8_t nnn = 0;
        Serial.println(rw_eeprom(device, 0, (uint16_t)&nnn, false));
      }

    } else if (strbve.startsWith("MD ")) {
      //模型モード
      if (strbve.indexOf("N   ") > 0) {
        s = "OK N   ";
        if (num) {
          modeN = true;
          notch_brk_num = 8;
          s += "ON";
        } else {
          modeN = false;
          s += "OFF";
        }
      }

      //ブレーキ弁調整モード
      else if (strbve.indexOf("ADJ ") > 0) {
        s = "OK ADJ ";
        if (num) {
          modeADJ = true;
          s += "ON";
        } else {
          modeADJ = false;
          s += "OFF";
        }
      }

      //ポテンショ読取モード
      else if (strbve.indexOf("POT ") > 0) {
        s = "OK POT ";
        if (num) {
          mode_POT = true;
          s += "ON";
        } else {
          mode_POT = false;
          s += "OFF";
        }
      } else {
        s = "E0";
      }
      Serial.println(s);

    } else {
      //通常モード：速度抽出
      bve_speed = strbve.substring(0, 4).toInt();
      bve_door = strbve.substring(5, 6).toInt();
      bve_current = strbve.substring(7, 12).toInt();
    }

    //電流計
    if (!curr_kaisei && (bve_current < 0)) {
      bve_current = 0;
    }
    if (curr_mode) {
      uint16_t v = 1500 - (bve_current * (vehicle_res / 1000.0));
      if (v > 2000) {
        v = 2000;
      }
    } else {
      int current = abs(bve_current);
      if (current > curr_limit) {
        current = curr_limit;
      }
    }

    //速度計表示補正
    disp_SpeedMeter(bve_speed, spd_limit);

    //電流計
    if (!curr_kaisei && (bve_current < 0)) {
      bve_current = 0;
    }
    if (curr_mode) {
      uint16_t v = 1500 - (bve_current * (vehicle_res / 1000.0));
      if (v > 2000) {
        v = 2000;
      }
      dac2.setVoltage(map(v, 0, 2000, 0, 4095), false);
    } else {
      int current = abs(bve_current);
      if (current > curr_limit) {
        current = curr_limit;
      }
      dac2.setVoltage(map(current, 0, curr_limit, 0, 4095), false);
    }

    //戸閉灯指示
    digitalWrite(8, !bve_door);

    //Serial1転送
    Serial1.print(strbve);
    Serial1.print("\r");

    /*2022-07-23 解除
      Serial1.print("BVE Speed: ");
      Serial1.println(bve_speed);
      Serial1.print(" Door: ");
      Serial1.println(bve_door);
    */
  }

  /*//読み残しは捨てる
    if (Serial.available() && modeBVE) {
    Serial.readStringUntil('\n');
    }*/
  read_IOexp();          //IOエキスパンダ読込ルーチン
  read_Light_Def();      //減光ライト読込ルーチン
  read_Light();          //前照灯読込ルーチン
  read_MC();             //マスコンノッチ読込ルーチン
  read_Dir();            //マスコンレバーサ読込ルーチン
  read_Break();          //ブレーキハンドル読込ルーチン
  read_Break_Setting();  //ブレーキハンドル読込ルーチン(未実装)
  read_Horn();           //ホーンペダル読込ルーチン
  read_Ats();            //ATS確認・警報持続読込ルーチン
  read_Panto();          //強制終了ルーチン
  read_EB();             //EBスイッチ読込ルーチン
  keyboard_control();    //キーボード(HID)アウトプットルーチン

  delay(10);


#ifdef DEBUG
  Serial.print(" ");
  Serial.print(notch_name);
  Serial.print(" ");
  Serial.print(notch_brk_name);
  Serial.print(" Dir: ");
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
    for (uint8_t i = 0; i < n; i++) {
      uint16_t temp_ioexp1 = mcp.readGPIOAB();
      if (temp_ioexp1 != 0) {
        if (temp_ioexp1_ini == temp_ioexp1) {
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
uint16_t adcRead(uint8_t ch) {  // 0 .. 7
  byte channelData = (ch + 8) << 4;
  // Serial.println(String(channelData,BIN));
  SPI.beginTransaction(settings);
  digitalWrite(SS_Brk, LOW);
  delayMicroseconds(100);
  SPI.transfer(0b00000001);                   // Start bit 1
  byte highByte = SPI.transfer(channelData);  // singleEnd
  byte lowByte = SPI.transfer(0x00);          // dummy
  delayMicroseconds(100);
  digitalWrite(SS_Brk, HIGH);
  SPI.endTransaction();
  return ((highByte & 0x03) << 8) + lowByte;
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
    Serial.print(" mc: ");
    Serial.print(notch_mc);
    Serial.print(" mc_latch: ");
    Serial.println(notch_mc_latch);
    Serial.print(" mc_H: ");
    Serial.print(notch_mc_H);
    Serial.print(" mc_H_latch: ");
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
  adc = adcRead(0);
  if (adc < POT_N) {
    adc = POT_N;
  } else if (adc > POT_EB) {
    adc = POT_EB;
  }
  brk_angl = map(adc, POT_N, POT_EB, 0, brk_full_angl * 100) / 100.0;

  if (mode_POT && !modeADJ) {
    Serial.print(" Pot1: ");
    Serial.print(10000 + adc);
    Serial.print(" Deg: ");
    Serial.print(1000 + brk_angl);
  }

  if (abs(brk_angl - brk_angl_latch) >= chat_filter) {
    //N位置
    if (brk_angl <= brk_sap_min_angl) {
      notch_brk = notch_brk_num + 1;
      notch_brk_name = "N ";

      //直通帯位置
    } else if (brk_angl < brk_sap_max_angl) {
      uint8_t temp_notch_brk = round(((float)brk_angl - (float)brk_sap_min_angl) / ((float)brk_sap_max_angl - (float)brk_sap_min_angl) * ((float)notch_brk_num - 1.0) + 0.5);
      notch_brk = notch_brk_num + 1 - temp_notch_brk;
      String s = String(temp_notch_brk);
      notch_brk_name = "B" + s;

      //常用最大位置～直通帯範囲まで
    } else if (brk_angl < brk_sap_angl) {
      notch_brk = 1;
      notch_brk_name = "B" + String(notch_brk_num);

      //自動帯
    } else if (brk_angl < brk_eb_angl) {
      notch_brk = notch_brk_num + 1;
      notch_brk_name = "N ";

      //非常位置以降
    } else {
      notch_brk = 0;
      notch_brk_name = "EB";
    }
    brk_angl_latch = brk_angl;
  }

  if (brk_angl < brk_keep_angl || brk_angl > brk_keep_full_angl) {
    bp_span = 20;
  } else if (brk_angl >= brk_keep_angl || brk_angl < brk_keep_full_angl) {
    bp_span = map(brk_angl, brk_keep_angl, brk_keep_full_angl, 100, 20);
  }
  BP(brk_angl);

  //自動帯圧力優先シーケンス
  //N位置
  if (brk_angl <= brk_sap_min_angl) {
    //自動ブレーキ
    if (autoair_notch_brk <= notch_brk_num + 1) {  //自動ブレーキ帯の段数が高いとき
      notch_brk = autoair_notch_brk;
    }
    //直通帯位置
  } else if (brk_angl < brk_sap_angl) {
    //自動ブレーキ
    if (notch_brk > autoair_notch_brk) {  //自動ブレーキ帯の段数が高いとき
      notch_brk = autoair_notch_brk;
    }

    //自動帯
  } else if (brk_angl < brk_eb_angl) {
    //自動ブレーキ
    if (notch_brk > autoair_notch_brk) {  //自動ブレーキ帯の段数が高いとき
      notch_brk = autoair_notch_brk;
    }
  }

  //ポテンショ生データ表示モード
  if (mode_POT && !modeADJ) {
    Serial.print(" Notch: ");
    Serial.print(notch_brk_name);
    Serial.print(" BP: ");
    Serial.print(brk_bp_press);
    Serial.print(" BP_notch: ");
    Serial.println(autoair_notch_brk);
  }
  //調整モード
  if (!mode_POT && modeADJ && adc != adc_latch) {
    Serial.print("ADC: ");
    Serial.print(10000 + adc);
    Serial.print(" DEG: ");
    Serial.print(1000 + brk_angl);
    Serial.print(" ");
    Serial.println(notch_brk_name);
  }
  adc_latch = adc;


#ifdef DEBUG
  bool sw = 0;
  if (adcRead(1) < 512) sw = 0;
  else sw = 1;
  Serial.print(" SW1: ");
  Serial.print(sw);

  if (adcRead(2) < 512) sw = 0;
  else sw = 1;
  Serial.print(" SW2: ");
  Serial.print(sw);

  if (adcRead(3) < 512) sw = 0;
  else sw = 1;
  Serial.print(" SW3: ");
  Serial.print(sw);

  if (adcRead(4) < 512) sw = 0;
  else sw = 1;
  Serial.print(" SW4: ");
  Serial.print(sw);
#endif
}

//キーボード(HID)出力
void keyboard_control(void) {
  //マスコンノッチが前回と異なるとき
  if (notch_mc != notch_mc_latch) {
    if (modeBVE) {
      uint8_t d = abs(notch_mc - notch_mc_latch);
#ifdef DEBUG
      Serial.print(" notch_mc: ");
      Serial.print(notch_mc);
      Serial.print(" notch_mc - notch_mc_latch: ");
      Serial.print(d);
      Serial.print(" Key: ");
#endif
      //力行ノッチ
      if (notch_mc >= 50 && notch_mc_latch >= 50 && notch_mc <= 55 && notch_mc_latch <= 55) {
        //進段
        if ((notch_mc - notch_mc_latch) > 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x5A);  //" / "
#ifdef DEBUG
            Serial.println("Z");
#endif
          }
        }
        //戻し
        if ((notch_mc - notch_mc_latch) < 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x41);  //" / "
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
  if (notch_mc_H != notch_mc_H_latch) {
    if (modeBVE) {
      uint8_t d = abs(notch_mc_H - notch_mc_H_latch);
      //抑速ノッチ
      if (notch_mc_H >= 100 && notch_mc_H_latch >= 100 && notch_mc_H <= 105 && notch_mc_H_latch <= 105) {
        //進段
        if ((notch_mc_H - notch_mc_H_latch) > 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x51);  //" / "
#ifdef DEBUG
            Serial.println("Q");
#endif
          }
        }
        //戻し
        if ((notch_mc_H - notch_mc_H_latch) < 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x41);  //" / "
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
    if (modeBVE && !modeADJ && !modeN) {
      uint8_t d = abs(notch_brk - notch_brk_latch);
#ifdef DEBUG
      Serial.print(" notch_brk: ");
      Serial.println(notch_brk);
      Serial.print(" notch_brk - notch_brk_latch: ");
      Serial.print(d);
      Serial.print(" Key: ");
#endif
      //ブレーキノッチ
      if (notch_brk <= (notch_brk_num + 1) && notch_brk_latch <= (notch_brk_num + 1) && notch_brk > 0) {
        //戻し
        if ((notch_brk - notch_brk_latch) > 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x2C);  //", "
#ifdef DEBUG
            Serial.println(", ");
#endif
          }
        }
        //ブレーキ
        if ((notch_brk - notch_brk_latch) < 0) {
          for (uint8_t i = 0; i < d; i++) {
            Keyboard.write(0x2E);  //"."
#ifdef DEBUG
            Serial.println(".");
#endif
          }
        }
      }
      if (notch_brk == 0) {
        Keyboard.write(0x2F);  //" / "
#ifdef DEBUG
        Serial.println(" / ");
#endif
      }
    }
    if (modeN) {
      Serial.println(notch_brk_name);
    }
  }

  //レバーサが前回と異なるとき
  if (iDir != iDir_latch) {
    if (modeBVE) {
      uint8_t d = abs(iDir - iDir_latch);
#ifdef DEBUG
      Serial.print(" iDir: ");
      Serial.print(iDir);
#endif
      //前進
      if ((iDir - iDir_latch) > 0) {
        for (uint8_t i = 0; i < d; i++) {
          Keyboard.write(0xDA);  //"↑"
#ifdef DEBUG
          Serial.println("↑");
#endif
        }
      }
      //後退
      if ((iDir - iDir_latch) < 0) {
        for (uint8_t i = 0; i < d; i++) {
          Keyboard.write(0xD9);  //"↓"
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
  autoair_notch_brk_latch = autoair_notch_brk;
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
      Serial.print("POT_N = ");
      Serial.print(EEPROM.get(0, value));
      Serial.print(" ADC = ");
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
      POT_N = (uint16_t)adj_N;
      EEPROM.put(0, POT_N);
      Serial.print("NEW POT_N = ");
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
      Serial.print("POT_EB = ");
      Serial.print(EEPROM.get(2, value));
      Serial.print(" ADC = ");
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
      POT_EB = (uint16_t)adj_EB;
      EEPROM.put(2, POT_EB);
      Serial.print("NEW POT_EB = ");
      Serial.println(POT_EB);
    }
  } else {
    setMode_EB = 0;
  }
}

void read_Horn(void) {
  Horn_1 = ~ioexp_1_AB & (1 << PIN_HORN_1);
  if (Horn_1 != Horn_1_latch) {
    if (Horn_1) {
      if (modeBVE) {
        Keyboard.press(0xB0);  //"Enter"
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xB0);
      }
    }
  }
  Horn_1_latch = Horn_1;

  Horn_2 = ~ioexp_1_AB & (1 << PIN_HORN_2);
  if (Horn_2 != Horn_2_latch) {
    if (Horn_2) {
      if (modeBVE) {
        Keyboard.press(0xDF);  //"Enter"
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
  //ATS警報持続
  Ats_Cont = ~ioexp_1_AB & (1 << PIN_ATS_CONT);
  if (Ats_Cont != Ats_Cont_latch) {
    if (Ats_Cont) {
      if (modeBVE) {
        if (adcRead(1) < 1) {
          Keyboard.press(0xD1);  //"Insert"
        }
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xD1);
      }
    }
  }
  Ats_Cont_latch = Ats_Cont;

  //ATS確認
  Ats_Conf = ~ioexp_1_AB & (1 << PIN_ATS_CONF);
  if (Ats_Conf != Ats_Conf_latch) {
    if (Ats_Conf) {
      if (modeBVE) {
        if (adcRead(1) < 1) {
          Keyboard.press(0x20);  //"Space"
        }
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0x20);
      }
    }
  }
  Ats_Conf_latch = Ats_Conf;

  //ATS復帰
  Ats_Rec = !digitalRead(9);
  if (Ats_Rec != Ats_Rec_latch) {
    if (Ats_Rec) {
      if (modeBVE) {
        //if (adcRead(1) < 1) {
        Keyboard.press(0xD2);  //"Home"
        //}
      }
    } else {
      if (modeBVE) {
        Keyboard.release(0xD2);
      }
    }
  }
  Ats_Rec_latch = Ats_Rec;
}

void read_Panto(void) {

  Panto = ~ioexp_1_AB & (1 << PIN_PANTO);
  if (Panto != Panto_latch) {
    if (Panto) {
      if (modeBVE) {
        Keyboard.press(0x82);  //"Alt"
        Keyboard.press(0xC5);  //"F4"
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
  /*  Light_Def = ~ioexp_1_AB & (1 << PIN_LIGFT_DEF);
    if ( Light_Def != Light_Def_latch )
    {
    modeBVE = Light_Def;
    if (!Light_Def) {
    notch_brk_num = 8;
    }
    }
    Light_Def_latch = Light_Def;*/
}

void read_Light(void) {
  /*Light_On = ~ioexp_1_AB & (1 << PIN_LIGHT_ON);
    if ( Light_On != Light_On_latch )
    {
    modeBVE = Light_On;
    if (!Light_On) {
    notch_brk_num = 8;
    }
    }
    Light_On_latch = Light_On;*/
}

void read_EB(void) {

  EB_SW = !digitalRead(5);
  if (EB_SW != EB_SW_latch) {
    if (EB_SW) {
      if (modeBVE) {
        Keyboard.press(KEY_DELETE);  //"Delete"
      }
    } else {
      if (modeBVE) {
        Keyboard.release(KEY_DELETE);  //"Delete"
      }
    }
  }
  EB_SW_latch = EB_SW;
}

void disp_SpeedMeter(uint16_t spd, uint16_t limit) {
  //速度計のリミットを適用
  if (spd > (limit * 10)) {
    spd = (limit * 10);
  }

  //速度計表示補正
  if (spd < 100) {
    spd = map(spd, 0, 100, 0, spd_adj_010);
  } else if (spd < 200) {
    spd = map(spd, 100, 200, spd_adj_010, spd_adj_020);
  } else if (spd < 300) {
    spd = map(spd, 200, 300, spd_adj_020, spd_adj_030);
  } else if (spd < 400) {
    spd = map(spd, 300, 400, spd_adj_030, spd_adj_040);
  } else if (spd < 500) {
    spd = map(spd, 400, 500, spd_adj_040, spd_adj_050);
  } else if (spd < 600) {
    spd = map(spd, 500, 600, spd_adj_050, spd_adj_060);
  } else if (spd < 700) {
    spd = map(spd, 600, 700, spd_adj_060, spd_adj_070);
  } else if (spd < 800) {
    spd = map(spd, 700, 800, spd_adj_070, spd_adj_080);
  } else if (spd < 900) {
    spd = map(spd, 800, 900, spd_adj_080, spd_adj_090);
  } else if (spd < 1000) {
    spd = map(spd, 900, 1000, spd_adj_090, spd_adj_100);
  } else if (spd < 1100) {
    spd = map(spd, 1000, 1100, spd_adj_100, spd_adj_110);
  } else if (spd < 1200) {
    spd = map(spd, 1100, 1200, spd_adj_110, spd_adj_120);
  } else if (spd < 1300) {
    spd = map(spd, 1200, 1300, spd_adj_120, spd_adj_130);
  } else if (spd < 1400) {
    spd = map(spd, 1300, 1400, spd_adj_130, spd_adj_140);
  } else if (spd < 1500) {
    spd = map(spd, 1400, 1500, spd_adj_140, spd_adj_150);
  } else if (spd < 1600) {
    spd = map(spd, 1500, 1600, spd_adj_150, spd_adj_160);
  } else {
    spd = map(spd, 1600, 1700, spd_adj_160, 4095);
  }
  dac.setVoltage(spd, false);
}

void BP(uint16_t angl) {
  //直通帯(運転位置)でBP圧を加圧
  if (angl < brk_sap_angl) {
    if ((millis() - bp_millis) > bp_span && brk_bp_press < 490) {
      brk_bp_press++;
      bp_millis = millis();
    }
    //重なり位置でBP圧は維持
  } else if (brk_angl < brk_keep_angl) {

    //常用位置でBP圧を減圧
  } else {
    if ((millis() - bp_millis) > bp_span && brk_bp_press > 0) {
      brk_bp_press--;
      bp_millis = millis();
    }
  }

  //自動帯でのBC圧をBP圧より生成
  uint16_t autoair_bc_press = (490 - brk_bp_press) * 2.5;
  if (autoair_bc_press > 440) {
    autoair_bc_press = 440;
  }

  //BC圧からブレーキノッチに変換
  autoair_notch_brk = map(autoair_bc_press, 0, 440, notch_brk_num + 1, 1);
}

String rw_eeprom(uint8_t dev, uint16_t *n, int *param, bool write) {
  if (write) {
    *param = *n;
    EEPROM.put(dev, *param);
  } else {
    EEPROM.get(dev, *param);
  }
  String s = "OK ";
  if (dev < 100) {
    s += "0";
  }
  if (dev < 10) {
    s += "0";
  }
  s += dev;
  s += " ";
  s += *param;

  return s;
}
