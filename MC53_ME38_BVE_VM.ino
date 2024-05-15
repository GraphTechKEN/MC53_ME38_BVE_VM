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
//MC53_ME38_BVE_VM_V4.1.0.2 BPの増減圧インターバルを追加
//MC53_ME38_BVE_VM_V4.1.0.3 自動帯の使用可否の選択機能を追加
//MC53_ME38_BVE_VM_V4.1.0.4 自動帯有効時、電制を無効とする
//MC53_ME38_BVE_VM_V4.1.0.5 自動帯有効時、レバーサをNとする
//MC53_ME38_BVE_VM_V4.1.0.6 自動帯有効時、マスコンノッチ投入でF/B対応、NでレバーサNとする
//MC53_ME38_BVE_VM_V4.1.0.7 個別読出追加、微修正、最大ノッチ指定追加
//MC53_ME38_BVE_VM_V4.1.0.8 最大ノッチ条件判定微修正
//MC53_ME38_BVE_VM_V4.1.0.9 EEPROM書き込み関数修正(int→uint16_t)、ATS_ConfとATS_Contを反転機能追加、
//MC53_ME38_BVE_VM_V4.1.0.10 小修正、他基板対応修正
//MC53_ME38_BVE_VM_V4.1.1.0 自動ブレーキ基板対応 ATS警報持続、確認ボタン基板転送対応


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

String strbve = "0000/1/ 00000/100000/0000000000000000000001";
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
String notch_brk_name_latch = "";
//以下ブレーキ設定値
uint16_t adc = 0;
uint16_t adc_latch = 0;
uint16_t notch_brk_num = 8;    //004 常用ブレーキ段数
uint16_t brk_sap_angl = 80;    //006 直通帯の角度
uint16_t brk_eb_angl = 150;    //008 非常位置
uint16_t brk_full_angl = 165;  //010 ブレーキ全体角度

uint16_t brk_sap_max_angl = 67;     //056 直通帯最大角度
uint16_t brk_sap_min_angl = 3;      //058 直通帯最小角度
uint16_t brk_keep_angl = 130;       //060 重なり全開位置
uint16_t brk_keep_full_angl = 135;  //062 重なり開始位置
uint16_t brk_bp_press = 490;        //BP管圧力
uint8_t brk_angl = 0;
uint8_t brk_angl_latch = 0;
//以上ブレーキ設定値
//以下速度計補正値
uint16_t spd_adj_010 = 150;   //012
uint16_t spd_adj_020 = 400;   //014
uint16_t spd_adj_030 = 680;   //016
uint16_t spd_adj_040 = 1010;  //018
uint16_t spd_adj_050 = 1330;  //020
uint16_t spd_adj_060 = 1650;  //022
uint16_t spd_adj_070 = 2000;  //024
uint16_t spd_adj_080 = 2340;  //026
uint16_t spd_adj_090 = 2680;  //028
uint16_t spd_adj_100 = 3020;  //030
uint16_t spd_adj_110 = 3340;  //032
uint16_t spd_adj_120 = 3650;  //034
uint16_t spd_adj_130 = 4000;  //036
uint16_t spd_adj_140 = 4095;  //038
uint16_t spd_adj_150 = 4095;  //040
uint16_t spd_adj_160 = 4095;  //042

uint16_t spd_limit = 120;  //044 速度上限
//以上速度計補正値

int16_t bve_current = 0;
uint16_t curr_kaisei = true;  //046 回生モード true:有効　false:無効
uint16_t curr_mode = true;    //048 計器モード true:電圧計 false:電流計
uint16_t curr_limit = 750;    //050 電流上限
uint16_t vehicle_res = 500;   //052 列車抵抗
uint16_t chat_filter = 0;     //054 チャタリングフィルタ[°]

int8_t iDir = 0;
int8_t iDir_latch = 0;
//String strDir = "  ";
//String strDir_N = "  ";
char cDir[2] = "  ";
char cDir_N[2] = "  ";
bool Horn_1 = 0;
bool Horn_1_latch = 0;
bool Horn_2 = 0;
bool Horn_2_latch = 0;
bool Ats_Cont = 0;
bool Ats_Conf = 0;
bool Ats_Cont_latch = 0;
bool Ats_Conf_latch = 0;
uint16_t Ats_Cont_flip = 0;  //074 警報持続ボタン反転 0:B接点 1以上:A接点
uint16_t Ats_Conf_flip = 0;  //076 ATS確認ボタン反転 0:B接点 1以上:A接点
bool Ats_Rec = 0;
bool Ats_Rec_latch = 0;
bool Ats_Pos = 0;
bool Ats_Pos_latch = 0;
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
uint16_t adj_N = 0;
uint16_t adj_EB = 0;

//以下ブレーキ位置調整用
uint16_t POT_N = 0;     //000
uint16_t POT_EB = 512;  //002
//以上ブレーキ位置調整用

unsigned long iniMillis_N = 0;
unsigned long iniMillis_EB = 0;
uint8_t setMode_N = 0;
uint8_t setMode_EB = 0;

bool mode_POT = false;

//運転モード
bool modeBVE = true;
bool modeN = false;
bool modeADJ = false;

//自動ブレーキ帯
unsigned long bp_millis = 0;
uint8_t bp_span = 20;
uint16_t bp_span_down = 20;  //064 自動帯減圧インターバル
uint16_t bp_span_up = 20;    //066 自動帯増圧インターバル
uint8_t autoair_notch_brk_latch = 0;
uint16_t autoair_use = true;    //068自動帯使用可否
bool autoair_dir_mask = false;  //自動帯使用時方向切替をマスク
uint16_t BC_press = 0;          //自動帯他基板より入力された値を格納
uint16_t RealAutoAir = 1;       //080 実際のエアー圧で自動帯再現

//マスコンノッチ
uint16_t notch_mc_num_max = 5;
uint16_t notch_mc_num = 5;


uint16_t Auto_Notch_Adjust = 1;  //078自動ノッチ合わせ機構

uint16_t AtsContactUse = 0;  //082 ATS接点判定使用
uint8_t Ats_In_Count_On = 0;
uint8_t Ats_In_Count_Off = 0;
uint16_t Evalve = true;  //084 E制御弁モード

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
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
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
  int16_t b = 0;
  EEPROM.get(100, b);
  if (b != 1) {
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
    EEPROM.put(64, 20);    //自動減圧インターバル
    EEPROM.put(66, 20);    //自動増圧インターバル
    EEPROM.put(68, 0);     //自動帯使用可否
    EEPROM.put(70, 5);     //マスコンノッチ最大数
    EEPROM.put(72, 5);     //マスコンノッチ数(車両)
    EEPROM.put(74, 0);     //警報持続反転
    EEPROM.put(76, 0);     //ATS確認反転
    EEPROM.put(78, 1);     //自動ノッチ合わせ
    EEPROM.put(80, 0);     //実際のエアー圧で自動帯再現
    EEPROM.put(82, 0);     //ATS接点情報を他基板へ伝送
    //初回書き込みフラグセット
    EEPROM.put(100, 1);
  } else {
    EEPROM.get(0, POT_N);
    EEPROM.get(2, POT_EB);
    EEPROM.get(4, notch_brk_num);  //ブレーキ段数
    EEPROM.get(6, brk_sap_angl);   //直通帯幅
    EEPROM.get(8, brk_eb_angl);    //非常位置
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
    EEPROM.get(62, brk_keep_full_angl);  //自動帯常用全開角度
    EEPROM.get(64, bp_span_down);        //自動帯減圧インターバル
    EEPROM.get(66, bp_span_up);          //自動帯増圧インターバル
    EEPROM.get(68, autoair_use);
    EEPROM.get(70, notch_mc_num_max);
    EEPROM.get(72, notch_mc_num);
    EEPROM.get(74, Ats_Cont_flip);      //警報持続ボタン反転 0:B接点 1以上:A接点
    EEPROM.get(76, Ats_Conf_flip);      //ATS確認ボタン反転 0:B接点 1以上:A接点
    EEPROM.get(78, Auto_Notch_Adjust);  //自動ノッチ合わせ
    EEPROM.get(80, RealAutoAir);        //実際のエアー圧で自動帯再現
    EEPROM.get(82, AtsContactUse);      //ATS接点情報を他基板へ伝送

    Ats_Cont = Ats_Cont_latch = Ats_Cont_flip;
    Ats_Conf = Ats_Conf_latch = Ats_Conf_flip;
  }

  //速度計テスト
  disp_SpeedMeter(spd_limit * 10, spd_limit);
  delay(1500);
  disp_SpeedMeter(0, spd_limit);
}

void loop() {
  if (Serial1.available() > 0) {
    String str1 = Serial1.readStringUntil('\r');
    if (str1.startsWith("BC ") && RealAutoAir) {
      BC_press = str1.substring(3, 6).toInt();
    } else {
      Serial.println(str1);
    }
  }
  if (Serial.available()) {
    strbve = Serial.readStringUntil('\r');
    uint8_t i = 0;
    String s = "";

    if (strbve.startsWith("WR ")) {
      //設定モード
      if (strbve.length() > 7) {
        uint8_t device = strbve.substring(3, 6).toInt();
        int16_t num = strbve.substring(7, 12).toInt();

        if (device < 100) {
          switch (device) {
            case 0:
              s = rw_eeprom(device, &num, &POT_N, true, num < 0 || num > 1023);
              break;
            case 2:
              s = rw_eeprom(device, &num, &POT_EB, true, num < 0 || num > 1023);
              break;

            //ブレーキ段数
            case 4:
              s = rw_eeprom(device, &num, (uint16_t)&notch_brk_num, true, num == 0 || num > 255);
              break;

            //直通帯幅
            case 6:
              s = rw_eeprom(device, &num, (uint16_t)&brk_sap_angl, true, num == 0 || num > brk_eb_angl);

              break;

            //非常位置
            case 8:
              s = rw_eeprom(device, &num, (uint16_t)&brk_eb_angl, true, num == 0 || num > brk_full_angl);
              break;

            //ブレーキ最大角度
            case 10:
              s = rw_eeprom(device, &num, (uint16_t)&brk_full_angl, true, num == 0 || num > 255);
              break;

            //速度計調整
            case 12:
              s = rw_eeprom(device, &num, &spd_adj_010, true, false);
              bve_speed = 100;
              break;
            case 14:
              s = rw_eeprom(device, &num, &spd_adj_020, true, false);
              bve_speed = 200;
              break;
            case 16:
              s = rw_eeprom(device, &num, &spd_adj_030, true, false);
              bve_speed = 300;
              break;
            case 18:
              s = rw_eeprom(device, &num, &spd_adj_040, true, false);
              bve_speed = 400;
              break;
            case 20:
              s = rw_eeprom(device, &num, &spd_adj_050, true, false);
              bve_speed = 500;
              break;
            case 22:
              s = rw_eeprom(device, &num, &spd_adj_060, true, false);
              bve_speed = 600;
              break;
            case 24:
              s = rw_eeprom(device, &num, &spd_adj_070, true, false);
              bve_speed = 700;
              break;
            case 26:
              s = rw_eeprom(device, &num, &spd_adj_080, true, false);
              bve_speed = 800;
              break;
            case 28:
              s = rw_eeprom(device, &num, &spd_adj_090, true, false);
              bve_speed = 900;
              break;
            case 30:
              s = rw_eeprom(device, &num, &spd_adj_100, true, false);
              bve_speed = 1000;
              break;
            case 32:
              s = rw_eeprom(device, &num, &spd_adj_110, true, false);
              bve_speed = 1100;
              break;
            case 34:
              s = rw_eeprom(device, &num, &spd_adj_120, true, false);
              bve_speed = 1200;
              break;
            case 36:
              s = rw_eeprom(device, &num, &spd_adj_130, true, false);
              bve_speed = 1300;
              break;
            case 38:
              s = rw_eeprom(device, &num, &spd_adj_140, true, false);
              bve_speed = 1400;
              break;
            case 40:
              s = rw_eeprom(device, &num, &spd_adj_150, true, false);
              bve_speed = 1500;
              break;
            case 42:
              s = rw_eeprom(device, &num, &spd_adj_160, true, false);
              bve_speed = 1600;
              break;

            //最高速度設定
            case 44:
              s = rw_eeprom(device, &num, &spd_limit, true, num == 0);
              break;

            //回生モード
            case 46:
              s = rw_eeprom(device, &num, (uint16_t)&curr_kaisei, true, num > 1);
              break;

            //計器モード
            case 48:
              s = rw_eeprom(device, &num, (uint16_t)&curr_mode, true, num > 1);
              break;

            //列車抵抗
            case 52:
              s = rw_eeprom(device, &num, &vehicle_res, true, num == 0);
              break;

            //チャタリング
            case 54:
              s = rw_eeprom(device, &num, (uint16_t)&chat_filter, true, num < 0 || num > 3);
              break;

            //常用最大角度
            case 56:
              s = rw_eeprom(device, &num, &brk_sap_max_angl, true, num == 0 || num > brk_sap_angl);
              break;

            //直通帯最小角度
            case 58:
              s = rw_eeprom(device, &num, &brk_sap_min_angl, true, num == 0 || num > brk_sap_max_angl);
              break;
            //自動帯常用開始角度
            case 60:
              s = rw_eeprom(device, &num, &brk_keep_angl, true, num < brk_sap_max_angl || num > brk_eb_angl);
              break;

            //自動帯常用全開角度
            case 62:

              s = rw_eeprom(device, &num, &brk_keep_full_angl, true, num < brk_sap_max_angl || num > brk_eb_angl);
              break;

            //自動帯減圧インターバル
            case 64:
              s = rw_eeprom(device, &num, &bp_span_down, true, num == 0 || num > 100);
              break;

            //自動帯増圧インターバル
            case 66:
              s = rw_eeprom(device, &num, &bp_span_up, true, num == 0 || num > 100);
              break;

            //自動帯使用可否
            case 68:
              s = rw_eeprom(device, &num, &autoair_use, true, num < 0 || num > 1);
              break;

            //マスコンノッチ最大数
            case 70:
              s = rw_eeprom(device, &num, &notch_mc_num_max, true, num < 0);
              break;

            //マスコンノッチ数(車両)
            case 72:
              s = rw_eeprom(device, &num, &notch_mc_num, true, num < 0);
              break;

              //警報持続ボタン反転 0:B接点 1以上:A接点
            case 74:
              s = rw_eeprom(device, &num, (uint16_t)&Ats_Cont_flip, true, num < 0 || num > 1);
              Ats_Cont_latch = Ats_Cont = Ats_Cont_flip;
              break;

              //ATS確認ボタン反転 0:B接点 1以上:A接点
            case 76:
              s = rw_eeprom(device, &num, (uint16_t)&Ats_Conf_flip, true, num < 0 || num > 1);
              Ats_Conf_latch = Ats_Conf = Ats_Conf_flip;
              break;

              //自動ノッチ合わせ
            case 78:
              s = rw_eeprom(device, &num, (uint16_t)&Auto_Notch_Adjust, true, num < 0 || num > 1);
              break;

              //実際のエアー圧で自動帯再現
            case 80:
              s = rw_eeprom(device, &num, (uint16_t)&RealAutoAir, true, num < 0 || num > 1);
              break;

              //ATS接点情報を他基板へ伝送
            case 82:
              s = rw_eeprom(device, &num, &AtsContactUse, true, num < 0 || num > 1);
              break;

            default:
              s = "E0";
              break;
          }
          Serial.println(s);
        }
      }

    } else if (strbve.startsWith("RD ")) {
      if (strbve.length() > 5) {
        uint8_t device = strbve.substring(3, 6).toInt();
        if (device < 100) {
          uint8_t nnn = 0;
          Serial.println(rw_eeprom(device, 0, (uint16_t)&nnn, false, false));
        }
      }

    } else if (strbve.startsWith("MD ")) {
      //模型モード
      if (strbve.indexOf("N   ") > 0) {
        s = "OK N   ";
        if (strbve.length() > 7) {
          int16_t num = strbve.substring(7, 12).toInt();
          if (num) {
            modeN = true;
            notch_brk_num = 8;
            s += "ON";
          } else {
            modeN = false;
            s += "OFF";
          }
        }
      }

      //ブレーキ弁調整モード
      else if (strbve.indexOf("ADJ ") > 0) {
        s = "OK ADJ ";
        if (strbve.length() > 7) {
          int16_t num = strbve.substring(7, 12).toInt();

          if (num) {
            modeADJ = true;
            s += "ON";
          } else {
            modeADJ = false;
            s += "OFF";
          }
        }
      }

      //ポテンショ読取モード
      else if (strbve.indexOf("POT ") > 0) {
        s = "OK POT ";
        if (strbve.length() > 7) {
          int16_t num = strbve.substring(7, 12).toInt();
          if (num) {
            mode_POT = true;
            s += "ON";
          } else {
            mode_POT = false;
            s += "OFF";
          }
        }
      } else {
        s = "E0";
      }
      Serial.println(s);

    } else {
      //通常モード：速度抽出
      if (strbve.length() > 11) {
        bve_speed = strbve.substring(0, 4).toInt();
        bve_door = strbve.charAt(5) == '1';
        bve_current = strbve.substring(7, 12).toInt();

        //自動ノッチ合わせ機構
        if (strbve.length() > 49) {
          if (Auto_Notch_Adjust) {
            if (strbve.charAt(47) == 'B') {
              int bve_rev = 0;
              if (strbve.charAt(44) == 'F') {
                bve_rev = 1;
              } else if (strbve.charAt(44) == 'B') {
                bve_rev = -1;
              }
              String bve_brk = strbve.substring(48, 50);
              char Buf[4];
              bve_brk.toCharArray(Buf, 4);
              uint8_t num = strtol(Buf, NULL, 16);  //16進数→10進数に変換
              if (!autoair_dir_mask) {
                iDir_latch = bve_rev;
                notch_brk_latch = (notch_brk_num + 1) - num;
              }
            }
          }
        }
      }
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
    //自動帯有効時、電制を無効とする
    if (autoair_use && brk_angl > brk_sap_angl) {
      if (strbve.length() > 18) {
        strbve.setCharAt(18, '0');
      }
    }
    if (Ats_Pos && AtsContactUse) {
      if (strbve.length() > 20) {
        strbve.setCharAt(20, '1');
      }
    }
    if (Ats_Cont) {
      if (strbve.length() > 13) {
        strbve.setCharAt(13, '1');
      }
    }
    Serial1.print(strbve);
    Serial1.print('\r');
  }

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
        if ((notch_mc_num_max == 5) && (notch_mc_num == 4)) {
          notch_mc = 54;
          notch_name = "P4";
        } else if ((notch_mc_num_max == 5) && (notch_mc_num == 5)) {
          notch_mc = 55;
          notch_name = "P5";
        }
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB & (1 << PIN_MC_4)) {
        if ((notch_mc_num_max == 4) && (notch_mc_num == 5)) {
          notch_mc = 55;
          notch_name = "P5";
        } else if (((notch_mc_num_max == 5) && (notch_mc_num == 5)) || ((notch_mc_num_max == 4) && (notch_mc_num == 4))) {
          notch_mc = 54;
          notch_name = "P4";
        }
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB & (1 << PIN_MC_3)) {
        notch_mc = 53;
        notch_name = "P3";
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB & (1 << PIN_MC_2)) {
        notch_mc = 52;
        notch_name = "P2";
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB & (1 << PIN_MC_1)) {
        notch_mc = 51;
        notch_name = "P1";
        autoair_dir_mask = false;
      } else {
        notch_mc = 50;
        notch_mc_H = 100;
        notch_name = "N ";
        if (brk_angl > brk_sap_angl && brk_angl < brk_eb_angl && autoair_use) {
          autoair_dir_mask = true;
          iDir = 0;
          //strDir = "N ";
          cDir[0] = 'N';
        }
      }
    } else {
      if (~ioexp_1_AB & (1 << PIN_MC_5) && !autoair_dir_mask) {
        notch_mc_H = 101;
        notch_name = "H1";
      } else if (~ioexp_1_AB & (1 << PIN_MC_3) && ioexp_1_AB & (1 << PIN_MC_4) && !autoair_dir_mask) {
        notch_mc_H = 102;
        notch_name = "H2";
      } else if (~ioexp_1_AB & (1 << PIN_MC_2) && ioexp_1_AB & (1 << PIN_MC_4) && !autoair_dir_mask) {
        notch_mc_H = 103;
        notch_name = "H3";
      } else if (~ioexp_1_AB & (1 << PIN_MC_2) && ~ioexp_1_AB & (1 << PIN_MC_4) && !autoair_dir_mask) {
        notch_mc_H = 104;
        notch_name = "H4";
      } else if (~ioexp_1_AB & (1 << PIN_MC_3) && ~ioexp_1_AB & (1 << PIN_MC_4) && !autoair_dir_mask) {
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
  if (!autoair_dir_mask) {
    if (~ioexp_1_AB & (1 << PIN_MC_DIR_F)) {
      iDir = 1;
      //strDir = "F ";
      cDir[0] = 'F';
      //strDir_N = "L ";
      cDir_N[0] = 'L';
    } else if (~ioexp_1_AB & (1 << PIN_MC_DIR_B)) {
      iDir = -1;
      //strDir = "B ";
      cDir[0] = 'B';
      //strDir_N = "R ";
      cDir_N[0] = 'R';
    } else {
      iDir = 0;
      //strDir = "N ";
      cDir[0] = 'N';
    }
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
      autoair_dir_mask = false;

      //直通帯位置
    } else if (brk_angl < brk_sap_max_angl) {
      uint16_t temp_notch_brk = round((float)(brk_angl - brk_sap_min_angl) / (float)(brk_sap_max_angl - brk_sap_min_angl) * (notch_brk_num - 1) + 0.5);

      notch_brk = notch_brk_num + 1 - temp_notch_brk;
      String s = String(temp_notch_brk);
      notch_brk_name = "B" + s;
      autoair_dir_mask = false;

      //常用最大位置～直通帯範囲まで
    } else if (brk_angl < brk_sap_angl) {
      notch_brk = 1;
      notch_brk_name = "B" + String(notch_brk_num);
      autoair_dir_mask = false;

      //自動帯
    } else if (brk_angl < brk_keep_angl) {
      if (autoair_use) {
        notch_brk = notch_brk_num + 1;
        notch_brk_name = "N ";
        if (notch_mc == 50 && notch_mc_H == 100) {
          autoair_dir_mask = true;
          iDir = 0;
          //strDir = "N ";
          cDir[0] = 'N';
        } else {
          autoair_dir_mask = false;
        }

      } else {
        notch_brk = 1;
        notch_brk_name = "B" + String(notch_brk_num);
        autoair_dir_mask = false;
      }

    } else if (brk_angl < brk_keep_full_angl) {
      if (autoair_use) {
        notch_brk_name = "A1";
        if (notch_mc == 50 && notch_mc_H == 100) {
          autoair_dir_mask = true;
          iDir = 0;
          //strDir = "N ";
          cDir[0] = 'N';
        } else {
          autoair_dir_mask = false;
        }
      } else {
        notch_brk = 1;
        notch_brk_name = "B" + String(notch_brk_num);
        autoair_dir_mask = false;
      }

    } else if (brk_angl < brk_eb_angl) {
      if (autoair_use) {
        notch_brk_name = "A2";
        if (notch_mc == 50 && notch_mc_H == 100) {
          autoair_dir_mask = true;
          iDir = 0;
          //strDir = "N ";
          cDir[0] = 'N';
        } else {
          autoair_dir_mask = false;
        }
      } else {
        notch_brk = 1;
        notch_brk_name = "B" + String(notch_brk_num);
        autoair_dir_mask = false;
      }

      //非常位置以降
    } else {
      notch_brk = 0;
      notch_brk_name = "EB";
      autoair_dir_mask = false;
    }
    brk_angl_latch = brk_angl;
  }

  if (autoair_use) {
    BP(brk_angl);
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

        for (uint8_t i = 0; i < d; i++) {
          //進段
          if ((notch_mc - notch_mc_latch) > 0) {
            Keyboard.write(0x5A);  //" / "
            //Serial.println("Z");
            if (notch_mc == 53) {
              Serial1.print("ATSM3");
              Serial1.print('\r');
            }
            if (notch_mc == 51) {
              Serial1.print("ATSM1");
              Serial1.print('\r');
            }
          } else {
            Keyboard.write(0x41);  //" / "
            //Serial.println("A");
            if (notch_mc == 50) {
              Serial1.print("ATSM0");
              Serial1.print('\r');
            }
            if (notch_mc == 52) {
              Serial1.print("ATSM2");
              Serial1.print('\r');
            }
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
        for (uint8_t i = 0; i < d; i++) {
          if ((notch_mc_H - notch_mc_H_latch) > 0) {
            Keyboard.write(0x51);  //" / "
            //Serial.println("Q");
            //戻し
          } else {
            Keyboard.write(0x41);  //" / "
            //Serial.println("A");
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
  if (notch_brk != notch_brk_latch || notch_brk_name != notch_brk_name_latch) {
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
        for (uint8_t i = 0; i < d; i++) {
          //戻し
          if ((notch_brk - notch_brk_latch) > 0) {
            Keyboard.write(0x2C);  //", "
#ifdef DEBUG
            Serial.println(", ");
#endif
          }
        }
        //ブレーキ
        if ((notch_brk - notch_brk_latch) < 0) {
          Keyboard.write(0x2E);  //"."
#ifdef DEBUG
          Serial.println(".");
#endif
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
      for (uint8_t i = 0; i < d; i++) {
        //前進
        if ((iDir - iDir_latch) > 0) {
          Keyboard.write(0xDA);  //"↑"
          //Serial.println("↑");
          //後進
        } else {
          Keyboard.write(0xD9);  //"↓"
          //Serial.println("↓");
        }
      }
    }
    if (modeN) {
      if (iDir != 0) {
        //Serial.println(strDir_N);
        Serial.print(cDir_N[0]);
        Serial.println(cDir_N[1]);
      }
    }
  }

  notch_mc_latch = notch_mc;
  notch_mc_H_latch = notch_mc_H;
  notch_brk_latch = notch_brk;
  notch_brk_name_latch = notch_brk_name;
  autoair_notch_brk_latch = autoair_notch_brk;
  iDir_latch = iDir;
}

//ブレーキ角度調整
void read_Break_Setting(void) {
  String s = "";
  uint16_t value = 0;
  bool btn_n = (adcRead(5) < 512);
  bool btn_eb = (adcRead(6) < 512);
  if (btn_n) {
    if (setMode_N == 0) {
      adj_N = adcRead(0);
      setMode_N = 1;
      iniMillis_N = millis();
      s = "POT_N = ";
      s += EEPROM.get(0, value);
      s += " ADC = ";
      s += adj_N;
    } else if (setMode_N == 1) {
      if (millis() - iniMillis_N > 3000) {
        setMode_N = 2;
      }
      adj_N = (adj_N * 9 + adcRead(0)) * 0.1;
    } else if (setMode_N == 2) {
      setMode_N = 0;
      POT_N = adj_N;
      EEPROM.put(0, POT_N);
      s = "NEW POT_N = ";
      s += POT_N;
    }
  } else {
    setMode_N = 0;
  }

  if (btn_eb) {
    if (setMode_EB == 0) {
      adj_EB = adcRead(0);
      setMode_EB = 1;
      iniMillis_EB = millis();
      s = "POT_EB = ";
      s += EEPROM.get(2, value);
      s += " ADC = ";
      s += adj_EB;
    } else if (setMode_EB == 1) {
      if (millis() - iniMillis_EB > 3000) {
        setMode_EB = 2;
        //Serial.println("Mode_1");
      }
      adj_EB = (adj_EB * 9 + adcRead(0)) * 0.1;
      //Serial.println("Mode_1");
    } else if (setMode_EB == 2) {
      setMode_EB = 0;
      POT_EB = adj_EB;
      EEPROM.put(2, POT_EB);
      s = "NEW POT_EB = ";
      s += POT_EB;
    }
  } else {
    setMode_EB = 0;
  }
  if (s != "") {
    Serial.println(s);
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

  //ATS誤作動防止
  bool Ats_In = (adcRead(1) < 1);
  if (Ats_In) {
    Ats_In_Count_On++;
    Ats_In_Count_Off = 0;
  } else {
    Ats_In_Count_Off++;
    Ats_In_Count_On = 0;
  }
  if (Ats_In_Count_On > 10) {
    Ats_Pos = true;
  } else if (Ats_In_Count_Off > 10) {
    Ats_Pos = false;
  }

  if (AtsContactUse) {
    if (Ats_Pos && !Ats_Pos_latch) {
      Serial1.print("ATS 1");
      Serial1.print("\r");
      Serial.println("ATS 1 ON");
    } else if (!Ats_Pos && Ats_Pos_latch) {
      Serial1.print("ATS 0");
      Serial1.print("\r");
      Serial.println("ATS 0 OFF");
    }
  }
  Ats_Pos_latch = Ats_Pos;

  //ATS警報持続
  if (Ats_Cont_flip) {
    Ats_Cont = ioexp_1_AB & (1 << PIN_ATS_CONT);
  } else {
    Ats_Cont = ~ioexp_1_AB & (1 << PIN_ATS_CONT);
  }
  if (Ats_Cont != Ats_Cont_latch) {
    if (Ats_Cont) {
      if (modeBVE) {
        //ATS警報持続ボタン押下時、点灯
        /*strbve.setCharAt(13, '1');
        Send_Serial1(strbve);*/
        Serial1.print("ACT 1");
        Serial1.print("\r");
        Serial.println("ACT 1 ON");

        Keyboard.press(0xD1);  //"Insert"
      }
    } else {
      if (modeBVE) {
        //ATS警報持続ボタン、消灯
        /*strbve.setCharAt(13, '/');
        Send_Serial1(strbve);*/
        Serial1.print("ACT 0");
        Serial1.print("\r");
        Serial.println("ACT 0 ON");

        Keyboard.release(0xD1);
      }
    }
  }
  Ats_Cont_latch = Ats_Cont;

  //ATS確認
  if (Ats_Conf_flip) {
    Ats_Conf = ioexp_1_AB & (1 << PIN_ATS_CONF);
  } else {
    Ats_Conf = ~ioexp_1_AB & (1 << PIN_ATS_CONF);
  }

  if (Ats_Conf != Ats_Conf_latch) {
    if (Ats_Conf) {
      if (modeBVE) {
        if (Ats_Pos) {
          //ATS確認ボタン押下時、消灯
          /*strbve.setCharAt(14, '0');
            Send_Serial1(strbve);*/
          Serial1.print("ACF 1");
          Serial1.print("\r");
          Serial.println("ACF 1 ON");

          Keyboard.press(0x20);  //"Space"
        }
      }
    } else {
      if (modeBVE) {
        //ATS確認ボタン、点灯
        /*strbve.setCharAt(14, '1');
          Send_Serial1(strbve);*/
        Serial1.print("ACF 0");
        Serial1.print("\r");
        Serial.println("ACF 0 OFF");

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
        Keyboard.press(0xD2);  //"Home"
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

  if (!RealAutoAir) {
    //BPの増減圧インターバルを設定
    if (angl < brk_keep_angl) {
      bp_span = bp_span_up;
    } else if ((angl >= brk_keep_angl) && (angl <= brk_keep_full_angl)) {
      bp_span = map(angl, brk_keep_angl, brk_keep_full_angl, bp_span_down, bp_span_down);
    } else if (angl > brk_keep_full_angl) {
      bp_span = bp_span_down;
    }

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
    BC_press = (490 - brk_bp_press) * 2.5;
    if (BC_press > 440) {
      BC_press = 440;
    }
  }

  //BC圧からブレーキノッチに変換
  autoair_notch_brk = map(BC_press, 0, 400, notch_brk_num + 1, 1);

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
}

String rw_eeprom(uint16_t dev, uint16_t *n, uint16_t *param, bool write, bool NGcondition) {
  String s = "OK ";
  if (!NGcondition) {
    if (write) {
      *param = *n;
      EEPROM.put(dev, *param);
    } else {
      EEPROM.get(dev, *param);
    }

    if (dev < 100) {
      s += "0";
    }
    if (dev < 10) {
      s += "0";
    }
    s += dev;
    s += " ";
    s += *param;
  } else {
    s = "E1 " + dev;
  }
  return s;
}

void Send_Serial1(String _str) {
  if (_str.charAt(4) == '/') {
    Serial1.print(_str);
    Serial1.print("\r");
  }
}