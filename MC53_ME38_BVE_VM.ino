//Arduino Micro または Leonard を使用してください

//Adafruit MCP23017 Arduino Library を導入してください。
//Adafruit_MCP4725 Arduino Library は使用しません
//導入方法：ライブラリマネージャーから上記を検索してインストールします


//簡単な説明
//コマンドに対応しました。デリミタ:CR、Baud:115200、DataBits:8、StopBit:1
//マスコン4段or5段、ブレーキX段+EBです。 ブレーキ段数は"WR 004 X(CR)"で段数(X)指定できます。
//ノッチの移動量に応じて、各キーボードコマンドを打ち込んでいることになりますので、BVEのキーアサイン設定はデフォルトに戻してください。
//マスコン側はMC53またはMC37の真理値に基づいてノッチ(N,P1～P(4)5)を指示します。レバーサも対応します。
//ブレーキ側はポテンショの値を角度換算し、ブレーキノッチ(N,B1～EB)を指示します。

//BVEゲーム開始時は、一旦ブレーキハンドルをN→常用最大(or EB)、マスコンノッチはN、レバーサハンドルをB→N→Fと動かす等してリセットします。
//※AtsEXを導入し、最新プラグインを適用すると上記を自動で調整します。

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
//MC53_ME38_BVE_VM_V4.1.1.3 自動ノッチ合わせ機構対応
//MC53_ME38_BVE_VM_V4.1.1.5 速度計補正配列化
//MC53_ME38_BVE_VM_V4.1.1.6 速度計補正最適化
//MC53_ME38_BVE_VM_V4.1.1.7 ブレーキノッチ逆転
//MC53_ME38_BVE_VM_V4.1.1.8_simple MCP3008とMCP4725を使用しない定義を追加
//MC53_ME38_BVE_VM_V4.1.1.9 微修正
//MC53_ME38_BVE_VM_V4.1.2.1 警報持続スイッチ反転を他スイッチに拡張(ATS確認を除く)、抑速-非常接点切替対応、ME38自動帯非常抜取対応

/*input_flip
  1bit:警報持続
  (2bit:ATS確認ボタン)
  (3bit:ATS復帰ボタン未実装) 
  4bit:EB
  5bit:警笛1
  6bit:警笛2
  7bit:抑速 0:抑速 1:非常
  */

#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP4725.h>
#include <EEPROM.h>
#include <Wire.h>

#include <Keyboard.h>

//#define SIMPLE MCP3008およびMCP4725を使用しない定義用

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

#ifndef SIMPLE
#define SS_Brk 4  //MCP3008_Brk
#define SS_Mc SS  //MCP23S17_MC
#endif

//↓デバッグのコメント(//)を解除するとシリアルモニタでデバッグできます
//#define DEBUG

Adafruit_MCP23X17 mcp;
#ifndef SIMPLE
Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac2;
#endif

SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE0);

uint16_t ioexp_1_AB = 0;
uint16_t bve_speed = 0;
uint8_t notch_mc = 0;              //マスコンノッチ
uint8_t notch_mc_latch = 0;        //マスコンノッチ格納
uint8_t notch_mc_H = 0;            //抑速ノッチ
uint8_t notch_mc_H_latch = 0;      //抑速ノッチ格納
uint8_t mc_DEC_latch = 0;          //低速制御
String notch_name = "";            //マスコンノッチ名称
uint8_t notch_brk = 0;             //ブレーキノッチ
uint8_t notch_brk_latch = 0;       //ブレーキノッチ格納
String notch_brk_name = "";        //ブレーキノッチ名称
String notch_brk_name_latch = "";  //ブレーキノッチ名称格納
//以下ブレーキ設定値

uint16_t notch_brk_num = 8;         //004 常用ブレーキ段数
uint16_t brk_sap_angl = 80;         //006 直通帯の角度
uint16_t brk_eb_angl = 150;         //008 非常位置
uint16_t brk_full_angl = 165;       //010 ブレーキ全体角度
uint16_t brk_sap_max_angl = 67;     //056 直通帯最大角度
uint16_t brk_sap_min_angl = 3;      //058 直通帯最小角度
uint16_t brk_keep_angl = 130;       //060 重なり全開位置
uint16_t brk_keep_full_angl = 135;  //062 重なり開始位置
uint16_t brk_bp_press = 490;        //BP管圧力
uint8_t brk_angl = 0;               //ブレーキ弁角度
uint8_t brk_angl_latch = 0;         //ブレーキ弁角度格納
//以上ブレーキ設定値

//以下速度計補正値
uint16_t spd_adj[18] = { 0, 150, 400, 680, 1010, 1330, 1650, 2000, 2340, 2680, 3020, 3340, 3650, 4000, 4095, 4095, 4095, 4095 };
uint16_t spd_limit = 120;  //044 速度上限
//以上速度計補正値


uint16_t curr_kaisei = true;  //046 回生モード true:有効　false:無効
uint16_t curr_mode = true;    //048 計器モード true:電圧計 false:電流計
uint16_t curr_limit = 750;    //050 電流上限
uint16_t vehicle_res = 500;   //052 列車抵抗
uint16_t chat_filter = 0;     //054 チャタリングフィルタ[°]

int8_t iDir = 0;
int8_t iDir_latch = 0;
char cDir[2] = "  ";
char cDir_N[2] = "  ";

//ATS
bool Ats_Pos = 0;              //ATS確認位置
bool Ats_Pos_latch = 0;        //ATS確認位置
uint8_t Ats_In_Count_On = 0;   //ATS確認位置チャタリング防止用
uint8_t Ats_In_Count_Off = 0;  //ATS確認位置チャタリング防止用
uint16_t input_flip = 0;       //074 ボタン反転(旧警報持続ボタン反転 0:B接点 1以上:A接点)0:B接点 1以上:A接点
uint16_t Ats_Conf_flip = 0;    //076 ATS確認ボタン反転 0:B接点 1以上:A接点
uint16_t AtsContactUse = 0;    //082 ATS接点判定使用
//ATS

//以下ブレーキ位置調整用
uint16_t POT_N = 0;     //000
uint16_t POT_EB = 512;  //002
//以上ブレーキ位置調整用

bool mode_POT = false;

//運転モード
bool modeBVE = true;
bool modeN = false;
bool modeADJ = false;

//自動ブレーキ帯
unsigned long bp_millis = 0;
uint8_t bp_span = 20;
uint8_t autoair_notch_brk = 0;
uint8_t autoair_notch_brk_latch = 0;
uint16_t bp_span_down = 20;      //064 自動帯減圧インターバル
uint16_t bp_span_up = 20;        //066 自動帯増圧インターバル
uint16_t autoair_use = true;     //068自動帯使用可否
bool autoair_dir_mask = false;   //自動帯使用時方向切替をマスク
uint16_t BC_press = 0;           //自動帯他基板より入力された値を格納
uint16_t RealAutoAir = 1;        //080 実際のエアー圧で自動帯再現
uint16_t notch_mc_num_max = 5;   //070マスコンノッチ最大数
uint16_t notch_mc_num = 5;       //072マスコンノッチ数(車両)
uint16_t Auto_Notch_Adjust = 1;  //078自動ノッチ合わせ機構


void setup() {
  pinMode(SS_Brk, OUTPUT);   //MCP3008
  pinMode(SS_Mc, OUTPUT);    //MCP23S17_MC
  pinMode(5, INPUT_PULLUP);  //EBスイッチ
  pinMode(8, OUTPUT);        //BVE_Door
  digitalWrite(8, 0);

#ifdef SIMPLE
  pinMode(A0, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
#endif

  pinMode(9, INPUT_PULLUP);   //ATS
  pinMode(10, INPUT_PULLUP);  //予備
  pinMode(11, INPUT_PULLUP);  //予備SW1
  pinMode(12, INPUT_PULLUP);  //予備SW2

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
#ifndef SIMPLE
  dac.begin(0x60);
  dac2.begin(0x61);
#endif

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
    EEPROM.put(0, POT_N);                    //POT_N
    EEPROM.put(2, POT_EB);                   //POT_EB
    EEPROM.put(4, notch_brk_num);            //ブレーキ段数設定
    EEPROM.put(6, brk_sap_angl);             //直通帯範囲
    EEPROM.put(8, brk_eb_angl);              //非常位置
    EEPROM.put(10, brk_full_angl);           //ブレーキ最大角度
    for (int i = 1; i <= 16; i++) {          //速度計補正値
      EEPROM.put((i * 2) + 10, spd_adj[i]);  //
    }                                        //
    EEPROM.put(44, spd_limit);               //速度計上限
    EEPROM.put(46, curr_kaisei);             //回生モード
    EEPROM.put(48, curr_mode);               //計器モード
    EEPROM.put(50, curr_limit);              //電流値上限
    EEPROM.put(52, vehicle_res);             //列車抵抗
    EEPROM.put(54, chat_filter);             //チャタリング
    EEPROM.put(56, brk_sap_max_angl);        //直通帯最大角度
    EEPROM.put(58, brk_sap_min_angl);        //直通帯最小角度
    EEPROM.put(60, brk_keep_angl);           //自動帯重なり開始位置
    EEPROM.put(62, brk_keep_full_angl);      //自動帯常用全開角度
    EEPROM.put(64, bp_span_down);            //自動帯減圧インターバル
    EEPROM.put(66, bp_span_up);              //自動帯増圧インターバル
    EEPROM.put(68, autoair_use);             //自動帯使用可否
    EEPROM.put(70, notch_mc_num_max);        //マスコンノッチ最大数
    EEPROM.put(72, notch_mc_num);            //マスコンノッチ数(車両)
    EEPROM.put(74, input_flip);              //入力反転(旧警報持続ボタン反転) 0:B接点 1以上:A接点
    EEPROM.put(76, Ats_Conf_flip);           //ATS確認ボタン反転 0:B接点 1以上:A接点
    EEPROM.put(78, Auto_Notch_Adjust);       //自動ノッチ合わせ
    EEPROM.put(80, RealAutoAir);             //実際のエアー圧で自動帯再現
    EEPROM.put(82, AtsContactUse);           //ATS接点情報を他基板へ伝送
    //初回書き込みフラグセット
    EEPROM.put(100, 1);
  } else {
    EEPROM.get(0, POT_N);                    //POT_N
    EEPROM.get(2, POT_EB);                   //POT_EB
    EEPROM.get(4, notch_brk_num);            //ブレーキ段数
    EEPROM.get(6, brk_sap_angl);             //直通帯幅
    EEPROM.get(8, brk_eb_angl);              //非常位置
    EEPROM.get(10, brk_full_angl);           //ブレーキ最大角度
    for (int i = 1; i <= 16; i++) {          //速度計補正値
      EEPROM.get((i * 2) + 10, spd_adj[i]);  //
    }                                        //
    EEPROM.get(44, spd_limit);               //速度計上限
    EEPROM.get(46, curr_kaisei);             //回生モード
    EEPROM.get(48, curr_mode);               //計器モード
    EEPROM.get(50, curr_limit);              //電流値上限
    EEPROM.get(52, vehicle_res);             //列車抵抗
    EEPROM.get(54, chat_filter);             //チャタリング
    EEPROM.get(56, brk_sap_max_angl);        //直通帯最大角度
    EEPROM.get(58, brk_sap_min_angl);        //直通帯最小角度
    EEPROM.get(60, brk_keep_angl);           //自動帯重なり開始位置
    EEPROM.get(62, brk_keep_full_angl);      //自動帯常用全開角度
    EEPROM.get(64, bp_span_down);            //自動帯減圧インターバル
    EEPROM.get(66, bp_span_up);              //自動帯増圧インターバル
    EEPROM.get(68, autoair_use);             //自動帯使用可否
    EEPROM.get(70, notch_mc_num_max);        //マスコンノッチ最大数
    EEPROM.get(72, notch_mc_num);            //マスコンノッチ数(車両)
    EEPROM.get(74, input_flip);              //入力反転(旧警報持続ボタン反転) 0:B接点 1以上:A接点
    EEPROM.get(76, Ats_Conf_flip);           //ATS確認ボタン反転 0:B接点 1以上:A接点
    EEPROM.get(78, Auto_Notch_Adjust);       //自動ノッチ合わせ
    EEPROM.get(80, RealAutoAir);             //実際のエアー圧で自動帯再現
    EEPROM.get(82, AtsContactUse);           //ATS接点情報を他基板へ伝送
  }



  //速度計テスト
  disp_SpeedMeter(spd_limit * 10);
  delay(1500);
  disp_SpeedMeter(0);
}

void loop() {
  static String strbve = "0000/1/ 00000/100000/0000000000000000000001";
  static String strbve_latch = "";
  static int16_t bve_current = 0;
  read_Serial1();
  read_USB(&strbve);

  if (strbve != strbve_latch) {
    uint8_t i = 0;
    String s = "";
    if (strbve.startsWith("WR ")) {
      //設定モード
      if (strbve.length() > 7) {
        uint8_t device = strbve.substring(3, 6).toInt();
        int16_t num = strbve.substring(7, 12).toInt();
        if (device < 100) {
          set_Settings(device, num);
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
      //通常モード：速度、戸閉、電流抽出
      if (strbve.length() > 11) {
        bve_speed = strbve.substring(0, 4).toInt();

        //戸閉灯指示
        digitalWrite(8, strbve.charAt(5) != '1');
        bve_current = strbve.substring(7, 12).toInt();

        //自動ノッチ合わせ機構
        AutoNotch(&strbve);
      }
    }

    //速度計表示補正 調整時表示のためここ
    disp_SpeedMeter(bve_speed);

    //電流計
    disp_CurrentMeter(bve_current);


    //Serial1転送
    send_Serial1(&strbve);
    strbve_latch = strbve;
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
#ifndef SIMPLE
  byte channelData = (ch + 8) << 4;
  //  Serial.println(String(channelData, BIN));
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
#else
  switch (ch) {
    case 0:
      return analogRead(A0);
      break;
    case 1:
      return analogRead(A3);
      break;
  }
#endif
}

//MCP23S17マスコンノッチ状態読込 (MC53抑速ブレーキ対応)
void read_MC(void) {
  uint8_t mcBit = ioexp_1_AB;      //MCP23S17読込
  static uint8_t mcBit_latch = 0;  //MCP23S17読込格納
  if (mcBit != mcBit_latch) {
    if (ioexp_1_AB >> PIN_MC_DEC & 1) {
      if (~ioexp_1_AB >> PIN_MC_5 & 1) {
        if ((notch_mc_num_max == 5) && (notch_mc_num == 4)) {
          notch_mc = 54;
          notch_name = "P4";
        } else if ((notch_mc_num_max == 5) && (notch_mc_num == 5)) {
          notch_mc = 55;
          notch_name = "P5";
        }
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB >> PIN_MC_4 & 1) {
        if ((notch_mc_num_max == 4) && (notch_mc_num == 5)) {
          notch_mc = 55;
          notch_name = "P5";
        } else if (((notch_mc_num_max == 5) && (notch_mc_num == 5)) || ((notch_mc_num_max == 4) && (notch_mc_num == 4))) {
          notch_mc = 54;
          notch_name = "P4";
        }
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB >> PIN_MC_3 & 1) {
        notch_mc = 53;
        notch_name = "P3";
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB >> PIN_MC_2 & 1) {
        notch_mc = 52;
        notch_name = "P2";
        autoair_dir_mask = false;
      } else if (~ioexp_1_AB >> PIN_MC_1 & 1) {
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
          cDir[0] = 'N';
        }
      }
    } else {
      if (input_flip >> 6 & 1) {  //非常有効時

        //マスコンデッドマン仮実装
        notch_mc = 50;
        notch_mc_H = 100;
        notch_name = "N ";
        notch_brk = notch_brk_num + 1;
        notch_brk_name = "EB";
        autoair_dir_mask = false;
      } else {  //抑速有効時
        if (!autoair_dir_mask) {
          if (~ioexp_1_AB >> PIN_MC_5 & 1) {
            notch_mc_H = 101;
            notch_name = "H1";
          } else if (~ioexp_1_AB >> PIN_MC_3 & 1 && ioexp_1_AB >> PIN_MC_4 & 1) {
            notch_mc_H = 102;
            notch_name = "H2";
          } else if (~ioexp_1_AB >> PIN_MC_2 & 1 && ioexp_1_AB >> PIN_MC_4 & 1) {
            notch_mc_H = 103;
            notch_name = "H3";
          } else if (~ioexp_1_AB >> PIN_MC_2 & 1 && ~ioexp_1_AB >> PIN_MC_4 & 1) {
            notch_mc_H = 104;
            notch_name = "H4";
          } else if (~ioexp_1_AB >> PIN_MC_3 & 1 && ~ioexp_1_AB >> PIN_MC_4 & 1) {
            notch_mc_H = 105;
            notch_name = "H5";
          }
        }
      }
    }
  }
  mcBit_latch = mcBit;
}

//マスコンレバーサ読取
void read_Dir(void) {
  if (!autoair_dir_mask) {
    if (~ioexp_1_AB >> PIN_MC_DIR_F & 1) {
      iDir = 1;
      cDir[0] = 'F';
      cDir_N[0] = 'L';
    } else if (~ioexp_1_AB >> PIN_MC_DIR_B & 1) {
      iDir = -1;
      cDir[0] = 'B';
      cDir_N[0] = 'R';
    } else {
      iDir = 0;
      cDir[0] = 'N';
    }
  }
}

//ブレーキ角度読取
uint16_t read_Break(void) {
  uint16_t adc = adcRead(0);
  static uint16_t adc_latch = 0;
  if (POT_N < POT_EB) {
    if (adc < POT_N) {
      adc = POT_N;
    } else if (adc > POT_EB) {
      adc = POT_EB;
    }
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
      //notch_brk = notch_brk_num + 1;
      notch_brk = 0;
      notch_brk_name = "N ";
      autoair_dir_mask = false;

      //直通帯位置
    } else if (brk_angl < brk_sap_max_angl) {
      uint16_t temp_notch_brk = round((float)(brk_angl - brk_sap_min_angl) / (float)(brk_sap_max_angl - brk_sap_min_angl) * (notch_brk_num - 1) + 0.5);
      //notch_brk = notch_brk_num + 1 - temp_notch_brk;
      notch_brk = temp_notch_brk;

      String s = String(temp_notch_brk);
      notch_brk_name = "B" + s;
      autoair_dir_mask = false;

      //常用最大位置～直通帯範囲まで
    } else if (brk_angl < brk_sap_angl) {
      notch_brk = notch_brk_num;
      notch_brk_name = "B" + String(notch_brk_num);
      autoair_dir_mask = false;

      //自動帯
    } else if (brk_angl < brk_keep_angl) {
      if (autoair_use) {
        //notch_brk = notch_brk_num + 1;
        notch_brk = 0;
        notch_brk_name = "N ";
        if (notch_mc == 50 && notch_mc_H == 100) {
          autoair_dir_mask = true;
          iDir = 0;
          cDir[0] = 'N';
        } else {
          autoair_dir_mask = false;
        }

      } else {
        //notch_brk = 1;
        notch_brk = notch_brk_num;
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
        //notch_brk = 1;
        notch_brk = notch_brk_num;
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
        //notch_brk = 1;
        notch_brk = notch_brk_num;
        notch_brk_name = "B" + String(notch_brk_num);
        autoair_dir_mask = false;
      }

      //非常位置以降
    } else {
      //notch_brk = 0;
      notch_brk = notch_brk_num + 1;
      notch_brk_name = "EB";
      autoair_dir_mask = false;
    }
    brk_angl_latch = brk_angl;
  }

  if (autoair_use) {
    BP(&brk_angl);
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
}

//キーボード(HID)出力
void keyboard_control(void) {
  //マスコンノッチが前回と異なるとき
  if (notch_mc != notch_mc_latch) {
    if (modeBVE) {
      uint8_t d = abs(notch_mc - notch_mc_latch);
      //力行ノッチ
      if (notch_mc >= 50 && notch_mc_latch >= 50 && notch_mc <= 55 && notch_mc_latch <= 55) {
        for (uint8_t i = 0; i < d; i++) {
          //進段
          if ((notch_mc - notch_mc_latch) > 0) {
            Keyboard.write(0x5A);  //" / "
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
        //if (notch_brk == notch_brk_num + 1) {
        if (notch_brk == 0) {
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
            Keyboard.write(0x51);  //" /
            //戻し
          } else {
            Keyboard.write(0x41);  //" / "
          }
        }
      }
    }
    if (modeN) {
      //if (notch_brk == notch_brk_num + 1) {
      if (notch_brk == 0) {
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
  //ブレーキノッチ(角度)が前回と異なるとき
  if (notch_brk != notch_brk_latch || notch_brk_name != notch_brk_name_latch) {
    if (modeBVE && !modeADJ && !modeN) {
      uint8_t d = abs(notch_brk - notch_brk_latch);
      //ブレーキノッチ
      //if (notch_brk <= (notch_brk_num + 1) && notch_brk_latch <= (notch_brk_num + 1) && notch_brk > 0) {
      if (notch_brk >= 0 && notch_brk_latch >= 0 && notch_brk < (notch_brk_num + 1)) {
        for (uint8_t i = 0; i < d; i++) {
          //戻し
          //if ((notch_brk - notch_brk_latch) > 0) {
          if ((notch_brk - notch_brk_latch) < 0) {
            Keyboard.write(0x2C);  //", "
          }
        }
        //ブレーキ
        //if ((notch_brk - notch_brk_latch) < 0) {
        if ((notch_brk - notch_brk_latch) > 0) {
          Keyboard.write(0x2E);  //"."
        }
      }
      //非常
      //if (notch_brk == 0) {
      if (notch_brk == notch_brk_num + 1) {
        Keyboard.write(0x2F);  //" / "
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
      for (uint8_t i = 0; i < d; i++) {
        //前進
        if ((iDir - iDir_latch) > 0) {
          Keyboard.write(0xDA);  //"↑"
          //後進
        } else {
          Keyboard.write(0xD9);  //"↓"
        }
      }
    }
    if (modeN) {
      if (iDir != 0) {
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
  static uint16_t adj_N = 0;
  static uint16_t adj_EB = 0;
  static unsigned long iniMillis_N = 0;
  static unsigned long iniMillis_EB = 0;
  static uint8_t setMode_N = 0;
  static uint8_t setMode_EB = 0;
  String s = "";
  uint16_t value = 0;
#ifndef SIMPLE
  bool btn_n = (adcRead(5) < 512);
  bool btn_eb = (adcRead(6) < 512);
#else
  bool btn_n = !digitalRead(3);
  bool btn_eb = !digitalRead(2);
#endif
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
      }
      adj_EB = (adj_EB * 9 + adcRead(0)) * 0.1;
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
  bool Horn_1 = ~ioexp_1_AB >> PIN_HORN_1 & 1;  //警笛1
  if (input_flip >> 4 & 1) {
    Horn_1 = !Horn_1;
  }
  static bool Horn_1_latch = Horn_1;
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

  bool Horn_2 = ~ioexp_1_AB >> PIN_HORN_2 & 1;  //警笛2
  if (input_flip >> 5 & 1) {
    Horn_2 = !Horn_2;
  }
  static bool Horn_2_latch = Horn_2;
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
    Horn_2_latch = Horn_2;
  }
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

  //ATS確認位置転送
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
  bool Ats_Cont = ~ioexp_1_AB >> PIN_ATS_CONT & 1;  //警報持続スイッチ
  //ATS警報持続
  if (input_flip & 1) {
    Ats_Cont = !Ats_Cont;
  }
  static bool Ats_Cont_latch = Ats_Cont;  //警報持続スイッチ
  if (Ats_Cont != Ats_Cont_latch) {
    if (Ats_Cont) {
      if (modeBVE) {
        Serial1.print("ACT 1");
        Serial1.print("\r");
        Serial.println("ACT 1 ON");
        Keyboard.press(0xD1);  //"Insert"
      }
    } else {
      if (modeBVE) {
        Serial1.print("ACT 0");
        Serial1.print("\r");
        Serial.println("ACT 0 OFF");

        Keyboard.release(0xD1);
      }
    }
    Ats_Cont_latch = Ats_Cont;
  }

  //ATS確認
  bool Ats_Conf = ~ioexp_1_AB >> PIN_ATS_CONF & 1;  //ATS確認ボタン
  if (Ats_Conf_flip) {
    Ats_Conf = !Ats_Conf;
  }
  static bool Ats_Conf_latch = Ats_Conf;  //ATS確認ボタン
  if (Ats_Conf != Ats_Conf_latch) {
    if (Ats_Conf) {
      if (modeBVE) {
        if (Ats_Pos) {
          Serial1.print("ACF 1");
          Serial1.print("\r");
          Serial.println("ACF 1 ON");

          Keyboard.press(0x20);  //"Space"
        }
      }
    } else {
      if (modeBVE) {
        Serial1.print("ACF 0");
        Serial1.print("\r");
        Serial.println("ACF 0 OFF");

        Keyboard.release(0x20);
      }
    }
    Ats_Conf_latch = Ats_Conf;
  }

  //ATS復帰
  bool Ats_Rec = digitalRead(9);
  if (input_flip >> 2 & 1) {
    Ats_Rec = !Ats_Rec;
  }
  static bool Ats_Rec_latch = Ats_Rec;
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
    Ats_Rec_latch = Ats_Rec;
  }
}

void read_Panto(void) {
  bool Panto = ~ioexp_1_AB >> PIN_PANTO & 1;
  static bool Panto_latch = false;
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
    Panto_latch = Panto;
  }
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
  bool EB_SW = digitalRead(5);
  if (input_flip >> 3 & 1) {
    EB_SW = !EB_SW;
  }
  static bool EB_SW_latch = 0;
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
    EB_SW_latch = EB_SW;
  }
}

void disp_SpeedMeter(uint16_t spd) {
  //速度計のリミットを適用
  if (spd > (spd_limit * 10)) {
    spd = (spd_limit * 10);
  }

  uint16_t volt = 0;
  for (int i = 1; i < 18; i++) {
    if (spd <= (i * 100)) {
      volt = map(spd, (i - 1) * 100, i * 100, spd_adj[i - 1], spd_adj[i]);
      break;
    }
  }
#ifndef SIMPLE
  dac.setVoltage(volt, false);
#else
  analogWrite(10, volt >> 4);
#endif
}

void BP(uint8_t *angl) {
  static bool EB_latch = false;
  if (!RealAutoAir) {
    //BPの増減圧インターバルを設定
    if (*angl < brk_keep_angl) {
      bp_span = bp_span_up;
    } else if ((*angl >= brk_keep_angl) && (*angl <= brk_keep_full_angl)) {
      bp_span = map(angl, brk_keep_angl, brk_keep_full_angl, bp_span_down, bp_span_down);
    } else if (*angl > brk_keep_full_angl) {
      bp_span = bp_span_down;
    }

    //直通帯(運転位置)でBP圧を加圧
    if (*angl < brk_sap_angl) {
      EB_latch = false;
      if ((millis() - bp_millis) > bp_span && brk_bp_press < 490) {
        brk_bp_press++;
        bp_millis = millis();
      }
      //重なり位置でBP圧は維持
    } else if (*angl < brk_keep_angl) {

      //常用位置でBP圧を減圧
    } else if (*angl < brk_eb_angl) {
      if ((millis() - bp_millis) > bp_span && brk_bp_press > 0) {
        brk_bp_press--;
        bp_millis = millis();
      }
    } else if (*angl >= brk_eb_angl) {
      EB_latch = true;
      brk_bp_press = 0;
    }

    //自動帯でのBC圧をBP圧より生成
    BC_press = (490 - brk_bp_press) * 2.5;
    //非常吐出なく、自動帯常用でBC圧力が440kPa以上のとき440kPaとする
    if (BC_press > 440) {
      if (*angl < brk_eb_angl && !EB_latch) {
        BC_press = 440;
      }
    }
    //急動部動作時
    if (BC_press > 490) {
      if (*angl >= brk_eb_angl || EB_latch) {
        BC_press = 490;
      }
    }
  }

  //BC圧からブレーキノッチに変換
  if (EB_latch) {
    autoair_notch_brk = notch_brk_num + 1;
  } else {
    autoair_notch_brk = map(BC_press, 0, 440, 0, notch_brk_num);
  }
  //自動帯圧力優先シーケンス
  //N位置
  if (*angl <= brk_sap_min_angl) {
    //自動ブレーキ
    if (autoair_notch_brk >= 0) {  //自動ブレーキ帯の段数が(N位置より)高いとき
      notch_brk = autoair_notch_brk;
    }
    //直通帯位置
  } else if (*angl < brk_sap_angl) {
    //自動ブレーキ
    if (notch_brk < autoair_notch_brk) {  //自動ブレーキ帯の段数が高いとき
      notch_brk = autoair_notch_brk;
    }

    //自動帯
  } else if (*angl < brk_eb_angl) {       //自動ブレーキ
    if (notch_brk < autoair_notch_brk) {  //自動ブレーキ帯の段数が高いとき
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

void AutoNotch(String *str) {
  if (str->length() > 49) {
    if (Auto_Notch_Adjust) {
      if (str->charAt(47) == 'B') {
        int bve_rev = 0;
        if (str->charAt(44) == 'F') {
          bve_rev = 1;
        } else if (str->charAt(44) == 'B') {
          bve_rev = -1;
        }
        String bve_brk = str->substring(48, 50);
        char Buf[4];
        bve_brk.toCharArray(Buf, 4);
        uint8_t num = strtol(Buf, NULL, 16);  //16進数→10進数に変換
        if (!autoair_dir_mask) {
          iDir_latch = bve_rev;
          //notch_brk_latch = (notch_brk_num + 1) - num;
          notch_brk_latch = num;
        }
      }
    }
  }
}

void disp_CurrentMeter(int16_t current) {
  //回生モードでないとき、負電流は0
  if (!curr_kaisei && (current < 0)) {
    current = 0;
  }
  //電圧計モードのとき計算後2000Vを超えた時は2000Vに固定
  if (curr_mode) {
    uint16_t v = 1500 - (current * (vehicle_res / 1000.0));
    if (v > 2000) {
      v = 2000;
    }
    //出力
#ifndef SIMPLE
    dac2.setVoltage(map(v, 0, 2000, 0, 4095), false);
#else
    //analogWrite(11, map(v, 0, 2000, 0, 4095) / 4);
    analogWrite(11, map(v, 0, 2000, 0, 4095) >> 4);
#endif
    //電流計モードの時
  } else {
    //電流計は正のみ(現状)
    int curr = abs(current);
    //電流が電流上限を超えた時は電流上限に抑える
    if (curr > curr_limit) {
      curr = curr_limit;
    }
    //出力
#ifndef SIMPLE
    dac2.setVoltage(map(curr, 0, curr_limit, 0, 4095), false);
#else
    //analogWrite(11, map(curr, 0, curr_limit, 0, 4095) / 4));
    analogWrite(11, map(curr, 0, curr_limit, 0, 4095) >> 4));
#endif
  }
}

void read_Serial1() {
  if (Serial1.available() > 0) {
    String str1 = Serial1.readStringUntil('\r');
    if (str1.startsWith("BC ") && RealAutoAir) {
      BC_press = str1.substring(3, 6).toInt();
    } else {
      Serial.println(str1);
    }
  }
}

void read_USB(String *str) {
  if (Serial.available()) {
    *str = Serial.readStringUntil('\r');
  }
}

void send_Serial1(String *str) {
  //自動帯有効時、電制を無効とする
  if (autoair_use && brk_angl > brk_sap_angl) {
    if (str->length() > 18) {
      str->setCharAt(18, '0');
    }
  }
  Serial1.print(*str);
  Serial1.print('\r');
}

void set_Settings(uint8_t device, int16_t num) {
  String s = "";
  switch (device) {
    //緩め位置設定
    case 0:
      s = rw_eeprom(device, &num, &POT_N, true, num < 0 || num > 1023);
      break;

    //非常位置設定
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
    case 14:
    case 16:
    case 18:
    case 20:
    case 22:
    case 24:
    case 26:
    case 28:
    case 30:
    case 32:
    case 34:
    case 36:
    case 38:
    case 40:
    case 42:
      s = rw_eeprom(device, &num, &spd_adj[(device - 10) / 2], true, false);
      bve_speed = ((device - 10) / 2) * 100;
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
    case 52:  //列車抵抗
      s = rw_eeprom(device, &num, &vehicle_res, true, num == 0);
      break;
    case 54:  //チャタリング
      s = rw_eeprom(device, &num, (uint16_t)&chat_filter, true, num < 0 || num > 3);
      break;
    case 56:  //常用最大角度
      s = rw_eeprom(device, &num, &brk_sap_max_angl, true, num == 0 || num > brk_sap_angl);
      break;
    case 58:  //直通帯最小角度
      s = rw_eeprom(device, &num, &brk_sap_min_angl, true, num == 0 || num > brk_sap_max_angl);
      break;
    case 60:  //自動帯常用開始角度
      s = rw_eeprom(device, &num, &brk_keep_angl, true, num < brk_sap_max_angl || num > brk_eb_angl);
      break;
    case 62:  //自動帯常用全開角度
      s = rw_eeprom(device, &num, &brk_keep_full_angl, true, num < brk_sap_max_angl || num > brk_eb_angl);
      break;
    case 64:  //自動帯減圧インターバル
      s = rw_eeprom(device, &num, &bp_span_down, true, num == 0 || num > 100);
      break;
    case 66:  //自動帯増圧インターバル
      s = rw_eeprom(device, &num, &bp_span_up, true, num == 0 || num > 100);
      break;
    case 68:  //自動帯使用可否
      s = rw_eeprom(device, &num, &autoair_use, true, num < 0 || num > 1);
      break;
    case 70:  //マスコンノッチ最大数
      s = rw_eeprom(device, &num, &notch_mc_num_max, true, num < 0);
      break;
    case 72:  //マスコンノッチ数(車両)
      s = rw_eeprom(device, &num, &notch_mc_num, true, num < 0);
      break;
    case 74:  //警報持続ボタン反転 0:B接点 1以上:A接点
      s = rw_eeprom(device, &num, &input_flip, true, num < 0 || num > 65535);
      break;
    case 76:  //ATS確認ボタン反転 0:B接点 1以上:A接点
      s = rw_eeprom(device, &num, &Ats_Conf_flip, true, num < 0 || num > 1);
      break;
    case 78:  //自動ノッチ合わせ
      s = rw_eeprom(device, &num, (uint16_t)&Auto_Notch_Adjust, true, num < 0 || num > 1);
      break;
    case 80:  //実際のエアー圧で自動帯再現
      s = rw_eeprom(device, &num, (uint16_t)&RealAutoAir, true, num < 0 || num > 1);
      break;
    case 82:  //ATS接点情報を他基板へ伝送
      s = rw_eeprom(device, &num, &AtsContactUse, true, num < 0 || num > 1);
      break;

    default:
      s = "E0";
      break;
  }
  Serial.println(s);
}