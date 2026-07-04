/*
MIT License

Copyright (c) 2026 Gazelle@Gazelle8087

https://github.com/Gazelle8087/Si4730-AM-FM-tuner

Permission is hereby granted, free of charge,
to any person obtaining a copy of this software
and associated documentation files (the “Software”),
to deal in the Software without restriction,
including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice
shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
変更履歴
2026 July 4 初回公開
*/

#include <TinyIRReceiver.hpp>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// ピンアサイン定義
constexpr auto IR_INPUT_PIN = 2;  // Atmega328P  4ピン IRレシーバー
constexpr auto GPIO2_PIN    = 3;  // Atmega328P  5ピン GPIO2/INT no use (INPUT_PULLUP)
constexpr auto RSTB_PIN     = 4;  // Atmega328P  6ピン Si4730 RESET (L active）
constexpr auto SENB_PIN     = 5;  // Atmega328P 11ピン 3w serial SENB (L active）
constexpr auto SDIO_PIN     = 6;  // Atmega328P 12ピン 3w serial SDIO bi-dirction
constexpr auto ACOMP_IN     = 7;  // Atmega328P 13ピン 電源電圧監視 AIN1
constexpr auto SCLK_PIN     = 8;  // Atmega328P 14ピン 3w serial SCLK
constexpr auto SCLK         = 0;  // Atmega328P 14ピン PORTB bit0
constexpr auto HARD_MUTE    = 9;  // Atmega328P 15ピン 2SC2878使ったハードミュート
constexpr auto SPI_CS       = 10; // Atmega328P 16ピン SPIのSS（現状未使用）
constexpr auto SPI_MOSI     = 11; // Atmega328P 17ピン SPIのMOSI（現状未使用）
constexpr auto SPI_MISO     = 12; // Atmega328P 18ピン SPIのMISO（現状未使用）
constexpr auto SPI_SCK      = 13; // Atmega328P 19ピン SPIのSCK（現状未使用）
constexpr auto XTAL1        = 20; // Atmega328P  9ピン 現状未使用
constexpr auto XTAL2        = 21; // Atmega328P 10ピン 現状未使用

constexpr auto LCD_ADRS = 0x3E;   // LCD I2C address

// ELPA スリムリモコン RC-TV013UD を設定コード「1241（サンヨー）」に設定して使用
// IRで受信したアドレス（メーカーコード）は 0x30になることを確認済
constexpr uint16_t MAKER_ADDRESS   = 0x30;

constexpr uint16_t FREQ_MAX_AM     = 1710;
//constexpr uint16_t FREQ_MAX_AM   = 30000; SWはNG
constexpr uint16_t FREQ_MAX_MW     = 1710;
//constexpr uint16_t FREQ_MIN_AM   = 149;  LWはNG
constexpr uint16_t FREQ_MIN_AM     = 522;
constexpr uint16_t FREQ_STEP_AM    = 9;
constexpr uint16_t FREQ_MAX_FM     = 9500;
constexpr uint16_t FREQ_MIN_FM     = 7600;
constexpr uint16_t FREQ_STEP_FM    = 10;
constexpr uint8_t  PRESET_CH_MAX   = 12;
constexpr uint8_t  PRESET_BANK_MAX = 12;

constexpr uint8_t WIDTH_INDEX_MAX_AM = 6;
constexpr uint8_t WIDTH_INDEX_MAX_FM = 4;
constexpr uint8_t WIDTH_INDEX_MIN_AM = 0;
constexpr uint8_t WIDTH_INDEX_MIN_FM = 0;
constexpr uint8_t MAX_VOL = 63;


// デフォルトに対し、モノラル遷移の閾値を下げた
// もともとPILOT存在判定が厳しめでPILOT検出あればステレオでノイズ盛大なことはあまりない模様
//
//                 パラメータ名（AN332記述に合わせた）         パラメータアドレス
constexpr uint16_t FM_DEEMPHASIS                       = 1;           //Property 0x1100
constexpr uint16_t FM_SEEK_BAND_BOTTOM                 = FREQ_MIN_FM; //Property 0x1400
constexpr uint16_t FM_SEEK_BAND_TOP                    = FREQ_MAX_FM; //Property 0x1401
constexpr uint16_t FM_SEEK_FREQ_SPACING                = FREQ_STEP_FM;//Property 0x1402
constexpr uint16_t FM_SEEK_TUNE_SNR_THRESHOLD          = 3;           //Property 0x1403
constexpr uint16_t FM_SEEK_TUNE_RSSI_THRESHOLD         = 20;          //Property 0x1404
constexpr uint16_t FM_BLEND_RSSI_STEREO_THRESHOLD      = 10;          //Property 0x1800
constexpr uint16_t FM_BLEND_RSSI_MONO_THRESHOLD        = 00;          //Property 0x1801
constexpr uint16_t FM_BLEND_RSSI_ATTACK_RATE           = 4000;        //Property 0x1802
constexpr uint16_t FM_BLEND_RSSI_RELEASE_RATE          = 4000;        //Property 0x1803
constexpr uint16_t FM_BLEND_SNR_STEREO_THRESHOLD       = 10;          //Property 0x1804
constexpr uint16_t FM_BLEND_SNR_MONO_THRESHOLD         = 00;          //Property 0x1805
constexpr uint16_t FM_BLEND_SNR_ATTACK_RATE            = 4000;        //Property 0x1806
constexpr uint16_t FM_BLEND_SNR_RELEASE_RATE           = 4000;        //Property 0x1807
constexpr uint16_t FM_BLEND_MULTIPATH_STEREO_THRESHOLD = 40;          //Property 0x1808
constexpr uint16_t FM_BLEND_MULTIPATH_MONO_THRESHOLD   = 60;          //Property 0x1809
constexpr uint16_t FM_BLEND_MULTIPATH_ATTACK_RATE      = 4000;        //Property 0x180A
constexpr uint16_t FM_BLEND_MULTIPATH_RELEASE_RATE     = 4000;        //Property 0x180B
constexpr uint16_t AM_DEEMPHASIS                       = 1;           //Property 0x3100
constexpr uint16_t AM_SEEK_BAND_BOTTOM                 = FREQ_MIN_AM; //Property 0x3400
constexpr uint16_t AM_SEEK_BAND_TOP                    = FREQ_MAX_AM; //Property 0x3401
constexpr uint16_t AM_SEEK_FREQ_SPACING                = FREQ_STEP_AM;//Property 0x3402
constexpr uint16_t AM_SEEK_TUNE_SNR_THRESHOLD          = 5;           //Property 0x3403
constexpr uint16_t AM_SEEK_TUNE_RSSI_THRESHOLD         = 25;          //Property 0x3404

// Si4730 コマンドコード
constexpr uint16_t POWER_UP_AM    = 0x0111;
constexpr uint16_t POWER_UP_FM    = 0x0110;
constexpr uint16_t POWER_UP_Q     = 0x011f;
constexpr uint16_t GET_REV        = 0x1000;
constexpr uint16_t POWER_DPWN     = 0x1100;
constexpr uint16_t SET_PROPERTY   = 0x1200;
constexpr uint16_t GET_INT_STATUS = 0x1400;
constexpr uint16_t FM_TUNE_FREQ   = 0x2000;
constexpr uint16_t FM_TUNE_STATUS = 0x2200;
constexpr uint16_t FM_RSQ_STATUS  = 0x2300;
constexpr uint16_t AM_TUNE_FREQ   = 0x4000;
constexpr uint16_t AM_TUNE_STATUS = 0x4200;
constexpr uint16_t AM_RSQ_STATUS  = 0x4300;
constexpr uint16_t AM_AGC_STATUS  = 0x4700;

// 3線シリアル用レジスタアドレス
constexpr uint8_t  CMD            = 0xa0;
constexpr uint8_t  ARG23          = 0xa1;
constexpr uint8_t  ARG45          = 0xa2;
constexpr uint8_t  STATUS         = 0xa8;
constexpr uint8_t  RESP23         = 0xa9;
constexpr uint8_t  RESP45         = 0xaa;
constexpr uint8_t  RESP67         = 0xab;
constexpr uint8_t  RESP89         = 0xac;

// Si4730 プロパティアドレス
constexpr uint16_t FM_CHANNEL_FILTER = 0x1102;
constexpr uint16_t AM_CHANNEL_FILTER = 0x3102;

constexpr uint32_t SQR_PERIOD       = 500; // 500ms 毎にRSQ表示
constexpr uint32_t DEBOUNCE_PERIOD  = 180; // デバウンス180ms
constexpr auto DELAY_AFTER_POWER_UP = 110; // POWER UP後の待ち時間
constexpr auto DELAY_AFTER_TUNE     = 200; // TUNE 後の待ち時間

uint32_t start_time = millis();           // RSQ表示タイマー
uint32_t debounce_time = millis();        // デバウンスタイマー

uint8_t status = 0;     //Si4730 データ授受 とりあえずグローバル変数にしてる
uint8_t resp1 = 0;      //制御固まったらスコープ変えるかも 
uint8_t resp2 = 0;
uint8_t resp3 = 0;
uint8_t resp4 = 0;
uint8_t resp5 = 0;
uint8_t resp6 = 0;
uint8_t resp7 = 0;
uint8_t resp8 = 0;
uint8_t resp9 = 0;

bool af_mute     = false;  //音声ミュートフラグ
bool fine_tuning = false;  //微調整フラグ
bool ch_empty    = false;  //選択chが空のときセット(一時的なEMPTY表示用）

// 帯域表構造体の定義
struct widthconfig {
  uint8_t index;      // PEOPERTYにセットする値
  uint8_t value;      // 帯域の数値
};

// AM用帯域幅テーブル
const widthconfig AM_WIDTH[7] = {
  {4, 10}, {5, 18}, {3, 20}, {6, 25},
  {2, 30}, {1, 40}, {0, 60}
};

// FM用帯域幅テーブル
const widthconfig FM_WIDTH[5] = {
  {4, 40}, {3, 60}, {2, 84}, {1,110}, {0,0}
};

enum band {
  AM,
  FM,
};

// プリセットメモリ構造体（3バイトに収める）
struct Ch_config {
  uint16_t frequency    : 16;
  uint8_t  width        :  3;
  bool     stereo       :  1;
  bool     auto_mute    :  1;
  band     band         :  1;
  bool     preset       :  1;
} __attribute__((packed)); // 隙間詰め指定

static_assert(sizeof(Ch_config) <= 3, "PRESET_memory > 3byte");

Ch_config AM_config = {   // AM初期値
  .frequency    = 729,    // 729kHz
  .width        = 6,      // 最大帯域 6kHz
  .stereo       = 0,      // AMは未使用
  .auto_mute    = 0,      // 未使用
  .band         = AM,
  .preset       = false,
};

Ch_config FM_config = {   // FM初期値
  .frequency    = 8250,   // 82.5MHz
  .width        = 4,      // 自動帯域調整
  .stereo       = 1,      // ステレオ指定
  .auto_mute    = 0,      // 未使用
  .band         = FM,
  .preset       = false,
};

// システム状態構造体
struct Sys_config {
  uint8_t preset_ch_am : 8; // 最後にCALLされたAM_ch
  uint8_t preset_ch_fm : 8; // 最後にCALLされたFM_ch
  uint8_t bank         : 8; // 選択中のプリセットバンク
  uint8_t vol          : 6; // 音量
  band    band         : 1; // 2値（AM/FM）1ビット
} __attribute__((packed));

Sys_config Tuner_config {
  .preset_ch_am = 0,
  .preset_ch_fm = 0,
  .bank         = 1,
  .vol          = MAX_VOL,
  .band         = FM,
};

// EEPROMメモリマップ
struct Eeprom_Memory_Map {
  uint8_t    reserve0;                                  // 0-2番地使わない
  uint8_t    reserve1;
  uint8_t    reserve2;
  Ch_config  presets[PRESET_CH_MAX * PRESET_BANK_MAX];  // プリセット最大144個
  Ch_config  dummy[193];                                // ダミー193個
  Sys_config Tuner_config;                              // ラストステーション全般 4バイト
  Ch_config  last_station_am;                           // ラストステーションAM 3バイト
  Ch_config  last_station_fm;                           // ラストステーションFM 3バイト
};
static_assert(sizeof(Eeprom_Memory_Map) <= 1024, "Shortage of EEPROM");
//===================================================================
// 赤外線リモコン リモートコード変換
// ※ ELPA スリムリモコン RC-TV013UD を設定コード「1241（サンヨー）」に設定して使用
// 引数 uint16_t address  IRで受信したアドレス（メーカーコード）
// 引数 uint8_t  IR_code  IRで受信したコマンド（リモートコード）
// 戻り uint8_t  key_code 機能番号（対象外または未登録のボタンは99）

uint8_t translateIR(uint16_t IR_address, uint8_t IR_command) {

  if (IR_address != MAKER_ADDRESS) {  // 対象外のメーカーコードなら、99を返して終了
    return 99;
  }

  uint8_t key_code;

  switch(IR_command)  {

    case  16: key_code=1;   break; // 数字1
    case  17: key_code=2;   break; // 数字2
    case  18: key_code=3;   break; // 数字3
    case  19: key_code=4;   break; // 数字4
    case  20: key_code=5;   break; // 数字5
    case  21: key_code=6;   break; // 数字6
    case  22: key_code=7;   break; // 数字7
    case  23: key_code=8;   break; // 数字8
    case  24: key_code=9;   break; // 数字9
    case  25: key_code=10;  break; // 数字10
    case  26: key_code=11;  break; // 数字11
    case  27: key_code=12;  break; // 数字12
    case   0: key_code=200; break; // [電源]
    case   5: key_code=201; break; // [入力切替]
    case   9: key_code=202; break; // [vol+]
    case  70: key_code=203; break; // [放送切替]
    case  10: key_code=204; break; // [vol-]
    case  52: key_code=205; break; // [ ▲ ]
    case  55: key_code=206; break; // [ < ]
    case  49: key_code=207; break; // [決定]
    case  54: key_code=208; break; // [ > ]
    case  53: key_code=209; break; // [ ▼ ]
    case 224: key_code=210; break; // [ d ]
    case 229: key_code=211; break; // [戻る]
    case  63: key_code=212; break; // [メニュー]
    case 220: key_code=213; break; // [番組表]
    case 225: key_code=214; break; // [青]
    case 226: key_code=215; break; // [赤]
    case 227: key_code=216; break; // [緑]
    case 228: key_code=217; break; // [黄]

    default: key_code=99;
  }
  return key_code;
}
//-------------------------------------------------------------------
uint8_t translateIR_N(uint16_t IR_address, uint8_t IR_command){

// 周波数直打用 IRコード⇒数値変換 [10] を 0として扱う

  uint8_t key_code;
  
  if (IR_address != MAKER_ADDRESS) {
    return 99;
  }
 
  switch(IR_command)  {
    case  16: key_code=1;  break; // 数字1
    case  17: key_code=2;  break; // 数字2
    case  18: key_code=3;  break; // 数字3
    case  19: key_code=4;  break; // 数字4
    case  20: key_code=5;  break; // 数字5
    case  21: key_code=6;  break; // 数字6
    case  22: key_code=7;  break; // 数字7
    case  23: key_code=8;  break; // 数字8
    case  24: key_code=9;  break; // 数字9
    case  25: key_code=0;  break; // 数字10

    default: key_code=99;
  }
  return key_code;
}
// ============ Si4730 双方向3線シリアル入出力 ========================
// ------------ 送信 -------------------------------------------------
void si4730_write16(uint8_t addr, uint16_t arg) {

  // 開始 (SENBをLOWに)
  PORTB &= ~(1 << SCLK);
  pinMode(SDIO_PIN, OUTPUT);
  PORTD &= ~(1 << SENB_PIN);

  // 上位3ビット
  for (int i = 3; i > 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // 4ビット目は書き込みフラグ 0
  PORTD &= ~(1 << SDIO_PIN);
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  //  残りのアドレス5ビット
  for (int i = 4; i >= 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // arg 16ビット
  for (int i = 15; i >= 0; i--) {
    if (arg & 0x8000){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    arg = arg << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // 終了の準備 (SENBを先にHIGHにする)
  PORTD |= (1 << SENB_PIN);

  // SENBがHIGHの状態で、最後に1回SCLKパルスを入れる
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  pinMode(SDIO_PIN, INPUT);
}
// ------------ 受信 8bit × 2 ----------------------------------------
uint8_t si4730_read(uint8_t addr, uint8_t &resp1, uint8_t &resp2) {

  // 開始 (SENBをLOWに)
  pinMode(SDIO_PIN, OUTPUT);
  PORTB &= ~(1 << SCLK);
  PORTD &= ~(1 << SENB_PIN);

  // 上位3ビット
  for (int i = 3; i > 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // 4ビット目は読み込みフラグ 1
  PORTD |= (1 << SDIO_PIN);
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  // 残りのアドレス5ビット
  for (int i = 4; i >= 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // 入力に切り替え
    pinMode(SDIO_PIN, INPUT);

  // resp1 8ビット
  resp1 = 0;
  for (int i = 7; i >= 0; i--) {
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
    resp1 = resp1 << 1;
    if ((PIND & (1 << SDIO_PIN))) {  // ここ早すぎると取りこぼす
      resp1++;                       // SCLK 下げを後にするとNG
    }
  }

  // resp2 8ビット
  resp2 = 0;
  for (int i = 7; i >= 0; i--) {
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
    resp2 = resp2 << 1;
    if ((PIND & (1 << SDIO_PIN))) {
      resp2++;
    }
  }

  // 終了の準備 (SENBを先にHIGHにする)
  PORTD |= (1 << SENB_PIN);

  // SENBがHIGHの状態で、最後に1回SCLKパルスを入れる
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  return resp1;
}
// ------------ 受信 16bit ----------------------------------------
uint16_t si4730_read16(uint8_t addr) {

  uint16_t resp = 0;

  // 開始 (SENBをLOWに)
  pinMode(SDIO_PIN, OUTPUT);
  PORTB &= ~(1 << SCLK);
  PORTD &= ~(1 << SENB_PIN);

  // 上位3ビット
  for (int i = 3; i > 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // 4ビット目は読み込みフラグ 1
  PORTD |= (1 << SDIO_PIN);
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  // 残りのアドレス5ビット
  for (int i = 4; i >= 0; i--) {
    if (addr & 0x80){
      PORTD |= (1 << SDIO_PIN);
    } else {
      PORTD &= ~(1 << SDIO_PIN);
    }
    addr = addr << 1;
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
  }

  // sdioを入力に切り替え
    pinMode(SDIO_PIN, INPUT);

  // resp 16ット

  for (int i = 15; i >= 0; i--) {
    PORTB |= (1 << SCLK);
    PORTB &= ~(1 << SCLK);
    resp = resp <<1;
    if ((PIND & (1 << SDIO_PIN))) {
      resp++;
    }
  }

  // 終了の準備 (SENBを先にHIGHにする)
  PORTD |= (1 << SENB_PIN);

  // SENBがHIGHの状態で、最後に1回SCLKパルスを入れる
  PORTB |= (1 << SCLK);
  PORTB &= ~(1 << SCLK);

  return resp;
}
// ============ Si4730 双方向3線シリアル入出力 ここまで================
//============= LCD 基本入出力関数 ===================================
// ----- 1602にデータを送信
void LCD_write_data(uint8_t data){
  Wire.beginTransmission(LCD_ADRS);
  Wire.write(0x40);
  Wire.write(data);
  Wire.endTransmission();
  delay(1);
}
// ----- 1602にコマンドを送信
void LCD_write_command(uint8_t command){
  Wire.beginTransmission(LCD_ADRS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
  delay(10);  
}
// ----- 数字を表示 10以上は A,B,C.....
void LCD_put_num(uint8_t var){
  if (var <= 9){
    LCD_write_data(var + 0x30);
  } else {
    LCD_write_data(var + 0x37);
  }
}
// ----- LCD ユーティリティ関数 ------------------
// ----- LCD初期化 コントラスト指定
void LCD_init(){
  delay(100);
  LCD_write_command(0x38);
  delay(20);
  LCD_write_command(0x39);
  delay(20);
  LCD_write_command(0x14);
  delay(20);
  LCD_write_command(0x73);
  delay(20);
  LCD_write_command(0x56);
  delay(20);
  LCD_write_command(0x6c);
  delay(20);
  LCD_write_command(0x38);
  delay(20);
  LCD_write_command(0x01);
  delay(20);
  LCD_write_command(0x0c);
  delay(20);
}
// ----- FM周波数表示
void LCD_put_FM_freq(uint16_t freq){
  LCD_write_command(0x84);
  LCD_put_num(freq/1000);
  freq = freq % 1000;
  LCD_put_num(freq/100);
  LCD_write_data(0x2e);
  freq = freq % 100;
  LCD_put_num(freq/10);
  freq = freq % 10;
  LCD_put_num(freq);
}
// ----- AM周波数表示 先頭の0は空白
void LCD_put_AM_freq(uint16_t freq){
  LCD_write_command(0x84);
  if (freq >= 10000) {
    LCD_put_num(freq/10000);
  } else {
    LCD_write_data(0x20);
  }
  freq = freq % 10000;
  if (freq >= 1000) {
    LCD_put_num(freq/1000);
  } else {
    LCD_write_data(0x20);
  }
  freq = freq % 1000;
  LCD_put_num(freq/100);
  freq = freq % 100;
  LCD_put_num(freq/10);
  freq = freq % 10;
  LCD_put_num(freq);
}
// ----- 周波数の位置に [EMPTY]表示
void LCD_empty(){
  LCD_write_command(0x84);
  LCD_write_data(0x45);
  LCD_write_data(0x4d);
  LCD_write_data(0x50);
  LCD_write_data(0x54);
  LCD_write_data(0x59);
  start_time = millis();
}
// ----- AM同調周波数表示
void LCD_put_AM_cap(uint16_t cap){
  cap = ((float)cap * 0.095 + 7.0);
  LCD_write_command(0xCd);
  LCD_put_num(cap/100);
  cap = cap % 100;
  LCD_put_num(cap/10);
  cap = cap % 10;
  LCD_put_num(cap);
}
// ----- AM帯域表示
void LCD_put_AM_width(uint8_t width){
  LCD_write_command(0x8d);
  LCD_put_num(width/10);
  LCD_write_data(0x2e);
  width = width % 10;
  LCD_put_num(width);  
}
// ----- FM帯域表示 自動時は [  A]と表示
void LCD_put_FM_width(uint8_t width){
  LCD_write_command(0x8d);
  if (width == 0){
    LCD_write_data(0x20);
    LCD_write_data(0x20);
    LCD_write_data(0x41);
  } else {
  if (width / 100) {
    LCD_put_num(width/100);
  } else {
    LCD_write_data(0x20);
  }
    width = width % 100;
    LCD_put_num(width/10);
    width = width % 10;
    LCD_put_num(width);
  }  
}
// ----- RSSI表示
void LCD_put_RSSI(uint8_t rssi){
  LCD_write_command(0xc1);
  LCD_put_num(rssi/10);
  rssi = rssi % 10;
  LCD_put_num(rssi);   
}
// ----- SNR 表示
void LCD_put_SNR(uint8_t snr){
  LCD_write_command(0xc5);
  LCD_put_num(snr/10);
  snr = snr % 10;
  LCD_put_num(snr);   
}
// ----- Pilot検出かつ強制モノラルOFFの場合 [Bxxx] と表示
// ----- Bがパイロット検出表示 数字がブレンド値
// ----- 強制モノラルONの場合 ブレンド値を --- でマスク
void LCD_put_BLEND(uint8_t blend){
  LCD_write_command(0xcc);
  LCD_write_data(0x42);
  LCD_put_num(blend/100);
  blend = blend % 100;
  LCD_put_num(blend/10);
  blend = blend % 10;
  LCD_put_num(blend);   
}
// ----- Pilot非検出かつ強制モノラルOFFの場合 空白 [    ]
void LCD_put_MONO(){
  LCD_write_command(0xcc);
  LCD_write_data(0x20);
  LCD_write_data(0x20);
  LCD_write_data(0x20);
  LCD_write_data(0x20);
}
// ----- Pilot検出かつ強制モノラルONの場合 [B---]
void LCD_put_force_MONO(){
  LCD_write_command(0xcc);
  LCD_write_data(0x42);
  LCD_write_data(0x2d);
  LCD_write_data(0x2d);
  LCD_write_data(0x2d);
}
// ---- Pilot非検出かつ強制モノラルONの場合 [ ---]
void LCD_put_force_MONO2(){
  LCD_write_command(0xcc);
  LCD_write_data(0x20);
  LCD_write_data(0x2d);
  LCD_write_data(0x2d);
  LCD_write_data(0x2d);
}
// ----- マルチパス指標表示
void LCD_put_MLT(uint8_t mlt){
  LCD_write_command(0xc9);
  LCD_put_num(mlt/10);
  mlt = mlt % 10;
  LCD_put_num(mlt);    
}
// ----- ATT値表示
void LCD_put_ATT(uint8_t att){
  LCD_write_command(0xc9);
  LCD_put_num(att/10);
  att = att % 10;
  LCD_put_num(att);    
}
// ----- プリセット番号表示
void LCD_put_P(uint8_t p){
  LCD_write_command(0x80);
  LCD_write_data(0x50);
  LCD_put_num(p/PRESET_CH_MAX + 1);
  LCD_put_num(p%PRESET_CH_MAX + 1);
}
// ----- 実際に受信しているプリセット番号と異なるため
// ----- バンク選択後 Ch番号を[-]でマスク
// ----- 直前のバンクと同じ場合にはマスクしない
void LCD_put_bank(uint8_t bank){
  LCD_write_command(0x80);
  LCD_write_data(0x50);
  LCD_put_num(bank);
  LCD_write_data(0x2d);
}
// ----- バンク選択時カーソルブリンクON
void LCD_blink_bank(){
  LCD_write_command(0x80);
  LCD_write_data(0x50);
  LCD_write_command(0x81);
  LCD_write_command(0x0d);
}
// ----- プリセット書き込み待ち時カーソルブリングON
void LCD_blink_Ch(){
  LCD_write_command(0x82);
  LCD_write_command(0x0d);
}
//----- カーソルブリンクOFF
void LCD_blink_off(){
  LCD_write_command(0x0c);
}
// ----- AM受信に先立ってRSQフォーマットをAM用にする
void LCD_start_AM(){
  static char AM_str2[] ="R   S   A   C";
  uint8_t i;
  LCD_write_command(0xC0);
  for (i=0;i<sizeof(AM_str2);i++){
    LCD_write_data(AM_str2[i]);
  }
}
// ----- FM受信に先立ってRSQフォーマットをFM用にする
void LCD_start_FM(){
  static char FM_str2[] ="R   S   M   B";
  uint8_t i;

  LCD_write_command(0xC0);
  for (i=0;i<sizeof(FM_str2);i++){
    LCD_write_data(FM_str2[i]);
  }
}
// ----- ファインチューニングモードでは 周波数の右に < を表示
void LCD_put_Mode(){
    LCD_write_command(0x89);
  if (fine_tuning){
    LCD_write_data(0x3c);
  } else {
    LCD_write_data(0x20);
  }
}
// ----- 音量表示 -----
void LCD_put_VOL(uint8_t vol){
  LCD_write_command(0x8a);
  LCD_put_num(vol/10);
  vol = vol % 10;
  LCD_put_num(vol);
}
// ----- AF MUTE表示 -----
void LCD_put_MUTE(){
  LCD_write_command(0x8a);
  LCD_write_data(0x2d);
  LCD_write_data(0x2d);
}
// ----- AM direct 枠表示 -----
void LCD_put_AM_direct(){
  LCD_write_command(0x85);
  LCD_write_data(0x5f);
  LCD_write_data(0x5f);
  LCD_write_data(0x5f);
  LCD_write_data(0x5f);
}
// ----- AM direct 枠表示 -----
void LCD_put_FM_direct(){
  LCD_write_command(0x84);
  LCD_write_data(0x5f);
  LCD_write_data(0x5f);
  LCD_write_data(0x2e);
  LCD_write_data(0x5f);
  LCD_write_data(0x20);
}
//----------- LCD関連ここまで ---------------------------------------
// ============ Si4730 基本機能関数 ここから==========================
// ----- Query Library ID -------------------------------------------
void si4730_power_up_id(){
  Serial.println("Query Library ID");
  si4730_write16(ARG23, 0x500);
  si4730_write16(CMD, POWER_UP_Q);
  delay(100);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_read(RESP23,resp2,resp3);
  si4730_read(RESP45,resp4,resp5);
  si4730_read(RESP67,resp6,resp7);
  Serial.print("PN: ");
  Serial.println(resp1);
  Serial.print("FWMAJOR: ");
  Serial.println((char)resp2);
  Serial.print("FWMINOR: ");
  Serial.println((char)resp3);
  Serial.print("Resp4(RESERVED): ");
  Serial.println(resp4, HEX);
  Serial.print("Resp5(RESERVED): ");
  Serial.println(resp5, HEX);
  Serial.print("CHIPREV: ");
  Serial.println((char)resp6);
  Serial.print("LIBRARYID: ");
  Serial.println(resp7, HEX);  
}
// ----- Get Rev ----------------------------------------------------
void si4730_get_rev(){
  Serial.println("Get Rev");
  si4730_write16(CMD, GET_REV);
  delay(100);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_read(RESP23,resp2,resp3);
  si4730_read(RESP67,resp6,resp7);
  si4730_read(RESP89,resp8,resp9);
  Serial.print("PN: ");
  Serial.println(resp1);
  Serial.print("FWMAJOR: ");
  Serial.println((char)resp2);
  Serial.print("FWMINOR: ");
  Serial.println((char)resp3);
  Serial.print("PATCH: ");
  Serial.println(si4730_read16(RESP45), HEX);
  Serial.print("CMPMAJOR: ");
  Serial.println((char)resp6);
  Serial.print("CMPMINOR: ");
  Serial.println((char)resp7);
  Serial.printf("CHIPREV: ");
  Serial.println((char)resp8);
}
// ----- プロパティ書き込み ------------------------------------------
void si4730_set_property(uint16_t prop, uint16_t propd) {
  si4730_write16(ARG23,prop);
  si4730_write16(ARG45,propd);
  si4730_write16(CMD,SET_PROPERTY);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  delay(10);
}
// ----- POWER DOWN -------------------------------------------------
void si4730_power_down(){
  si4730_write16(CMD, POWER_DPWN);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
}
// ----- AM 受信開始 -------------------------------------------------
void si4730_power_up_am(){
  digitalWrite(HARD_MUTE, HIGH);
  si4730_write16(ARG23, 0x0500);
  si4730_write16(CMD, POWER_UP_AM);
  start_time = millis();
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  af_mute = false;
  Tuner_config.band = AM;
  set_width_am(AM_config.width);
  si4730_set_property(0x4000,(uint16_t)Tuner_config.vol);
  si4730_set_property(0x3100,AM_DEEMPHASIS);
  si4730_set_property(0x3400,AM_SEEK_BAND_BOTTOM);
  si4730_set_property(0x3401,AM_SEEK_BAND_TOP);
  si4730_set_property(0x3402,AM_SEEK_FREQ_SPACING);
  si4730_set_property(0x3403,AM_SEEK_TUNE_SNR_THRESHOLD);
  si4730_set_property(0x3404,AM_SEEK_TUNE_RSSI_THRESHOLD);
  LCD_put_VOL(Tuner_config.vol);
  LCD_start_AM();
  LCD_put_Mode();
  while ((millis() - start_time) < DELAY_AFTER_POWER_UP);
  digitalWrite(HARD_MUTE, LOW);
}
//----- FM 受信開始 --------------------------------------------------
void si4730_power_up_fm(){
  digitalWrite(HARD_MUTE, HIGH);
  si4730_write16(ARG23, 0x0500);
  si4730_write16(CMD, POWER_UP_FM);
  start_time = millis();
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  Tuner_config.band = FM;
  af_mute = false;
  set_width_fm(FM_config.width);
  si4730_set_property(0x4000,(uint16_t)Tuner_config.vol);
  si4730_set_property(0x1100,FM_DEEMPHASIS);
  si4730_set_property(0x1400,FM_SEEK_BAND_BOTTOM);
  si4730_set_property(0x1401,FM_SEEK_BAND_TOP);
  si4730_set_property(0x1402,FM_SEEK_FREQ_SPACING);
  si4730_set_property(0x1403,FM_SEEK_TUNE_SNR_THRESHOLD);
  si4730_set_property(0x1404,FM_SEEK_TUNE_RSSI_THRESHOLD);
  si4730_set_property(0x1800,FM_BLEND_RSSI_STEREO_THRESHOLD);
  si4730_set_property(0x1801,FM_BLEND_RSSI_MONO_THRESHOLD);
  si4730_set_property(0x1802,FM_BLEND_RSSI_ATTACK_RATE);
  si4730_set_property(0x1803,FM_BLEND_RSSI_RELEASE_RATE);
  si4730_set_property(0x1804,FM_BLEND_SNR_STEREO_THRESHOLD);
  si4730_set_property(0x1805,FM_BLEND_SNR_MONO_THRESHOLD);
  si4730_set_property(0x1806,FM_BLEND_SNR_ATTACK_RATE);
  si4730_set_property(0x1807,FM_BLEND_SNR_RELEASE_RATE);
  si4730_set_property(0x1808,FM_BLEND_MULTIPATH_STEREO_THRESHOLD);
  si4730_set_property(0x1809,FM_BLEND_MULTIPATH_MONO_THRESHOLD);
  si4730_set_property(0x180A,FM_BLEND_MULTIPATH_ATTACK_RATE);
  si4730_set_property(0x180B,FM_BLEND_MULTIPATH_RELEASE_RATE);
  LCD_put_VOL(Tuner_config.vol);
  LCD_start_FM();
  LCD_put_Mode();
  while ((millis() - start_time) < DELAY_AFTER_POWER_UP);
  digitalWrite(HARD_MUTE, LOW);
}
// ----- AM 周波数指定チューニング -----------------------------------
// この関数自体は AM(MW)/SW/LW 関係なく使えるつもりだったがチップ非対応
void si4730_tune_freq_am(uint16_t am_freq, uint16_t cap){
  if(am_freq > FREQ_MAX_MW){
    cap = 1;                   // SW受信時は CAP=1を指定
  }
// チューニングの直前に、SW制限を外す隠しパラメータを叩いたが効果なかった
//si4730_set_property(0x3103, 0x7800); // AVC MAX GAINをSW用に解放
//si4730_set_property(0x3102, 0x0001); // 入力をFMピン(SW対応)に強制切り替え
  si4730_write16(ARG23, am_freq);
  si4730_write16(ARG45, cap);
  si4730_write16(CMD, AM_TUNE_FREQ);
  delay(DELAY_AFTER_TUNE);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_write16(CMD, GET_INT_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_write16(CMD, AM_TUNE_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x81) != 0x81);
  am_freq = si4730_read16(RESP23);
  LCD_put_AM_freq(am_freq);
  LCD_put_AM_cap(si4730_read16(RESP67));
  cap = si4730_read16(RESP67);
}
// ----- FM 周波数指定チューニング ------------------------------------
void si4730_tune_freq_fm(uint16_t fm_freq){
  si4730_write16(ARG23, fm_freq);
  si4730_write16(ARG45, 0);
  si4730_write16(CMD, FM_TUNE_FREQ);
  delay(DELAY_AFTER_TUNE);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_write16(CMD, GET_INT_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_write16(CMD, FM_TUNE_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x81) != 0x81);
  LCD_put_FM_freq(si4730_read16(RESP23));
}
//==================== Si4730制御基本関数ここまで ====================
//===== loop()からリモコンデコード結果により飛んでくる
// プリセット呼び出し
bool tuner_preset_call(uint8_t ch){
  Ch_config config;
  EEPROM.get((offsetof(Eeprom_Memory_Map, presets) + ch * sizeof(Ch_config)), config);
  if ((config.band == FM) && (config.frequency >= FREQ_MIN_FM && config.frequency <= FREQ_MAX_FM)){        //preset is FM
    if (Tuner_config.band == AM){
      si4730_power_down();
      si4730_power_up_fm();
    }
    FM_config = config;
    Tuner_config.preset_ch_fm = ch;
    FM_config.preset = true;
    Tuner_config.bank = ch / PRESET_CH_MAX +1;
    set_width_fm(FM_config.width);
    si4730_tune_freq_fm(FM_config.frequency);
    LCD_put_P(ch);
    return true;
  } else if ((config.band == AM) && config.frequency >= FREQ_MIN_AM && config.frequency <= FREQ_MAX_AM){ //preset is AM
    if (Tuner_config.band == FM){
      si4730_power_down();
      si4730_power_up_am();
    }
    AM_config = config;
    Tuner_config.preset_ch_am = ch;
    AM_config.preset = true;
    Tuner_config.bank = ch / PRESET_CH_MAX +1;
    set_width_am(AM_config.width);
    si4730_tune_freq_am(AM_config.frequency,0);
    LCD_put_P(ch);
    return true;
  } else {
    ch_empty = true;
    return false;
  }
}
//---------------------------------------------------------------------------------
// プリセットUP
void tuner_preset_up(){
  uint8_t ch;
  uint8_t ch_org;
  bool preset;
  if(Tuner_config.band == AM){
    ch = Tuner_config.preset_ch_am;
    preset = AM_config.preset;
  } else {
    ch = Tuner_config.preset_ch_fm;
    preset = FM_config.preset;
  }
  if(preset){
    ch++;
    if (ch == PRESET_BANK_MAX*PRESET_CH_MAX){
      ch = 0;
    }
  }
  ch_org = ch;
  while(! tuner_preset_call(ch)){
    ch++;
    if (ch == PRESET_BANK_MAX*PRESET_CH_MAX){
      ch = 0;
    }
    if(ch == ch_org){
      break;
    }
  }
}
//---------------------------------------------------------------------------------
// プリセットDOWN
void tuner_preset_down(){
  uint8_t ch;
  uint8_t ch_org;
  bool preset;
  if(Tuner_config.band == AM){
    ch = Tuner_config.preset_ch_am;
    preset = AM_config.preset;
  } else {
    ch = Tuner_config.preset_ch_fm;
    preset = FM_config.preset;
  }
  if(preset){
    if(ch == 0){
      ch = PRESET_BANK_MAX*PRESET_CH_MAX;
    }
    ch--;
  }
  ch_org = ch;
  while(! tuner_preset_call(ch)){
    if (ch == 0){
      ch = PRESET_BANK_MAX*PRESET_CH_MAX;
    }
    ch--;
    if(ch == ch_org){
      break;
    }
  }
}
//---------------------------------------------------------------------------------
// プリセット書込
void tuner_preset_write(){
  uint8_t ch;
  uint8_t key_code;
  uint32_t start_time = millis(); 
  constexpr uint32_t TIMEOUT_MS = 5000; // 5秒タイムアウト
  LCD_blink_Ch();
  while (1) {
    if ((millis() - debounce_time)< DEBOUNCE_PERIOD){
      TinyIRReceiverData.justWritten = false; // デバウンス
    }
    if (TinyIRReceiverData.justWritten) {
      TinyIRReceiverData.justWritten = false; // フラグクリア
      debounce_time = millis();
      key_code = translateIR(TinyIRReceiverData.Address, TinyIRReceiverData.Command); 
      if (key_code <= PRESET_CH_MAX) { 
          ch = PRESET_CH_MAX * (Tuner_config.bank - 1) + key_code -1;
        if (Tuner_config.band == AM){
          Tuner_config.preset_ch_am = ch;
          AM_config.preset = true;
          EEPROM.put((offsetof(Eeprom_Memory_Map, presets) + ch * sizeof(AM_config)), AM_config);
          Tuner_config.bank = ch / PRESET_CH_MAX +1;
          LCD_put_P(ch);
        } else if (Tuner_config.band == FM){
          Tuner_config.preset_ch_fm = ch;
          FM_config.preset = true;
          EEPROM.put((offsetof(Eeprom_Memory_Map, presets) + ch * sizeof(FM_config)), FM_config);
          Tuner_config.bank = ch / PRESET_CH_MAX +1;
          LCD_put_P(ch);
        }
        break; 
      } else {
        break;
      }
    }
      if (millis() - start_time > TIMEOUT_MS){
      break;
    }
  }
  LCD_blink_off();
}
//---------------------------------------------------------------------------------
// バンク切替
void tuner_bank_change(){
  uint8_t bank;
  uint32_t start_time = millis(); 
  constexpr uint32_t TIMEOUT_MS = 5000; // 5秒タイムアウト
  LCD_blink_bank();
  while (1) {
    if ((millis() - debounce_time)< DEBOUNCE_PERIOD){
      TinyIRReceiverData.justWritten = false; // デバウンス
    }
    if (TinyIRReceiverData.justWritten) {
      TinyIRReceiverData.justWritten = false; // フラグクリア
      debounce_time = millis();
      bank = translateIR(TinyIRReceiverData.Address, TinyIRReceiverData.Command); 
      if (bank <= PRESET_BANK_MAX) { 
        if (Tuner_config.bank != bank){
          Tuner_config.bank = bank;
          LCD_put_bank(Tuner_config.bank);
        }
        break;
      }
    }
    if (millis() - start_time > TIMEOUT_MS){
      break;
    }
  }
  LCD_blink_off();
}
//---------------------------------------------------------------------------------
//----- AM周波数直打取得 -----
void tuner_direct_am() {
  uint16_t target_freq = 0;
  uint8_t digit_count = 0;
  uint8_t key_code;
  uint32_t start_time = millis(); 
  constexpr uint32_t TIMEOUT_MS = 5000; // 5秒タイムアウト
  // 4桁入力を受け付ける
  LCD_put_AM_direct();
  LCD_write_command(0x85);
  LCD_write_command(0x0d);
  while (digit_count < 4 && (millis() - start_time < TIMEOUT_MS))  {
    if ((millis() - debounce_time)< DEBOUNCE_PERIOD){
      TinyIRReceiverData.justWritten = false; // デバウンス
    }
    if (TinyIRReceiverData.justWritten) {
      TinyIRReceiverData.justWritten = false; // フラグクリア
      debounce_time = millis();
      key_code = translateIR_N(TinyIRReceiverData.Address, TinyIRReceiverData.Command); 
      if (key_code < 10) { 
        if(digit_count == 0){
          if (key_code >= 5){
            digit_count++;
            LCD_write_data(0x20);
          } else if (key_code > 1){
            break;
          }
        }
        target_freq = (target_freq * 10) + key_code;
        digit_count++;
        LCD_put_num(key_code);
        start_time = millis(); // タイムアウトをリフレッシュ
      } else {
        break;
      }
    }
  }
  if (target_freq >= FREQ_MIN_AM && target_freq <= FREQ_MAX_AM) {
    AM_config.frequency = target_freq;
    AM_config.preset = false; 
    si4730_tune_freq_am(AM_config.frequency, 0); 
    LCD_put_bank(Tuner_config.bank);
  }
  LCD_write_command(0x0c);
  LCD_put_AM_freq(AM_config.frequency);
}
//------------------------------------------
//----- FM周波数直打取得 -----
void tuner_direct_fm() {
  uint16_t target_freq = 0;
  uint8_t digit_count = 0;
  uint8_t key_code;
  uint32_t start_time = millis(); 
  constexpr uint32_t TIMEOUT_MS = 5000; // 5秒タイムアウト
  // 3桁入力を受け付ける
  LCD_put_FM_direct();
  LCD_write_command(0x84);
  LCD_write_command(0x0d);
  while (digit_count < 3 && (millis() - start_time < TIMEOUT_MS)) {
    if ((millis() - debounce_time)< DEBOUNCE_PERIOD){
      TinyIRReceiverData.justWritten = false; // デバウンス
    }
    if (TinyIRReceiverData.justWritten) {
      TinyIRReceiverData.justWritten = false; // フラグクリア
      debounce_time = millis();
      key_code = translateIR_N(TinyIRReceiverData.Address, TinyIRReceiverData.Command); 
      if (key_code < 10) { 
        if((digit_count == 0) && (key_code < 7)){
          break;
        }
        target_freq = (target_freq * 10) + key_code;
        digit_count++;
        LCD_put_num(key_code);
        if (digit_count == 2){
          LCD_write_data(0x2e);
        }
        start_time = millis(); // タイマーリフレッシュ
      } else {
        break;
      }
    }
  }
  target_freq *= 10;
    if (target_freq >= FREQ_MIN_FM && target_freq <= FREQ_MAX_FM) {
    FM_config.frequency = target_freq;
    FM_config.preset = false; 
    si4730_tune_freq_fm(FM_config.frequency); 
    LCD_put_bank(Tuner_config.bank);
  }
  LCD_write_command(0x0c);
  LCD_put_FM_freq(FM_config.frequency);
}
//------------------------------------------------------------------------------
void tuner_direct(){
  if (Tuner_config.band == AM){
    tuner_direct_am();
  } else {
    tuner_direct_fm();
  }
}
//----- MUTE -----
void tuner_mute(){
  if(af_mute){
//  digitalWrite(HARD_MUTE, LOW);
  si4730_set_property(0x4001,0);
  af_mute = false;
    LCD_put_VOL(Tuner_config.vol);
  }
  else{
//  digitalWrite(HARD_MUTE, HIGH);
  si4730_set_property(0x4001,3);
  af_mute = true;
  LCD_put_MUTE();
  }
}
//----- VOL + -----
void tuner_vol_up(){
  if(Tuner_config.vol < MAX_VOL){
    Tuner_config.vol++ ;
    LCD_put_VOL(Tuner_config.vol);
    si4730_set_property(0x4000,(uint16_t)Tuner_config.vol);
  }
}
//----- VOL - -----
void tuner_vol_down(){
  if(Tuner_config.vol > 0){
    Tuner_config.vol-- ;
    LCD_put_VOL(Tuner_config.vol);
    si4730_set_property(0x4000,(uint16_t)Tuner_config.vol);
  }
}
//----- AM帯域セット -----
void set_width_am(uint8_t width_index){
  if (width_index > WIDTH_INDEX_MAX_AM){
    width_index = WIDTH_INDEX_MAX_AM;
  }
  si4730_set_property(AM_CHANNEL_FILTER,(uint16_t)AM_WIDTH[width_index].index);
  LCD_put_AM_width(AM_WIDTH[width_index].value);
}
//----- FM帯域セット -----
void set_width_fm(uint8_t width_index){
  if (width_index > WIDTH_INDEX_MAX_FM){
    width_index = WIDTH_INDEX_MAX_FM;
  }
  si4730_set_property(FM_CHANNEL_FILTER,(uint16_t)FM_WIDTH[width_index].index);
  LCD_put_FM_width(FM_WIDTH[width_index].value);
}
//----------- 帯域UP -----------------------
void tuner_width_up(){
  if (Tuner_config.band == AM){      //----- AM帯域セット -----
    if (AM_config.width < WIDTH_INDEX_MAX_AM){
      AM_config.width++;
    }
    set_width_am(AM_config.width);
  }
  else if(Tuner_config.band == FM){      //----- FM帯域セット -----
    if (FM_config.width < WIDTH_INDEX_MAX_FM){
      FM_config.width++;
    }
    set_width_fm(FM_config.width);
  }
}
//----------- 帯域DOWN -----------------------
void tuner_width_down(){
  if (Tuner_config.band == AM){
    if (AM_config.width > WIDTH_INDEX_MIN_AM){
      AM_config.width--;
    }
    set_width_am(AM_config.width);
  }
  else if(Tuner_config.band == FM){
    if (FM_config.width > WIDTH_INDEX_MIN_FM){
      FM_config.width--;
    }
    set_width_fm(FM_config.width);
  }
}
//----- 十字キー左 --------------------------
void tuner_left(){
  if (fine_tuning){
    tuner_width_down();
  } else {
    tuner_preset_down();
  }
}
//----- 十字キー右 --------------------------
void tuner_right(){
  if (fine_tuning){
    tuner_width_up();
  } else {
    tuner_preset_up();
  }
}
//----------- バンド切替 -----------------------
void tuner_band_change(){
  if (Tuner_config.band == AM){
    Tuner_config.band = FM;
    si4730_power_down();
    si4730_power_up_fm();
    if (FM_config.preset){
      tuner_preset_call(Tuner_config.preset_ch_fm);
    } else {
      LCD_put_bank(Tuner_config.bank);
      si4730_tune_freq_fm(FM_config.frequency);
    }
  }
  else if(Tuner_config.band == FM){
    Tuner_config.band = AM;
    si4730_power_down();
    si4730_power_up_am();
    if (AM_config.preset){
      tuner_preset_call(Tuner_config.preset_ch_am);
    } else {
      LCD_put_bank(Tuner_config.bank);
      si4730_tune_freq_am(AM_config.frequency,0);
    }
  }
}
//------------ 1 step 周波数UP --------------------
void tuner_up(){
  if (Tuner_config.band == AM){
    if (fine_tuning){
      AM_config.frequency ++;
    } else {      
      AM_config.frequency += FREQ_STEP_AM;
      AM_config.frequency = (AM_config.frequency / FREQ_STEP_AM)*FREQ_STEP_AM;
    }
    if (AM_config.frequency > FREQ_MAX_AM){
      AM_config.frequency = FREQ_MIN_AM;
    }
    AM_config.preset = false;
    LCD_put_bank(Tuner_config.bank);
    si4730_tune_freq_am(AM_config.frequency,0);
  }
  else if(Tuner_config.band == FM){
    if (fine_tuning){
      FM_config.frequency ++;
      } else {
      FM_config.frequency += FREQ_STEP_FM;
      FM_config.frequency = (FM_config.frequency / FREQ_STEP_FM)*FREQ_STEP_FM;
      }
    if (FM_config.frequency > FREQ_MAX_FM){
      FM_config.frequency = FREQ_MIN_FM;
    }
    FM_config.preset = false;
    LCD_put_bank(Tuner_config.bank);
    si4730_tune_freq_fm(FM_config.frequency);
  }
}
//------------ 1 step 周波数down --------------------
void tuner_down(){
  if (Tuner_config.band == AM){
    if (fine_tuning){
      AM_config.frequency --;
    } else {      
      AM_config.frequency -= FREQ_STEP_AM;
      AM_config.frequency = ((AM_config.frequency + FREQ_STEP_AM - 1) / FREQ_STEP_AM)*FREQ_STEP_AM;
    }
    if (AM_config.frequency < FREQ_MIN_AM){
      AM_config.frequency = FREQ_MAX_AM;
    }
    AM_config.preset = false;
    LCD_put_bank(Tuner_config.bank);
    si4730_tune_freq_am(AM_config.frequency,0);
  }
  else if(Tuner_config.band == FM){
    if (fine_tuning){
      FM_config.frequency --;
      } else {
      FM_config.frequency -= FREQ_STEP_FM;
      FM_config.frequency = ((FM_config.frequency + FREQ_STEP_FM - 1) / FREQ_STEP_FM)*FREQ_STEP_FM;
      }
    if (FM_config.frequency < FREQ_MIN_FM){
      FM_config.frequency = FREQ_MAX_FM;
    }
    FM_config.preset = false;
    LCD_put_bank(Tuner_config.bank);
    si4730_tune_freq_fm(FM_config.frequency);
  }
}
//----- Tuning Mode 変更 -----
void tuner_mode_change(){
  fine_tuning = !fine_tuning;
  LCD_put_Mode();
}
//----- 強制モノラル-----
void tuner_force_mono(){
  if (Tuner_config.band == FM){
    if (FM_config.stereo){
      FM_config.stereo = false;
      si4730_set_property(0x1800,127);
      si4730_set_property(0x1801,127);
      si4730_set_property(0x1804,127);
      si4730_set_property(0x1805,127);
      si4730_set_property(0x1808,0);
      si4730_set_property(0x1809,0);
    } else {
      FM_config.stereo = true;
      si4730_set_property(0x1800,FM_BLEND_RSSI_STEREO_THRESHOLD);
      si4730_set_property(0x1801,FM_BLEND_RSSI_MONO_THRESHOLD);
      si4730_set_property(0x1804,FM_BLEND_SNR_STEREO_THRESHOLD);
      si4730_set_property(0x1805,FM_BLEND_SNR_MONO_THRESHOLD);
      si4730_set_property(0x1808,FM_BLEND_MULTIPATH_STEREO_THRESHOLD);
      si4730_set_property(0x1809,FM_BLEND_MULTIPATH_MONO_THRESHOLD);
    }
  }
}
//==========  RSQ 表示 ====================================
// ----- LOOP()から定期的に呼ばれる
// ----- AM RSQ 表示 -----
void tuner_rsq_am(){
  if(ch_empty){
    LCD_put_AM_freq(AM_config.frequency);
   ch_empty = false;
  }
  si4730_write16(CMD, AM_RSQ_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_read(RESP45,resp4,resp5);
  LCD_put_RSSI(resp4);
  LCD_put_SNR(resp5);
  si4730_write16(CMD, AM_AGC_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_read(RESP23,resp2,resp3);
  LCD_put_ATT(resp2);
}
//----- FM RSQ 表示
void tuner_rsq_fm(){
  if(ch_empty){
    LCD_put_FM_freq(FM_config.frequency);
   ch_empty = false;
  }
  si4730_write16(CMD, FM_RSQ_STATUS);
  while ((si4730_read(STATUS,status,resp1) & 0x80) == 0);
  si4730_read(RESP23,resp2,resp3);
  si4730_read(RESP45,resp4,resp5);
  si4730_read(RESP67,resp6,resp7);
  LCD_put_RSSI(resp4);
  LCD_put_SNR(resp5);
  LCD_put_MLT(resp6);
  if (resp3 & 0x80){
    if (FM_config.stereo){
      LCD_put_BLEND(resp3 & 0x7f);
    } else {
      LCD_put_force_MONO();
    }
  } else {
    if (FM_config.stereo){
      LCD_put_MONO();
    } else {
      LCD_put_force_MONO2();
    }
  }
}
//----- RSQ 表示 -----
void tuner_rsq(){
  if (Tuner_config.band == AM){
    tuner_rsq_am();
  }
  else if(Tuner_config.band == FM){
    tuner_rsq_fm();
  }
}
//===================================================================
// ラストステーションメモリ試験用
void test_last_save(){
  Serial.println("last station memory write.");
  EEPROM.put((offsetof(Eeprom_Memory_Map, Tuner_config)), Tuner_config);
  EEPROM.put((offsetof(Eeprom_Memory_Map, last_station_am)), AM_config);
  EEPROM.put((offsetof(Eeprom_Memory_Map, last_station_fm)), FM_config);
}
//==================== SETUP ========================================
void setup(){
  Ch_config EEP_Ch_config;
  Sys_config EEP_Sys_config;
  const char Title[] ="Si4730-M02 Tuner";
  const char Copyright[] ="(c) Gazelle8087 ";
  uint8_t i;

  DIDR1  = 0b00000010; 
  pinMode(RSTB_PIN, OUTPUT);
  PORTD &= ~(1 << RSTB_PIN);  // RESET

  pinMode(SENB_PIN, OUTPUT);
  pinMode(SDIO_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(HARD_MUTE, OUTPUT);

  PORTD |= (1 << RSTB_PIN);  // RESET RELEASE

  pinMode(GPIO2_PIN, INPUT_PULLUP);
  pinMode(SPI_CS,    INPUT_PULLUP);
  pinMode(SPI_MOSI,  INPUT_PULLUP);
  pinMode(SPI_MISO,  INPUT_PULLUP);
  pinMode(SPI_SCK,   INPUT_PULLUP);
  pinMode(XTAL1,     INPUT_PULLUP);
  pinMode(XTAL2,     INPUT_PULLUP);
  pinMode(A0,        INPUT_PULLUP);
  pinMode(A1,        INPUT_PULLUP);
  pinMode(A2,        INPUT_PULLUP);
  pinMode(A3,        OUTPUT);
  digitalWrite(A3,HIGH);

  Wire.begin();
  LCD_init();

  LCD_write_command(0x80);
  for (i=0;i<sizeof(Title);i++){
    LCD_write_data(Title[i]);
  }
   LCD_write_command(0xC0);
  for (i=0;i<sizeof(Copyright);i++){
    LCD_write_data(Copyright[i]);
  }

  Serial.begin(9600);
  Serial.println("Si4730-M02 tuner");
  Serial.println("(c) 2026 Gazelle8087");

// EEP ラストステーションメモリ 初期化チェック
// AM保存周波数チェック NGならソース初期値のまま
  EEPROM.get((offsetof(Eeprom_Memory_Map, last_station_am)), EEP_Ch_config);
//  if(EEP_Ch_config.frequency <= FREQ_MAX_AM && EEP_Ch_config.frequency >= FREQ_MIN_AM){
  if((EEP_Ch_config.band == AM) && (EEP_Ch_config.frequency <= FREQ_MAX_AM && EEP_Ch_config.frequency >= FREQ_MIN_AM)){
    AM_config = EEP_Ch_config;
  }

// FM保存周波数チェック NGならソース初期値のまま
  EEPROM.get((offsetof(Eeprom_Memory_Map, last_station_fm)), EEP_Ch_config);
//  if(EEP_Ch_config.frequency <= FREQ_MAX_FM && EEP_Ch_config.frequency >= FREQ_MIN_FM){
  if((EEP_Ch_config.band == FM) && (EEP_Ch_config.frequency <= FREQ_MAX_FM && EEP_Ch_config.frequency >= FREQ_MIN_FM)){
    FM_config = EEP_Ch_config;
  }
// 音量チェック NGなら最大値を設定
  EEPROM.get((offsetof(Eeprom_Memory_Map, Tuner_config)), EEP_Sys_config);
  if(EEP_Sys_config.vol > MAX_VOL){
    EEP_Sys_config.vol = MAX_VOL;
  }
// バンク番号チェック NGなら1にする
  if(EEP_Sys_config.bank == 0 || EEP_Sys_config.bank > PRESET_BANK_MAX){
    EEP_Sys_config.bank = 1;
  }
// AMプリセットchチェック NGなら0にする
  if(EEP_Sys_config.preset_ch_am >= PRESET_BANK_MAX * PRESET_CH_MAX){
    EEP_Sys_config.preset_ch_am = 0;
    AM_config.preset = false;
  }
// FMプリセットchチェック NGなら0にする
  if(EEP_Sys_config.preset_ch_fm >= PRESET_BANK_MAX * PRESET_CH_MAX){
    EEP_Sys_config.preset_ch_fm = 0;
    FM_config.preset = false;
  }
  Tuner_config = EEP_Sys_config;
// EEP 読み出し・チェック、初期化おわり

  delay(1000);

  LCD_write_command(0x80);
  for (int i=0;i<16;i++){
    LCD_write_data(0x20);
  }
  LCD_write_command(0xc0);
  for(int i=0;i<16;i++){
    LCD_write_data(0x20);
  }

//  si4730_power_up_id();
  if (Tuner_config.band == AM){
    si4730_power_up_am();
//    si4730_get_rev();
    if (AM_config.preset){
      tuner_preset_call(Tuner_config.preset_ch_am);
    } else {
      LCD_put_bank(Tuner_config.bank);
      si4730_tune_freq_am(AM_config.frequency,0);
    }
  }
  else if(Tuner_config.band == FM){
    si4730_power_up_fm();
//    si4730_get_rev();
    if (FM_config.preset){
      tuner_preset_call(Tuner_config.preset_ch_fm);
    } else {
      LCD_put_bank(Tuner_config.bank);
      si4730_tune_freq_fm(FM_config.frequency);
    }
  }
// TynyIRreceiver 割り込み設定
  initPCIInterruptForTinyReceiver();

// アナログコンパレータによる電源断割込み準備

  DIDR1 = (1 << AIN1D);                 // D7 デジタル入力バッファ無効化
  ADCSRB &= ~(1 << ACME);               // アナログコンパレーターマルチプレクサ停止（D7入力指定）
  ACSR = ((1 << ACBG) | (1 << ACIS1));  // 内部1.1V基準 立下りエッジ指定（まだACIEは立てない）
  delay(1);                             // ACO 同期待ち（誤割込み抑止）
  ACSR |= (1 << ACI);                   // ★ 割込みフラグをクリア
  delay(1);                             // ACO 同期待ち（誤割込み抑止）
  ACSR |= (1 << ACIE);                  // 割り込み許可
}
//==================== END OF SETUP ================================
//==================================================================
// 共通の電源断処理関数
// インライン展開（コード埋め込み）を強制する
static inline void save_and_halt() __attribute__((always_inline));
void save_and_halt() {

//  PORTC &= 0b11110111;                 // オシロ観察用 本番では消す
  TWCR = 0;                            // I2C停止
  PORTC &= ~((1 << PC4) | (1 << PC5)); // SDA, SCLを強制LOW
  CLKPR = 0b10000000;                  // クロック落とした方がEEP処理終わった時の
  CLKPR = 0b00000011;                  // 残存電圧が高かった
  //PRR   = 0b11111111;                  // 全ペリフェラル停止

// EEPROMへの保存処理                   // UI開発中は書き込みしない本番では戻す
  EEPROM.put(offsetof(Eeprom_Memory_Map, Tuner_config), Tuner_config);
//  PORTC |= 0b00001000; 		       // オシロ観察用 本番では消す
  EEPROM.put(offsetof(Eeprom_Memory_Map, last_station_am), AM_config);
//  PORTC &= 0b11110111;                 // オシロ観察用 本番では消す
  EEPROM.put(offsetof(Eeprom_Memory_Map, last_station_fm), FM_config);

  // 省電力設定
  wdt_disable();     // ウォッチドッグ停止
  PRR = 0b11111111;  // 全ペリフェラル停止

//  PORTC |= 0b00001000; // オシロ観察用 本番では消す
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  cli();             // 割り込みを完全禁止
  sleep_cpu();       // ここで安全にBORリセットを待つ
}
//==================== ISR =========================================
// ISR_NAKED を指定して、割り込みのオーバーヘッド（PUSH/POP）をゼロにする
ISR(ANALOG_COMP_vect, ISR_NAKED) {
  save_and_halt();        //       インライン展開される
                          // 戻らないのでreti()不要
}
//==================== LOOP ========================================
void loop(){

  uint8_t key_code;

  if (ACSR & (1 << ACO)) { //電源断ポーリング（割り込み併用）
    save_and_halt(); 
  }

  if ((millis() - debounce_time)< DEBOUNCE_PERIOD){
    TinyIRReceiverData.justWritten = false; // デバウンス
  }

  if (TinyIRReceiverData.justWritten) {
    TinyIRReceiverData.justWritten = false; // フラグクリア
    debounce_time = millis();  

    key_code = translateIR(TinyIRReceiverData.Address, TinyIRReceiverData.Command); 

    switch(key_code){
      case 99:  break;                       // 他リモコン混信
      case 200: tuner_mute();         break; // [電源]
      case 201: tuner_rsq();          break; // [入力切替]
      case 202: tuner_vol_up();       break; // [Vol+]
      case 203: tuner_band_change();  break; // [放送切替]
      case 204: tuner_vol_down();     break; // [Vol-]
      case 205: tuner_up();           break; // [ ▲ ]
      case 206: tuner_left();         break; // [ < ]
      case 207: tuner_mode_change();  break; // [決定]
      case 208: tuner_right();        break; // [ > ]
      case 209: tuner_down();         break; // [ ▼ ]
      case 210: tuner_direct();       break; // [ d ]
      case 211: break;                       // [ 戻る ]
      case 212: tuner_bank_change();  break; // [ メニュー ]
      case 213: tuner_preset_write(); break; // [番組表]
      case 214: tuner_force_mono();   break; // [ 青 ]
      case 215: break;                       // [ 赤 ]
      case 216: break;                       // [ 緑 ]
      case 217: test_last_save();     break; // [ 黄 ]

      default: 
      if(key_code <= PRESET_CH_MAX){          // [CHボタン] key_code = CH番号
        if(!tuner_preset_call(PRESET_CH_MAX * (Tuner_config.bank - 1) + key_code -1)){
          LCD_empty();
        }
      }
      break;
    }   // End Case

    if (ACSR & (1 << ACO)) {  //電源断ポーリング（割り込み併用）
      save_and_halt(); 
    }
  }

  TinyIRReceiverData.justWritten = false; // デバウンス

  if (millis() - start_time > SQR_PERIOD){
    tuner_rsq();
    start_time = millis();
  }

  if (ACSR & (1 << ACO)) {  //電源断ポーリング（割り込み併用）
    save_and_halt(); 
  }
}
//==================== END OF LOOP ===================================
