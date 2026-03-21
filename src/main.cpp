#include <arduino.h>
#include <CANCREATE.h>
#include <ICM42688.h>
#include <LPS25HB.h>
#include <SPIflash.h>
#include <SPICREATE.h>

#define SPIFREQ 5000000 // 1200000とどっちにするのか

#define CAN_RX 47
#define CAN_TX 48
#define SCK 17
#define MISO 16
#define MOSI 18
#define ICMCS 8
#define LPSCS 7
#define FlashCS 15
#define MIN1 9
#define MIN2 10
#define led_pin 11
#define LPS25HB_WHO_AM_I 0xBD // LPS25HBの固有ID (10進数で189)

SPICREATE::SPICreate SPIC;
CAN_CREATE CAN(true);
ICM icm42688;
LPS lps;
Flash flash;

u_int8_t tx[256]; // 1ページ分
u_int8_t rx[256]; // 2ページ分

hw_timer_t *timer = NULL;
u_int8_t LPS_temp[3] = {0, 0, 0};
volatile u_int8_t LPS25_data[3] = {0, 0, 0};
int16_t ICM_temp[6];
volatile int16_t ICM_data[6];

spi_host_device_t host_in = SPI2_HOST;

can_return_t can_data;
u_int32_t can_id;
char can_cmd;
char Serial_cmd = -1;
u_int32_t start_time = 0;
u_int32_t log_time = 0;
u_int32_t flash_time = 0;
volatile int tick = 0;
int lps_sample_count1 = 0;
int icm_sample_count = 0;
int lps_sample_count2 = 0;
int lps_risyou_count = 0;
int icm_risyou_count = 0;
int lps_kaisan_count = 0;
int icm_risyou_sum = 0;
int avg_ax = 0;
int avg_ay = 0;
int avg_az = 0;
const int tikatika_syuuki = 200;
const int tikatika_syuuki2 = 4000;
volatile int tikatika_count = 0;
volatile int r_count = 0;
volatile int l_count = 0;
volatile int LED_count = 0;
volatile int kaisan_count = 0;
u_int32_t lps_now = 0;
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int32_t pa = 0;
int32_t pb = 0;
volatile int lps_risyou_raw = 0;
int lps_risyou_sum = 0;
int lps_risyou_before = 0;
int lps_kaisan_raw = 0;
int lps_kaisan_sum = 0;
int lps_kaisan_before = 0;
volatile bool risyou_flag = false;
volatile bool kaisan_flag = false;
volatile bool risyou_siriaru = false;
volatile int kaisan_timer = 0;
volatile bool kaisan_siriaru = false;
volatile bool standby_flag = false;
volatile bool erase_flag = false;
bool log_flag = false;
volatile bool tikatika_now = false;

void exec_can(u_int32_t, char, int);
void standby();
void raw_data();
void log_data();
void update_LED();
void update_kaisan();

IRAM_ATTR void counter()
{ // 1msで呼ばれる
  tick++;
  if (tick % 40 == 0)
  {

    lps_risyou_raw = ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
    lps_risyou_sum += lps_risyou_raw;
    lps_sample_count1++;
    if (lps_sample_count1 == 20) // 40→5に変更
    {
      lps_sample_count1 = 0;
      if ((lps_risyou_before - lps_risyou_sum / 20) >= 2) // ここを変える テストのときは4 本番は20⇐今回は0.1hPa=20だから合ってる 前回と今回が逆になっていたので調整
      {
        lps_risyou_count++;
        if (lps_risyou_count == 5)
        {
          risyou_flag = true;
          lps_risyou_count = 0;
        }
      }
      else
      {
        lps_risyou_count = 0;
      }
      lps_risyou_before = lps_risyou_sum / 20;
      lps_risyou_sum = 0;
    }
  }

  if (tick % 1 == 0) // 本番は60→1
  {
    icm_sample_count++;
    avg_ax += ICM_data[0] / 20;
    avg_ay += ICM_data[1] / 20;
    avg_az += ICM_data[2] / 20;
    if (icm_sample_count == 20)
    {
      icm_risyou_sum = sqrt(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az) * 16 / 32768;
      if (icm_risyou_sum >= 2) // 本番は4
      {
        icm_risyou_count++;
        if (icm_risyou_count == 10) // ここを変える テストのときは2 本番は50
        {
          risyou_flag = true;
          // risyoudetect();//離床検知
          icm_risyou_count = 0;
        }
      }
      else
      {
        icm_risyou_count = 0;
      }
      avg_ax = 0;
      avg_ay = 0;
      avg_az = 0;
      icm_sample_count = 0;
    }
  }

  if (tick % 20 == 0)
  {
    lps_kaisan_raw = ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
    lps_kaisan_sum += lps_kaisan_raw;
    lps_sample_count2++;         // 生データ
    if (lps_sample_count2 == 10) // 5(10にした)回の平均(40になっていたので変更)
    {
      lps_sample_count2 = 0;
      if (lps_kaisan_sum / 10 > lps_kaisan_before)
      {
        lps_kaisan_count++;
        if (lps_kaisan_count == 5) // ここを変える テストのときは4 本番は9
        {
          kaisan_flag = true; // 開傘条件クリア
          lps_kaisan_count = 0;
        }
      }
      else
      {
        lps_kaisan_count = 0;
      }
      lps_kaisan_before = lps_kaisan_sum / 10;
      lps_kaisan_sum = 0;
    }
  }
  if (risyou_flag && !kaisan_flag)
  {
    kaisan_timer++;
  }
  r_count++;
  l_count++;
  LED_count++;
  if (kaisan_flag)
  {
    kaisan_count++;
  }
}

void setup()
{
  delay(1000);
  Serial.begin(115200);

  pinMode(ICMCS, OUTPUT);
  pinMode(LPSCS, OUTPUT);
  pinMode(FlashCS, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(MIN1, OUTPUT);
  pinMode(MIN2, OUTPUT);

  digitalWrite(ICMCS, HIGH);
  digitalWrite(LPSCS, HIGH);
  digitalWrite(FlashCS, HIGH);
  digitalWrite(led_pin, HIGH);
  digitalWrite(MIN1, LOW);
  digitalWrite(MIN2, LOW);
  tikatika_now = true;

  SPIC.begin(SPI2_HOST, SCK, MISO, MOSI);

  if (CAN.begin(100E3, CAN_RX, CAN_TX, 10))
  {
    Serial.println("CAN failed");
  }
  else
  {
    Serial.println("CAN succeeded !!!!");
  }

  icm42688.begin(&SPIC, ICMCS, SPIFREQ);
  // Flashの初期化 この前にSPIの初期化を行う必要がある
  flash.begin(&SPIC, FlashCS, SPIFREQ);
  lps.begin(&SPIC, LPSCS, SPIFREQ);

  delay(3000);
  // Serial.println("start erase...");
  // flash.erase();
  // Serial.println("erase DONE!!!");
  // erase_flag = true;

  // タイマー割り込み
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &counter, false);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}

bool check = false;
int i = 0;
int j = 0;

void loop()
{
  if (Serial.available())
  {
    Serial_cmd = Serial.read();
    Serial.print("Serial cmd :");
    Serial.println(Serial_cmd);
    exec_can(0, 0, Serial_cmd);
  }
  if (CAN.available())
  {
    if (CAN.readWithDetail(&can_data))
    {
      Serial.println("CAN failed");
    }
    else
    {
      if (can_data.size > 0)
      {
        can_id = can_data.id;
        can_cmd = can_data.data[0];
        Serial.print("can id :");
        Serial.print(can_id, HEX);
        Serial.print(", ");
        Serial.print("can cmd :");
        Serial.println(can_cmd);
        exec_can(can_id, can_cmd, 0);
      }
    }
  }

  if (standby_flag == true)
  {
    standby();
  }
  if (log_flag)
  {
    if (l_count >= 500)
    {
      log_data();
      l_count = 0;
    }
  }
}
void exec_can(u_int32_t can_id, char can_cmd, int Serial_cmd)
{
  if ((can_id == 0x005 && can_cmd == 's') || Serial_cmd == 's')
  {
    digitalWrite(led_pin, HIGH);
    tikatika_now = true;
    log_flag = true;
    erase_flag = false;
    check = false;
    i = 0;
    j = 0;

    risyou_flag = false;
    risyou_siriaru = false;

    kaisan_flag = false;
    kaisan_siriaru = false;
    kaisan_timer = 0;
    kaisan_count = 0;

    lps_sample_count1 = 0;
    icm_sample_count = 0;
    lps_sample_count2 = 0;
    lps_risyou_count = 0;
    icm_risyou_count = 0;
    lps_kaisan_count = 0;
    lps_risyou_sum = 0;
    lps_kaisan_sum = 0;
    lps_risyou_before = 0;
    avg_ax = 0;
    avg_ay = 0;
    avg_az = 0;
    lps_kaisan_before = 0;
    start_time = millis();
    standby_flag = true;
  }
  if ((can_id == 0x00a && can_cmd == 'e') || Serial_cmd == 'e')
  {
    standby_flag = false;
    log_flag = false;
  }

  if ((can_id == 0x00d && can_cmd == 'p') || Serial_cmd == 'p')
  {
    kaisan_count = 0;
    kaisan_timer = 0;
    kaisan_siriaru = false;
    kaisan_flag = true;
  }
  if ((can_id == 0x00d && can_cmd == 'r') || Serial_cmd == 'r')
  {
    kaisan_flag = false;
  }

  if ((can_id == 0x011 && can_cmd == 'l') || Serial_cmd == 'l')
  {
    erase_flag = false;
    check = false;
    i = 0;
    j = 0;
    start_time = millis();
    log_flag = true;
  }
  if ((can_id == 0x01e && can_cmd == 'm') || Serial_cmd == 'm')
  {
    log_flag = false;
  }
}
void standby()
{
  if (r_count >= 500)
  {
    raw_data();
    r_count = 0;
  }

  if (!risyou_siriaru && risyou_flag)
  {
    Serial.println("risyou");
    risyou_siriaru = true;
    delay(500);
  }
  if (kaisan_timer >= 15000)
  {
    kaisan_flag = true;
    kaisan_timer = 0;
  }
  if (!kaisan_siriaru && kaisan_flag)
  {
    Serial.println("kaisan");
    kaisan_siriaru = true;
    update_LED();
    delay(500);
  }
  if (LED_count >= 500)
  {
    update_LED();
    LED_count = 0;
  }
  update_kaisan();
  if (kaisan_count >= 3000)
  {
    kaisan_flag = false;
  }
}
void raw_data()
{
  Serial.print("生データ"); // ただし遅延アリ、データは同時のもの
  Serial.print(log_time);
  Serial.print(", ");
  Serial.print(lps_now);
  Serial.print(", ");
  Serial.print(ICM_data[0]);
  Serial.print(",");
  Serial.print(ICM_data[1]);
  Serial.print(",");
  Serial.println(ICM_data[2]);
}

void log_data()
{
  if (!erase_flag)
  {
    Serial.println("start erase...");
    flash.erase();
    Serial.println("erase DONE!!!");
    erase_flag = true;
  }
    log_time = millis() - start_time;
  lps.Get(LPS_temp);
  noInterrupts();
  LPS25_data[0] = LPS_temp[0];
  LPS25_data[1] = LPS_temp[1];
  LPS25_data[2] = LPS_temp[2];
  interrupts();

  lps_now = (LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200 / 4096;

  icm42688.Get(ICM_temp);
  noInterrupts();
  ICM_data[0] = ICM_temp[0];
  ICM_data[1] = ICM_temp[1];
  ICM_data[2] = ICM_temp[2];
  interrupts();
  if (!check)
  {
    tx[i] = log_time >> 16;
    tx[i + 1] = log_time >> 8;
    tx[i + 2] = log_time;
    tx[i + 3] = LPS25_data[0];
    tx[i + 4] = LPS25_data[1];
    tx[i + 5] = LPS25_data[2];
    tx[i + 6] = ICM_data[0] >> 8;
    tx[i + 7] = ICM_data[0];
    tx[i + 8] = ICM_data[1] >> 8;
    tx[i + 9] = ICM_data[1];
    tx[i + 10] = ICM_data[2] >> 8;
    tx[i + 11] = ICM_data[2];
    i += 12;
    delay(10);
    if (i == 252)
    {
      Serial.println("hoge");
      flash.write(j, tx);
      delay(10); // 直後に読み出ししちゃうと誤った値が出る恐れがある
      // 読み込み 256byte単位で読み込みができる
      flash.read(j, rx);
      // 読み込んだデータをシリアルで表示
      i = 0;
      j += 256;
      for (int k = 0; k < 252; k += 12)
      {
        flash_time = rx[k] * 65536 + rx[k + 1] * 256 + rx[k + 2];
        pa = (rx[k + 5] << 16) | (rx[k + 4] << 8) | (rx[k + 3]);
        ax = (rx[k + 6] << 8) | (rx[k + 7]);
        ay = (rx[k + 8] << 8) | (rx[k + 9]);
        az = (rx[k + 10] << 8) | (rx[k + 11]);
        pb = pa * 200 / 4096;
        Serial.print("flashデータ");
        Serial.print(flash_time);
        Serial.print(", ");
        Serial.print(pb);
        Serial.print(", ");
        Serial.print(ax);
        Serial.print(", ");
        Serial.print(ay);
        Serial.print(", ");
        Serial.println(az);
      }
      if (j == 5 * 256)
      {
        check = true;
      }
    }
  }
}

void update_LED()
{
  if (risyou_flag && !kaisan_flag)
  {
    if (tikatika_now)
    {
      digitalWrite(led_pin, LOW);
      tikatika_now = false;
    }
    else
    {
      digitalWrite(led_pin, HIGH);
      tikatika_now = true;
    }
  }
  if (risyou_flag && kaisan_flag)
  {
    digitalWrite(led_pin, LOW);
    tikatika_now = true;
  }
}
void update_kaisan()
{
  if (kaisan_flag)
  {
    digitalWrite(MIN1, LOW);
    digitalWrite(MIN2, HIGH);
  }
  else
  {
    digitalWrite(MIN1, LOW);
    digitalWrite(MIN2, LOW);
  }
}