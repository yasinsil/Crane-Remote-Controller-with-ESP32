// ESP32 Kumanda Kodu v2.1 - Stabilizasyon ve Debug Düzeltmeleri
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_task_wdt.h> // 🐕 WATCHDOG KÜTÜPHANESİ

// 🐕 WATCHDOG AYARLARI
#define WATCHDOG_TIMEOUT 2     // 2 SANİYE - Kumanda donma koruması
#define WATCHDOG_CHECK_INTERVAL 1500 // 1.5 saniyede bir besleme kontrolü

// ADS1115 nesnesi
Adafruit_ADS1115 ads;

// I2C pinleri
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 50000  // 50kHz

// BUTON PİNLERİ - 10K PULL-UP İLE
#define START_BUTON_PIN 34  // Motor Start butonu
#define STOP_BUTON_PIN 35   // Motor Stop butonu

// Vinç ESP32'nin MAC adresi
uint8_t receiverMAC[] = {0x68, 0x25, 0xDD, 0xEF, 0x11, 0x94};

// Kumanda → Vinç veri yapısı - BUTON DESTEKLİ
typedef struct {
  int16_t solJoyX;
  int16_t solJoyY;
  int16_t sagJoyX;
  int16_t sagJoyY;
  bool baglanti;
  bool startButon;  // GPIO34 - Motor Start butonu
  bool stopButon;   // GPIO35 - Motor Stop butonu
  uint32_t paketID;
  uint32_t checksum;
} KumandaVeri;

// Vinç → Kumanda cevap yapısı
typedef struct {
  bool vinçHazir;
  uint32_t alinanPaketID;
  uint8_t hataKodu;
} VinçCevap;

KumandaVeri veri;
VinçCevap sonVinçCevap;
int i2cHataCount = 0;
uint32_t paketSayaci = 1;
unsigned long sonVinçCevapZamani = 0;
bool vinçBaglantiDurumu = false;

// OTOMATIK RESTART SİSTEMİ değişkenleri
unsigned long sistemBaslangic = 0;
unsigned long restartSuresi = 3600000;  // 1 saat
bool joystickAktif = false;
unsigned long restartBeklemeBaslangic = 0;
bool restartBeklemeModu = false;

// AKILLI DEADZONE
const int16_t solJoyMerkezX = 13764;
const int16_t solJoyMerkezY = 13617;
const int16_t sagJoyMerkezX = 13662;
const int16_t sagJoyMerkezY = 13162;
const int16_t deadzoneTolerans = 600;

// 🐕 WATCHDOG SETUP
void watchdogSetup() {
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,
    .trigger_panic = true
  };
  if (esp_task_wdt_init(&wdt_config) != ESP_OK) {
      Serial.println("HATA: Watchdog başlatılamadı!");
  }
  if (esp_task_wdt_add(NULL) != ESP_OK) {
    Serial.println("HATA: Mevcut görev watchdog'a eklenemedi!");
  }
  Serial.printf("*** WATCHDOG AKTİF: %d saniye timeout ***\n", WATCHDOG_TIMEOUT);
}

// 🐕 WATCHDOG YÖNETİMİ
void watchdogYonetimi() {
    // Kumanda için watchdog'u her döngüde beslemek genellikle yeterlidir.
    // Sadece I2C hattı gibi kritik bir donanım uzun süre kilitlenirse devreye girmesi hedeflenir.
    esp_task_wdt_reset();
}

// ESP-NOW gönderim callback
void veriGonderildi(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Bu fonksiyonu basit tutmak genellikle daha iyidir.
}

// Vinç cevabı alma callback
void vinçCevabiAlindi(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(VinçCevap)) {
    return;
  }
  sonVinçCevap = *(VinçCevap*)incomingData;
  sonVinçCevapZamani = millis();
  vinçBaglantiDurumu = true;
}

// AKILLI DEADZONE KONTROLÜ
bool joystickDeadzoneDa() {
  if (abs(veri.solJoyX - solJoyMerkezX) > deadzoneTolerans || abs(veri.solJoyY - solJoyMerkezY) > deadzoneTolerans) {
    return false;
  }
  if (abs(veri.sagJoyX - sagJoyMerkezX) > deadzoneTolerans || abs(veri.sagJoyY - sagJoyMerkezY) > deadzoneTolerans) {
    return false;
  }
  return true;
}

// OTOMATIK RESTART KONTROLÜ
bool otomatikRestartKontrol() {
  unsigned long uptime = millis() - sistemBaslangic;
  if (uptime >= restartSuresi) {
    if (!restartBeklemeModu) {
      restartBeklemeModu = true;
      restartBeklemeBaslangic = millis();
      Serial.println("1 SAAT DOLDU! Restart icin joystick'leri deadzone'a getirin...");
    }
    if (restartBeklemeModu) {
      if (joystickDeadzoneDa()) {
        if (millis() - restartBeklemeBaslangic >= 5000) {
          Serial.println("5 SANIYE DEADZONE'DA = RESTART!");
          Serial.println("Watchdog devre dışı bırakılıyor ve planlı restart yapılıyor...");
          esp_task_wdt_deinit(); // Watchdog'u devre dışı bırak
          delay(1000);
          ESP.restart();
          return false;
        }
      } else {
        restartBeklemeBaslangic = millis(); // Sayacı sıfırla
      }
    }
  }
  return true;
}

// BUTON OKUMA FONKSİYONU
bool butonOku() {
  veri.startButon = !digitalRead(START_BUTON_PIN);  // LOW = basılı
  veri.stopButon = !digitalRead(STOP_BUTON_PIN);    // LOW = basılı
  
  // Debug - sadece buton basıldığında
  if (veri.startButon || veri.stopButon) {
    Serial.printf("BUTON: Start=%s Stop=%s\n", 
                  veri.startButon ? "BASILI" : "serbest",
                  veri.stopButon ? "BASILI" : "serbest");
  }
  
  return true;
}

// OPERASYON TAKİBİ
void operasyonTakip() {
  joystickAktif = !joystickDeadzoneDa();
}

// Güvenli ADS1115 okuma fonksiyonu
bool adsGuvenliceOku() {
  Wire.beginTransmission(0x48);
  if (Wire.endTransmission() != 0) {
    i2cHataCount++;
    if (i2cHataCount > 3) {
      Serial.println("I2C hard reset yapiliyor...");
      Wire.end();
      delay(50);
      Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
      Wire.setTimeOut(3000);
      delay(100);
      if (ads.begin()) {
        ads.setGain(GAIN_TWOTHIRDS);
        ads.setDataRate(RATE_ADS1115_128SPS);
        i2cHataCount = 0;
        Serial.println("Hard reset basarili");
      } else {
        Serial.println("Hard reset basarisiz!");
      }
    }
    return false;
  }
  
  veri.solJoyX = ads.readADC_SingleEnded(0);
  veri.solJoyY = ads.readADC_SingleEnded(1);
  veri.sagJoyX = ads.readADC_SingleEnded(2);
  veri.sagJoyY = ads.readADC_SingleEnded(3);
  
  i2cHataCount = 0;
  veri.baglanti = true;
  veri.paketID = paketSayaci;
  
  // Butonları oku
  butonOku();
  
  // Checksum hesapla - butonlar dahil
  veri.checksum = veri.solJoyX ^ veri.solJoyY ^ veri.sagJoyX ^ veri.sagJoyY ^ 
                  veri.paketID ^ (veri.startButon ? 1 : 0) ^ (veri.stopButon ? 2 : 0);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  sistemBaslangic = millis();
  
  Serial.println("Kumanda ESP32 V2.1 - WATCHDOG KORUMALI");
  Serial.println("RESTART KURALI: 1 saat + 5 saniye deadzone!");
  Serial.println("BUTON DESTEKLİ: GPIO34=START, GPIO35=STOP");
  
  // 🐕 WATCHDOG'U İLK İŞ OLARAK BAŞLAT
  watchdogSetup();
  
  // BUTON PİNLERİNİ BAŞLAT - INPUT_PULLUP
  pinMode(START_BUTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTON_PIN, INPUT_PULLUP);
  Serial.println("Buton pinleri hazir (10K pull-up)");
  
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
  Wire.setTimeOut(3000);
  
  if (!ads.begin()) {
    Serial.println("ADS1115 bulunamadi! Sistem durduruldu.");
    while (1) { delay(1000); }
  }
  
  ads.setGain(GAIN_TWOTHIRDS);
  ads.setDataRate(RATE_ADS1115_128SPS);
  Serial.println("ADS1115 hazir");
  
  // *** STABİL WIFI BAŞLATMA RUTİNİ ***
  Serial.println("Guvenli WiFi baslatma...");
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);
  
  Serial.print("Kumanda MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW baslatma hatasi!");
    return;
  }
  
  esp_now_register_send_cb(veriGonderildi);
  esp_now_register_recv_cb(vinçCevabiAlindi);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer ekleme hatasi!");
    return;
  }
  
  Serial.println("ESP-NOW hazir - Cift yonlu haberlesme aktif");
  Serial.println("Kumanda hazir! Gonderim baslatiliyor...");
}

void loop() {
  // 🐕 WATCHDOG'U HER DÖNGÜNÜN BAŞINDA BESLE
  watchdogYonetimi();

  if (!otomatikRestartKontrol()) {
    return;
  }
  
  operasyonTakip();
  
  static unsigned long sonOkuma = 0;
  if (millis() - sonOkuma >= 15) {
    if (adsGuvenliceOku()) {
      paketSayaci++;
    }
    sonOkuma = millis();
  }
  
  static unsigned long sonGonderim = 0;
  if (millis() - sonGonderim >= 20) {
    esp_now_send(receiverMAC, (uint8_t*)&veri, sizeof(veri));
    sonGonderim = millis();
  }
  
  if (millis() - sonVinçCevapZamani > 3000) {
    if (vinçBaglantiDurumu) {
      vinçBaglantiDurumu = false;
      Serial.println("Vinc baglantisi kesildi");
    }
  }
  
  static unsigned long sonDebug = 0;
  if (millis() - sonDebug > 2000) {
    char vinçDurum = vinçBaglantiDurumu ? 'V' : 'X';
    char joystickDurum = joystickAktif ? 'A' : 'D';
    char restartDurum = restartBeklemeModu ? 'W' : 'R';
    unsigned long uptimeDakika = (millis() - sistemBaslangic) / 60000;
    
    Serial.printf("Sol(%d,%d) Sag(%d,%d) | PktID:%lu | Vinc:%c | Joy:%c | Rst:%c | Up:%ludk\n", 
                  veri.solJoyX, veri.solJoyY, 
                  veri.sagJoyX, veri.sagJoyY, 
                  veri.paketID, vinçDurum, joystickDurum, restartDurum, uptimeDakika);
    
    // *** GERİ EKLENEN DETAYLI DEBUG BLOĞU ***
    if (vinçBaglantiDurumu) {
      Serial.printf("--> Vinc Son Paket:%lu | Hata Kodu:%d | I2C Hata:%d\n", 
                    sonVinçCevap.alinanPaketID, sonVinçCevap.hataKodu, i2cHataCount);
    }
    
    sonDebug = millis();
  }
  
  delay(5);
}
