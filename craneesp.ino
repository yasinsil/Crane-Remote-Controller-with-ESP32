// ESP32 Vinç Kodu v3.5 - Dual Zone PWM Control v1.0
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

// GÜVENLİK PİNLERİ
#define TXS_OE_PIN 16
#define HEARTBEAT_LED_PIN 17

// WATCHDOG AYARLARI
#define WATCHDOG_TIMEOUT 2
#define WATCHDOG_CHECK_INTERVAL 1500

// DUAL ZONE PWM AYARLARI
#define MIN_SOLENOID_PWM 75       // Yeni solenoid açılma eşiği
#define PRECISION_ZONE_MAX_PWM 110 // Precision zone sonu
#define MAX_PWM 190               // Maksimum PWM

#define PRECISION_ZONE_END 60     // Joystick %60'ına kadar precision zone
#define SOFT_START_TIME_MS 75     // 0→75 PWM için soft start süresi

// PWM VE DİGİTAL PİNLER
const int mosfetPins[6] = {2, 4, 5, 18, 19, 23};
const int sepetSolPin = 14;
const int sepetSagPin = 26;
const int motorStartPin = 33;
const int motorStopPin = 13;

// JOYSTICK MERKEZ VE DEADZONE
const int16_t solJoyMerkezX = 13199;
const int16_t solJoyMerkezY = 13032;
const int16_t sagJoyMerkezX = 13031;
const int16_t sagJoyMerkezY = 13578;
const int16_t deadzoneTolerans = 1000;

// DUAL ZONE RAMPA YAPILAR
struct DualZoneRampa {
  int currentPwm;
  int targetPwm;
  unsigned long lastUpdate;
  bool softStartActive;
  unsigned long softStartTime;
};

// 6 PWM kanalı için dual zone rampa state'leri
DualZoneRampa dzStates[6];

// PWM DEĞERLERİ
int pwmDegerler[6] = {0, 0, 0, 0, 0, 0};
int gercekPwmDegerler[6] = {0, 0, 0, 0, 0, 0};
unsigned long sonRampaGuncelleme = 0;

// DİGİTAL KONTROL DEĞERLERİ
bool sepetSol = false;
bool sepetSag = false;
bool motorStart = false;
bool motorStop = false;

// BUTON KOMUT DEĞİŞKENLERİ
bool kumandaStartKomutu = false;
bool kumandaStopKomutu = false;

// SOFT START RAMPA DEĞİŞKENLERİ (Digital pinler için - değişmez)
struct SoftStart {
  int currentPwm;
  unsigned long startTime;
  bool active;
};

SoftStart sepetSolSoft = {0, 0, false};
SoftStart sepetSagSoft = {0, 0, false};
SoftStart motorStartSoft = {0, 0, false};
SoftStart motorStopSoft = {0, 0, false};

// VERİ YAPILARI
typedef struct {
  int16_t solJoyX;
  int16_t solJoyY;
  int16_t sagJoyX;
  int16_t sagJoyY;
  bool baglanti;
  bool startButon;
  bool stopButon;
  uint32_t paketID;
  uint32_t checksum;
} KumandaVeri;

typedef struct {
  bool vinçHazir;
  uint32_t alinanPaketID;
  uint8_t hataKodu;
} VinçCevap;

// GLOBAL DEĞİŞKENLER
int16_t solJoyX = solJoyMerkezX, solJoyY = solJoyMerkezY;
int16_t sagJoyX = sagJoyMerkezX, sagJoyY = sagJoyMerkezY;
bool baglantiDurumu = false;
unsigned long sonVeriZamani = 0;
unsigned long paketSayisi = 0;
unsigned long sonMemoryCheck = 0;
uint32_t sonAlinanPaketID = 0;
uint32_t beklenenPaketID = 1;

uint8_t yetkiliKumandaMAC[6] = {0x78, 0x1C, 0x3C, 0xA7, 0xFD, 0xF4};
bool kumandaPeerEklendi = false;

// GÜVENLIK DEĞİŞKENLERİ
unsigned long sonHeartbeat = 0;
unsigned long kesinRestartSuresi = 1800000;
bool acilDurdurmaAktif = false;
uint32_t consecutiveErrors = 0;
unsigned long sistemBaslangic = 0;

unsigned long sonWatchdogCheck = 0;
uint32_t watchdogResetSayisi = 0;

bool reconnectionModu = false;
unsigned long reconnectionBaslangic = 0;

bool operasyonAktif = false;
unsigned long sonAktivite = 0;
unsigned long operasyonBaslangic = 0;
unsigned long guvenliDurumSayaci = 0;

// DUAL ZONE PWM FONKSİYONLARI

// Dual zone setup
void dualZoneSetup() {
  for (int i = 0; i < 6; i++) {
    dzStates[i].currentPwm = 0;
    dzStates[i].targetPwm = 0;
    dzStates[i].lastUpdate = millis();
    dzStates[i].softStartActive = false;
    dzStates[i].softStartTime = 0;
  }
  Serial.println("*** DUAL ZONE PWM CONTROL v1.0 AKTİF ***");
  Serial.printf("Solenoid PWM: %d | Precision Zone: %d-%d (%d%%) | Power Zone: %d-%d (%d%%)\n", 
                MIN_SOLENOID_PWM, MIN_SOLENOID_PWM, PRECISION_ZONE_MAX_PWM, PRECISION_ZONE_END,
                PRECISION_ZONE_MAX_PWM, MAX_PWM, 100-PRECISION_ZONE_END);
}

// Joystick percentage'ı PWM'e çevir - DUAL ZONE MAPPING
int dualZonePwmMapping(int joystickPercent) {
  if (joystickPercent <= 0) return 0;
  
  if (joystickPercent <= PRECISION_ZONE_END) {
    // PRECISION ZONE: %0-60 → PWM 75-110 (Linear)
    return map(joystickPercent, 0, PRECISION_ZONE_END, MIN_SOLENOID_PWM, PRECISION_ZONE_MAX_PWM);
  } else {
    // POWER ZONE: %60-100 → PWM 110-190 (Linear) 
    return map(joystickPercent, PRECISION_ZONE_END, 100, PRECISION_ZONE_MAX_PWM, MAX_PWM);
  }
}

// Dual zone rampalama
int dualZoneRampaHesapla(int kanal, int targetPwm) {
  if (kanal < 0 || kanal >= 6) return 0;
  
  DualZoneRampa *dz = &dzStates[kanal];
  unsigned long simdikiZaman = millis();
  
  // Zaman kontrolü - 10ms'de bir güncelle
  if (simdikiZaman - dz->lastUpdate < 10) {
    return dz->currentPwm;
  }
  
  // SELECTIVE SLOWDOWN Counter - Her kanal için ayrı
  static int rampaCounters[6] = {0, 0, 0, 0, 0, 0};
  
  // HEDEF DEĞİŞİMİ TESPİTİ - Counter Reset
  static int oncekiHedefler[6] = {-1, -1, -1, -1, -1, -1};
  if (oncekiHedefler[kanal] != targetPwm) {
    Serial.printf("K%d: Hedef değişti %d→%d (mevcut:%d)\n", kanal, dz->targetPwm, targetPwm, dz->currentPwm);
    rampaCounters[kanal] = 0;  // COUNTER RESET!
    oncekiHedefler[kanal] = targetPwm;
  }
  
  dz->targetPwm = targetPwm;
  dz->lastUpdate = simdikiZaman;
  
  // DURUM 1: Hedef 0 - Normal rampalı iniş
  if (targetPwm == 0) {
    dz->softStartActive = false;
    rampaCounters[kanal] = 0;  // İniş başlarken counter reset
    
    if (dz->currentPwm <= 0) {
      dz->currentPwm = 0;
      return 0;
    }
    
    // Normal iniş rampalama
    int rampaHizi;
    if (dz->currentPwm > 120) rampaHizi = 2;
    else if (dz->currentPwm > 40) rampaHizi = 1;  
    else rampaHizi = 1;
    
    dz->currentPwm = max(0, dz->currentPwm - rampaHizi);
    return dz->currentPwm;
  }
  
  // DURUM 2: 0'dan başlangıç - SOFT START (0→75)
  if (dz->currentPwm == 0 && targetPwm > 0) {
    dz->softStartActive = true;
    dz->softStartTime = simdikiZaman;
    dz->currentPwm = 1;
    rampaCounters[kanal] = 0;  // Soft start başlarken counter reset
    Serial.printf("K%d: Soft start başladı (0→75)\n", kanal);
    return 1;
  }
  
  // DURUM 2.5: Düşük PWM'den tekrar başlangıç - MICRO SOFT START
  if (dz->currentPwm > 0 && dz->currentPwm < MIN_SOLENOID_PWM && targetPwm > 0) {
    // 75 altındayken tekrar joystick itildi - direkt 75'e çık
    rampaCounters[kanal] = 0;  // Micro soft start başlarken counter reset
    dz->currentPwm += 3;  // Hızlı çıkış
    if (dz->currentPwm >= MIN_SOLENOID_PWM) {
      dz->currentPwm = MIN_SOLENOID_PWM;
      Serial.printf("K%d: Micro soft start tamamlandı (%d PWM)\n", kanal, MIN_SOLENOID_PWM);
    }
    return dz->currentPwm;
  }
  
  // DURUM 3: SOFT START devam ediyor (0→75)
  if (dz->softStartActive && dz->currentPwm < MIN_SOLENOID_PWM) {
    unsigned long elapsed = simdikiZaman - dz->softStartTime;
    
    if (elapsed <= SOFT_START_TIME_MS) {
      dz->currentPwm = map(elapsed, 0, SOFT_START_TIME_MS, 0, MIN_SOLENOID_PWM);
    } else {
      dz->currentPwm = MIN_SOLENOID_PWM;
      dz->softStartActive = false;
      rampaCounters[kanal] = 0;  // Soft start bitince counter reset
      Serial.printf("K%d: Soft start tamamlandı (%d PWM)\n", kanal, MIN_SOLENOID_PWM);
    }
    return dz->currentPwm;
  }
  
  // DURUM 4: Smooth Rampa (75+ PWM) - SELECTIVE SLOWDOWN
  if (dz->currentPwm >= MIN_SOLENOID_PWM) {
    int fark = targetPwm - dz->currentPwm;
    
    if (fark == 0) return dz->currentPwm;
    
    // Counter artır
    rampaCounters[kanal]++;
    
    int rampaHizi = 0;
    
    // Her 2. seferde artır (20ms etkisi)
    if (rampaCounters[kanal] % 2 == 0) {
      rampaHizi = 1;
    } else {
      rampaHizi = 0;  // Bu sefer artırma
    }
    
    if (fark > 0) {
      dz->currentPwm = min(targetPwm, dz->currentPwm + rampaHizi);
    } else {
      dz->currentPwm = max(targetPwm, dz->currentPwm - rampaHizi);
    }
    
    // DEBUG: Counter takılma kontrolü
    if (rampaCounters[kanal] > 1000) {
      Serial.printf("K%d: COUNTER OVERFLOW! Reset ediliyor\n", kanal);
      rampaCounters[kanal] = 0;
    }
    
    return dz->currentPwm;
  }
  
  return dz->currentPwm;
}

// Digital pinler için eski soft start (değişmez)
int softStartRampa(SoftStart &soft, bool target) {
  if (target && !soft.active && soft.currentPwm == 0) {
    soft.startTime = millis();
    soft.active = true;
    soft.currentPwm = 5;
    return 5;
  }
  
  if (target && soft.active) {
    unsigned long elapsed = millis() - soft.startTime;
    
    if (elapsed <= 25) {
      soft.currentPwm = map(elapsed, 0, 25, 5, 100);
    } else if (elapsed <= 50) {
      soft.currentPwm = map(elapsed, 25, 50, 100, 255);
    } else {
      soft.currentPwm = 255;
      soft.active = false;
    }
    return soft.currentPwm;
  }
  
  if (target && !soft.active) {
    soft.currentPwm = 255;
    return 255;
  }
  
  if (!target) {
    soft.active = false;
    soft.currentPwm = 0;
    return 0;
  }
  
  return soft.currentPwm;
}

// DUAL ZONE RAMPA GÜNCELLEME
void rampaGuncelle() {
  // PWM pinleri için dual zone rampalama
  for (int i = 0; i < 6; i++) {
    gercekPwmDegerler[i] = dualZoneRampaHesapla(i, pwmDegerler[i]);
    ledcWrite(mosfetPins[i], gercekPwmDegerler[i]);
  }
  
  // Digital pinler için eski soft start rampa
  int sepetSolPwm = softStartRampa(sepetSolSoft, sepetSol);
  int sepetSagPwm = softStartRampa(sepetSagSoft, sepetSag);
  int motorStartPwm = softStartRampa(motorStartSoft, motorStart);
  int motorStopPwm = softStartRampa(motorStopSoft, motorStop);
  
  ledcWrite(sepetSolPin, sepetSolPwm);
  ledcWrite(sepetSagPin, sepetSagPwm);
  ledcWrite(motorStartPin, motorStartPwm);
  ledcWrite(motorStopPin, motorStopPwm);
}

// DEADZONE KONTROLLARI (değişmez)
bool solJoyYDeadzoneDa() { return abs(solJoyY - solJoyMerkezY) <= deadzoneTolerans; }
bool solJoyXDeadzoneDa() { return abs(solJoyX - solJoyMerkezX) <= deadzoneTolerans; }
bool sagJoyYDeadzoneDa() { return abs(sagJoyY - sagJoyMerkezY) <= deadzoneTolerans; }
bool sagJoyXDeadzoneDa() { return abs(sagJoyX - sagJoyMerkezX) <= deadzoneTolerans; }
bool tumJoystickDeadzoneDa() { return solJoyXDeadzoneDa() && solJoyYDeadzoneDa() && sagJoyXDeadzoneDa() && sagJoyYDeadzoneDa(); }

// WATCHDOG SETUP (değişmez)
void watchdogSetup() {
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,
    .trigger_panic = true
  };
  esp_err_t init_result = esp_task_wdt_init(&wdt_config);
  if (init_result == ESP_ERR_INVALID_STATE) {
      Serial.println("Watchdog zaten başlatılmış, yeniden yapılandırılıyor...");
      if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
          Serial.println("HATA: Watchdog yeniden yapılandırılamadı!");
          return;
      }
  } else if (init_result != ESP_OK) {
      Serial.println("HATA: Watchdog başlatılamadı!");
      return;
  }
  if (esp_task_wdt_add(NULL) != ESP_OK) {
    Serial.println("HATA: Mevcut görev watchdog'a eklenemedi!");
  }
  Serial.printf("*** WATCHDOG AKTİF: %d saniye timeout ***\n", WATCHDOG_TIMEOUT);
}

// WATCHDOG YÖNETİMİ (değişmez)
void watchdogYonetimi() {
  if (reconnectionModu) {
    esp_task_wdt_reset();
    return;
  }
  unsigned long simdikiZaman = millis();
  if (simdikiZaman - sonWatchdogCheck >= WATCHDOG_CHECK_INTERVAL) {
    bool watchdogBesle = true;
    if (ESP.getFreeHeap() < 20000) {
      watchdogBesle = false;
      Serial.printf("WATCHDOG: Critical memory (%d bytes)\n", ESP.getFreeHeap());
    }
    if (consecutiveErrors > 5) {
      watchdogBesle = false;
      Serial.printf("WATCHDOG: ESP-NOW stack problemi (%lu hata)\n", consecutiveErrors);
    }
    if (watchdogBesle) {
      esp_task_wdt_reset();
    } else {
      Serial.printf("*** WATCHDOG BESLENMEDI - %d saniye sonra RESET! ***\n", WATCHDOG_TIMEOUT);
    }
    sonWatchdogCheck = simdikiZaman;
  }
}

// RAMPALI KAPATMA (Dual zone uyumlu)
void rampaliKapatma() {
    Serial.println("Yumuşak kapatma rampası başlatılıyor...");
    for (int i = 0; i < 6; i++) {
        pwmDegerler[i] = 0;
    }
    sepetSol = false;
    sepetSag = false;
    motorStart = false;
    motorStop = false;
    
    unsigned long kapatmaBaslangic = millis();
    while (millis() - kapatmaBaslangic < 5000) {
        rampaGuncelle();
        esp_task_wdt_reset();
        bool hepsiKapali = true;
        for (int i = 0; i < 6; i++) {
            if (gercekPwmDegerler[i] > 0) {
                hepsiKapali = false;
                break;
            }
        }
        if (hepsiKapali && sepetSolSoft.currentPwm == 0 && sepetSagSoft.currentPwm == 0) {
            Serial.println("Rampalı kapatma tamamlandı.");
            return;
        }
        delay(10);
    }
    Serial.println("HATA: Rampalı kapatma zaman aşımına uğradı!");
}

// ACİL DURDURMA (değişmez)
void acilDurdur(const char* neden) {
  if (acilDurdurmaAktif) return;
  acilDurdurmaAktif = true;
  Serial.printf("ACIL DURDURMA: %s\n", neden);
  rampaliKapatma();
  digitalWrite(TXS_OE_PIN, LOW);
  Serial.println("Sistem guvenli duruma alindi");
  for (int i = 0; i < 10; i++) {
    digitalWrite(HEARTBEAT_LED_PIN, HIGH); delay(100);
    digitalWrite(HEARTBEAT_LED_PIN, LOW); delay(100);
    esp_task_wdt_reset();
  }
  if (strcmp(neden, "KUMANDA BAGLANTI TIMEOUT") == 0) {
    Serial.println("BAGLANTI KOPTU - RECONNECTION MODU");
    reconnectionModu = true;
    reconnectionBaslangic = millis();
    acilDurdurmaAktif = false;
    digitalWrite(TXS_OE_PIN, HIGH);
  }
}

// RESTART KONTROLÜ (değişmez)
bool kesinRestartKontrol() {
  unsigned long uptime = millis() - sistemBaslangic;
  if (uptime >= kesinRestartSuresi) {
    if (tumJoystickDeadzoneDa()) {
      Serial.println("30 DAKIKA + DEADZONE = RESTART!");
      esp_task_wdt_deinit();
      delay(1000);
      ESP.restart();
      return false;
    } else {
      static unsigned long sonUyari = 0;
      if (millis() - sonUyari > 10000) {
        Serial.printf("%lu DAKIKA GECTI! Joystick'i deadzone'a getirin\n", uptime / 60000);
        sonUyari = millis();
      }
    }
  }
  return true;
}

// GÜVENLİK KONTROLÜ (değişmez)
bool guvenlikKontrol() {
  unsigned long simdikiZaman = millis();
  if (simdikiZaman - sonHeartbeat > 100) {
    digitalWrite(HEARTBEAT_LED_PIN, !digitalRead(HEARTBEAT_LED_PIN));
    sonHeartbeat = simdikiZaman;
  }
  if (ESP.getFreeHeap() < 20000) {
    acilDurdur("KRITIK MEMORY AZLIGI");
    return false;
  }
  if (!kesinRestartKontrol()) {
    return false;
  }
  if (baglantiDurumu && (simdikiZaman - sonVeriZamani > 1000)) {
    baglantiDurumu = false;
    acilDurdur("KUMANDA BAGLANTI TIMEOUT");
    return false;
  }
  return true;
}

// OPERASYON GÜVENLİĞİ (değişmez)
bool operasyonGuvenligi() {
  bool hareketVar = !tumJoystickDeadzoneDa();
  if (hareketVar) {
    if (!operasyonAktif) {
      operasyonAktif = true;
      operasyonBaslangic = millis();
      Serial.println("OPERASYON BASLADI");
    }
    sonAktivite = millis();
    guvenliDurumSayaci = 0;
  } else {
    if (operasyonAktif) {
      guvenliDurumSayaci++;
    }
    if (guvenliDurumSayaci > 10) {
      if (operasyonAktif) {
        operasyonAktif = false;
        Serial.printf("OPERASYON BITTI (%lu saniye)\n", (millis() - operasyonBaslangic) / 1000);
      }
    }
  }
  return true;
}

// CEVAP GÖNDER (değişmez)
void cevapGonder(const uint8_t* kumandaMAC, uint8_t hataKodu) {
  if (!kumandaPeerEklendi) {
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, yetkiliKumandaMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      kumandaPeerEklendi = true;
    } else {
      return;
    }
  }
  VinçCevap cevap;
  cevap.vinçHazir = true;
  cevap.alinanPaketID = sonAlinanPaketID;
  cevap.hataKodu = hataKodu;
  esp_err_t result = esp_now_send(yetkiliKumandaMAC, (uint8_t*)&cevap, sizeof(cevap));
  if (result != ESP_OK) {
    consecutiveErrors++;
    if (consecutiveErrors > 5) {
      acilDurdur("ESP-NOW CEVAP HATALARI COKLU");
    }
    if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      kumandaPeerEklendi = false;
    }
  } else {
    consecutiveErrors = 0;
  }
}

// VERİ ALMA CALLBACK (değişmez)
void veriAlindi(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (memcmp(recv_info->src_addr, yetkiliKumandaMAC, 6) != 0) return;
  if (len != sizeof(KumandaVeri)) {
    cevapGonder(recv_info->src_addr, 2);
    return;
  }
  KumandaVeri* gelenVeri = (KumandaVeri*)incomingData;
  uint32_t hesaplananChecksum = gelenVeri->solJoyX ^ gelenVeri->solJoyY ^ gelenVeri->sagJoyX ^ gelenVeri->sagJoyY ^ 
                                gelenVeri->paketID ^ (gelenVeri->startButon ? 1 : 0) ^ (gelenVeri->stopButon ? 2 : 0);
  if (gelenVeri->checksum != hesaplananChecksum) {
    cevapGonder(recv_info->src_addr, 2);
    return;
  }
  
  solJoyX = gelenVeri->solJoyX;
  solJoyY = gelenVeri->solJoyY;
  sagJoyX = gelenVeri->sagJoyX;
  sagJoyY = gelenVeri->sagJoyY;
  baglantiDurumu = gelenVeri->baglanti;
  
  kumandaStartKomutu = gelenVeri->startButon;
  kumandaStopKomutu = gelenVeri->stopButon;
  
  if (kumandaStartKomutu || kumandaStopKomutu) {
    Serial.printf("KUMANDA BUTON: Start=%s Stop=%s\n", 
                  kumandaStartKomutu ? "AKTIF" : "pasif",
                  kumandaStopKomutu ? "AKTIF" : "pasif");
  }
  sonVeriZamani = millis();
  sonAlinanPaketID = gelenVeri->paketID;
  beklenenPaketID = gelenVeri->paketID + 1;
  paketSayisi++;
  if (reconnectionModu) {
    reconnectionModu = false;
    Serial.println("RECONNECTION BASARILI");
  }
  cevapGonder(recv_info->src_addr, 0);
}

// DUAL ZONE JOYSTICK KONTROL
void joystickKontrol() {
  if (acilDurdurmaAktif || !baglantiDurumu) {
    for (int i = 0; i < 6; i++) { pwmDegerler[i] = 0; }
    sepetSol = false;
    sepetSag = false;
    motorStart = false;
    motorStop = false;
    return;
  }
  
  // BUTON KOMUTLARI
  motorStart = kumandaStartKomutu;
  motorStop = kumandaStopKomutu;
  
  // Sol Y ekseni (Yukarı/Aşağı) - DUAL ZONE CONTROL
  if (solJoyY > (solJoyMerkezY + deadzoneTolerans)) {
    int joystickPercent = map(solJoyY, solJoyMerkezY + deadzoneTolerans, 26450, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[0] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[1] = 0;
  } else if (solJoyY < (solJoyMerkezY - deadzoneTolerans)) {
    int joystickPercent = map(solJoyY, solJoyMerkezY - deadzoneTolerans, 0, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[1] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[0] = 0;
  } else {
    pwmDegerler[0] = 0;
    pwmDegerler[1] = 0;
  }
  
  // Sol X ekseni (Sol/Sağ) - DUAL ZONE CONTROL
  if (solJoyX > (solJoyMerkezX + deadzoneTolerans)) {
    int joystickPercent = map(solJoyX, solJoyMerkezX + deadzoneTolerans, 26450, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[3] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[2] = 0;
  } else if (solJoyX < (solJoyMerkezX - deadzoneTolerans)) {
    int joystickPercent = map(solJoyX, solJoyMerkezX - deadzoneTolerans, 0, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[2] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[3] = 0;
  } else {
    pwmDegerler[2] = 0;
    pwmDegerler[3] = 0;
  }
  
  // Sağ Y ekseni - DUAL ZONE CONTROL
  if (sagJoyY > (sagJoyMerkezY + deadzoneTolerans)) {
    int joystickPercent = map(sagJoyY, sagJoyMerkezY + deadzoneTolerans, 26450, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[4] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[5] = 0;
  } else if (sagJoyY < (sagJoyMerkezY - deadzoneTolerans)) {
    int joystickPercent = map(sagJoyY, sagJoyMerkezY - deadzoneTolerans, 0, 0, 100);
    joystickPercent = constrain(joystickPercent, 0, 100);
    
    pwmDegerler[5] = dualZonePwmMapping(joystickPercent);
    pwmDegerler[4] = 0;
  } else {
    pwmDegerler[4] = 0;
    pwmDegerler[5] = 0;
  }
  
  // Sağ X ekseni - Sepet dönüş (SOFT START - değişmez)
  int sagXYuzde = 0;
  if (abs(sagJoyX - sagJoyMerkezX) > deadzoneTolerans) { 
    sagXYuzde = (sagJoyX > sagJoyMerkezX) ? 
      map(sagJoyX, sagJoyMerkezX + deadzoneTolerans, 26450, 0, 100) : 
      map(sagJoyX, sagJoyMerkezX - deadzoneTolerans, 0, 0, 100); 
    sagXYuzde = constrain(sagXYuzde, 0, 100); 
  }
  
  if (sagJoyX > (sagJoyMerkezX + deadzoneTolerans) && sagXYuzde >= 70) { 
    sepetSol = false; 
    sepetSag = true;
  } else if (sagJoyX < (sagJoyMerkezX - deadzoneTolerans) && sagXYuzde >= 70) { 
    sepetSol = true; 
    sepetSag = false;
  } else { 
    sepetSol = false; 
    sepetSag = false; 
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  sistemBaslangic = millis();
  
  Serial.println("*** DUAL ZONE PWM CONTROL v1.0 - TEST SİSTEMİ ***");
  Serial.println("Precision Zone: %0-60 → PWM 75-110 | Power Zone: %60-100 → PWM 110-190");
  watchdogSetup();
  
  pinMode(TXS_OE_PIN, OUTPUT);
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(TXS_OE_PIN, LOW);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);
  
  // PWM pinleri setup
  for (int i = 0; i < 6; i++) { 
    pinMode(mosfetPins[i], OUTPUT); 
    digitalWrite(mosfetPins[i], LOW); 
    ledcAttach(mosfetPins[i], 1000, 8); 
    ledcWrite(mosfetPins[i], 0); 
  }
  
  // Digital pinler setup
  pinMode(sepetSolPin, OUTPUT);
  pinMode(sepetSagPin, OUTPUT);
  pinMode(motorStartPin, OUTPUT);
  pinMode(motorStopPin, OUTPUT);
  
  ledcAttach(sepetSolPin, 1000, 8);
  ledcAttach(sepetSagPin, 1000, 8);
  ledcAttach(motorStartPin, 1000, 8);
  ledcAttach(motorStopPin, 1000, 8);
  
  ledcWrite(sepetSolPin, 0);
  ledcWrite(sepetSagPin, 0);
  ledcWrite(motorStartPin, 0);
  ledcWrite(motorStopPin, 0);
  
  // DUAL ZONE SISTEMI BAŞLAT
  dualZoneSetup();
  
  Serial.println("3 saniye guvenlik bekleme...");
  for (int i = 3; i >= 1; i--) {
    Serial.printf("%d...\n", i);
    digitalWrite(HEARTBEAT_LED_PIN, HIGH); delay(500);
    digitalWrite(HEARTBEAT_LED_PIN, LOW); delay(500);
    esp_task_wdt_reset();
  }
  digitalWrite(TXS_OE_PIN, HIGH);
  Serial.println("TXS'ler aktif - DUAL ZONE PWM CONTROL HAZIR!");
  
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print("Vinc MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW baslatma hatasi!");
    while(1);
  }
  esp_now_register_recv_cb(veriAlindi);
  Serial.println("*** DUAL ZONE PWM CONTROL SİSTEMİ HAZIR! ***");
  esp_task_wdt_reset();
}

void loop() {
  watchdogYonetimi();
  if (!guvenlikKontrol()) {
    delay(100);
    return;
  }
  operasyonGuvenligi();
  
  // Memory kontrol
  static unsigned long sonSaglikKontrol = 0;
  if (millis() - sonSaglikKontrol > 5000) {
    static uint32_t oncekiHeap = 0;
    uint32_t freeHeap = ESP.getFreeHeap();
    if (oncekiHeap > 0) {
      int32_t heapDegisim = (int32_t)freeHeap - (int32_t)oncekiHeap;
      if (heapDegisim < -5000) {
        Serial.printf("HIZLI MEMORY AZALMA: %ld bytes\n", heapDegisim);
        consecutiveErrors++;
      }
      if (consecutiveErrors > 3) {
        acilDurdur("MEMORY LEAK TESPITI");
        return;
      }
    }
    oncekiHeap = freeHeap;
    sonSaglikKontrol = millis();
  }
  
  if (millis() - sonMemoryCheck > 300000) {
    Serial.printf("Free Heap: %d bytes | Uptime: %lu dakika\n", ESP.getFreeHeap(), millis()/60000);
    sonMemoryCheck = millis();
  }

  if (baglantiDurumu && !acilDurdurmaAktif && !reconnectionModu) {
    joystickKontrol();
    if (millis() - sonRampaGuncelleme >= 10) {
      rampaGuncelle();
      sonRampaGuncelleme = millis();
    }
    
    // DUAL ZONE DEBUG OUTPUT
    static unsigned long sonDebug = 0;
    if (millis() - sonDebug > 500) {
      char opDurum = operasyonAktif ? 'A' : 'D';
      Serial.printf("Joy: Sol(%d,%d) Sag(%d,%d) | ", solJoyX, solJoyY, sagJoyX, sagJoyY);
      Serial.printf("HEDEF: [%d,%d,%d,%d,%d,%d] | ", 
                    pwmDegerler[0], pwmDegerler[1], pwmDegerler[2], 
                    pwmDegerler[3], pwmDegerler[4], pwmDegerler[5]);
      Serial.printf("GERÇEK: [%d,%d,%d,%d,%d,%d] | ", 
                    gercekPwmDegerler[0], gercekPwmDegerler[1], gercekPwmDegerler[2], 
                    gercekPwmDegerler[3], gercekPwmDegerler[4], gercekPwmDegerler[5]);
      
      // Zone bilgisi
      for (int i = 0; i < 6; i++) {
        if (pwmDegerler[i] > 0) {
          char zoneChar = (pwmDegerler[i] <= PRECISION_ZONE_MAX_PWM) ? 'P' : 'W';
          char softChar = dzStates[i].softStartActive ? 'S' : '-';
          if (i == 0) Serial.printf("ZONE: ");
          Serial.printf("K%d:%c%c ", i, zoneChar, softChar);
        }
      }
      
      Serial.printf("| Soft: Sol(14):%d Sag(26):%d | %c | PaketID:%lu\n", 
                    sepetSolSoft.currentPwm, sepetSagSoft.currentPwm, opDurum, sonAlinanPaketID);
      sonDebug = millis();
    }
  } else {
    static unsigned long sonBeklemeLog = 0;
    if (millis() - sonBeklemeLog > 3000) {
      if (reconnectionModu) {
        Serial.printf("RECONNECTION MODU - BEKLEME (%lu saniye)\n", 
                      (millis() - reconnectionBaslangic) / 1000);
      } else {
        Serial.printf("Kumanda bekleniyor... | Uptime: %lu dakika\n", millis()/60000);
      }
      sonBeklemeLog = millis();
    }
  }
  delay(5);
}
