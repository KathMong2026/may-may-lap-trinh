/*
 * MÁY MAY LẬP TRÌNH - BƯỚC 1
 * Đồng bộ bàn X-Y (2 motor bước) với góc kim (encoder), cộng 1 solenoid chân vịt.
 *
 * Phần cứng: Arduino Uno R3 + CNC Shield V3 + 2 driver A4988 + 2 motor NEMA 17
 *            Encoder LPD3806-600BM, cảm biến quang chữ U FC-03,
 *            mạch MOSFET cách ly + solenoid 12V.
 *
 * Ý tưởng chính:
 *   - Encoder cho biết góc trục kim 0..359 độ (0 độ = kim ở điểm cao nhất).
 *   - Bàn X-Y CHỈ được dịch trong "cửa sổ" khi kim đang ở trên mặt vải
 *     (mặc định từ 300 độ qua 0 tới 60 độ).
 *   - Mỗi vòng quay của trục = đúng 1 mũi may.
 *   - Nếu cửa sổ đóng mà bàn chưa dịch xong -> báo lỗi và dừng (máy thật sẽ gãy kim).
 *
 * Mở Serial Monitor ở tốc độ 115200, chọn "Newline" hoặc "No line ending".
 * Lệnh: s = chạy mẫu, p = dừng, r = về đầu mẫu, f = nâng/hạ chân vịt,
 *       e = bật/tắt driver motor, h = trợ giúp
 */

#include <util/atomic.h>

// ================== CHÂN TRÊN CNC SHIELD V3 ==================
// Shield đã nối sẵn: X STEP = D2, Y STEP = D3, X DIR = D5, Y DIR = D6, EN = D8.
// Code dùng truy cập cổng trực tiếp cho STEP/DIR để chạy nhanh trong ngắt:
//   D2 = PD2, D3 = PD3, D5 = PD5, D6 = PD6
const uint8_t PIN_X_STEP = 2;
const uint8_t PIN_Y_STEP = 3;
const uint8_t PIN_X_DIR  = 5;
const uint8_t PIN_Y_DIR  = 6;
const uint8_t PIN_EN     = 8;    // LOW = bật driver

const uint8_t PIN_ENC_A  = 9;    // cắm vào chân tín hiệu "X-" (End stops) trên shield
const uint8_t PIN_ENC_B  = 10;   // cắm vào chân tín hiệu "Y-"
const uint8_t PIN_SOL    = 11;   // cắm vào chân tín hiệu "Z-" (D11 có PWM)
const uint8_t PIN_INDEX  = A0;   // cắm vào chân "Abort"

// ================== THÔNG SỐ CHỈNH ĐƯỢC ==================
const int16_t ENC_PPR      = 600;           // số xung/vòng ghi trên encoder
const int16_t ENC_CPR      = ENC_PPR * 4;   // giải mã cả 2 cạnh của A và B -> 2400 đếm/vòng
const int8_t  ENC_DIR      = 1;             // góc chạy ngược chiều thì đổi thành -1

const bool    INDEX_ACTIVE_HIGH = true;     // FC-03: bị che = HIGH. Ngược thì đổi false
const int16_t INDEX_ANGLE  = 0;             // góc lúc miếng chắn vừa vào khe (kim cao nhất)

const int16_t WIN_START    = 300;           // cửa sổ dịch vải mở (độ)
const int16_t WIN_END      = 60;            // cửa sổ dịch vải đóng (độ)

const long    STEPS_PER_MM = 80;            // dây đai GT2 + puly 20 răng + vi bước 1/16
const uint16_t TICK_HZ     = 8000;          // tốc độ phát xung tối đa: 8000 bước/giây

const uint8_t  SOL_KICK      = 255;         // % PWM lúc "đá" (255 = 100%)
const uint8_t  SOL_HOLD      = 80;          // PWM lúc "giữ" (80/255 ~ 31%)
const uint16_t SOL_KICK_MS   = 50;          // thời gian đá
const uint32_t SOL_MAX_ON_MS = 30000;       // tự nhả sau 30 giây cho đỡ nóng cuộn dây

// Đổi độ sang số đếm encoder
constexpr int16_t degToCount(int16_t d) { return (int16_t)((int32_t)d * ENC_CPR / 360); }
const int16_t WIN_S = degToCount(WIN_START);
const int16_t WIN_E = degToCount(WIN_END);
const int16_t IDX_C = degToCount(INDEX_ANGLE);

// ================== FILE MẪU MAY ==================
// Đơn vị dx, dy: 0,1 mm. "lap" = lặp lại dòng này bao nhiêu lần.
// Mỗi dòng STITCH = 1 mũi = 1 vòng quay trục kim.
enum Cmd : uint8_t { STITCH, FOOT_UP, FOOT_DOWN, END };
struct Buoc { int16_t dx; int16_t dy; uint8_t cmd; uint8_t lap; };

const Buoc PATTERN[] = {
  {   0,   0, FOOT_DOWN, 1 },   // hạ chân vịt
  {  25,   0, STITCH,    8 },   // 8 mũi 2,5 mm sang phải  -> cạnh 20 mm
  {   0,  25, STITCH,    8 },   // 8 mũi lên trên
  { -25,   0, STITCH,    8 },   // 8 mũi sang trái
  {   0, -25, STITCH,    8 },   // 8 mũi xuống dưới -> về chỗ cũ
  {   0,   0, FOOT_UP,   1 },   // nâng chân vịt
  {   0,   0, END,       1 }
};
const uint16_t PATTERN_LEN = sizeof(PATTERN) / sizeof(PATTERN[0]);

// ================== BIẾN DÙNG CHUNG VỚI NGẮT ==================
volatile int16_t  encPos   = 0;      // 0 .. ENC_CPR-1
volatile uint8_t  encPrev  = 0;
volatile bool     idxPrev  = false;
volatile bool     homed    = false;  // đã thấy mốc kim chưa
volatile uint16_t revCount = 0;

volatile bool     running  = false;
volatile bool     wasInWin = false;
volatile uint16_t pIdx = 0;          // đang ở dòng nào của mẫu
volatile uint8_t  pRep = 0;          // đã lặp dòng đó mấy lần
volatile uint16_t stitchCount = 0;

volatile int32_t  stepMajor = 0, stepLeft = 0, absX = 0, absY = 0, errX = 0, errY = 0;

enum Event : uint8_t { EV_NONE, EV_FOOT_UP, EV_FOOT_DOWN, EV_END, EV_FAULT };
volatile uint8_t  event = EV_NONE;

// Bảng giải mã encoder 2 kênh: chỉ số = (trạng thái cũ << 2) | trạng thái mới
const int8_t QDEC[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };

// ================== NGẮT ENCODER (D9, D10) ==================
ISR(PCINT0_vect) {
  uint8_t s = (PINB >> 1) & 0x03;              // bit0 = D9 (A), bit1 = D10 (B)
  encPrev = ((encPrev << 2) | s) & 0x0F;
  int8_t d = QDEC[encPrev] * ENC_DIR;
  if (d) {
    int16_t p = encPos + d;
    if (p >= ENC_CPR) p -= ENC_CPR;
    else if (p < 0)   p += ENC_CPR;
    encPos = p;
  }
}

// ================== NGẮT CẢM BIẾN MỐC KIM (A0) ==================
ISR(PCINT1_vect) {
  bool active = ((PINC & _BV(0)) != 0) == INDEX_ACTIVE_HIGH;
  if (active && !idxPrev) {        // cạnh lên: miếng chắn vừa vào khe
    encPos = IDX_C;                // ép góc về đúng mốc, xóa sai số tích lũy
    homed = true;
    revCount++;
  }
  idxPrev = active;
}

// Lấy dòng tiếp theo trong mẫu. Gọi từ ngắt timer khi cửa sổ vừa mở.
static void loadNext() {
  while (true) {
    if (pIdx >= PATTERN_LEN) { running = false; event = EV_END; return; }
    const Buoc &b = PATTERN[pIdx];
    if (pRep >= b.lap) { pIdx++; pRep = 0; continue; }
    pRep++;

    if (b.cmd == STITCH) {
      int32_t sx = (int32_t)b.dx * STEPS_PER_MM / 10;
      int32_t sy = (int32_t)b.dy * STEPS_PER_MM / 10;
      if (sx >= 0) PORTD &= ~_BV(5); else PORTD |= _BV(5);   // X DIR
      if (sy >= 0) PORTD &= ~_BV(6); else PORTD |= _BV(6);   // Y DIR
      int32_t ax = sx < 0 ? -sx : sx;
      int32_t ay = sy < 0 ? -sy : sy;
      absX = ax; absY = ay;
      stepMajor = ax > ay ? ax : ay;
      stepLeft = stepMajor;
      errX = 0; errY = 0;
      stitchCount++;
      return;
    }
    // Lệnh không phải mũi may: báo cho loop() xử lý, vòng này không dịch vải
    if (b.cmd == FOOT_UP)        event = EV_FOOT_UP;
    else if (b.cmd == FOOT_DOWN) event = EV_FOOT_DOWN;
    else { event = EV_END; running = false; }
    return;
  }
}

// ================== NGẮT TIMER 8 kHz: PHÁT XUNG STEP ==================
ISR(TIMER1_COMPA_vect) {
  if (!running) { wasInWin = false; return; }

  int16_t p = encPos;
  bool inWin = (WIN_S > WIN_E) ? (p >= WIN_S || p <= WIN_E)
                               : (p >= WIN_S && p <= WIN_E);

  // Cửa sổ vừa mở và mũi trước đã xong -> nạp mũi mới
  if (inWin && !wasInWin && stepLeft == 0) loadNext();

  // Trong cửa sổ: chia đều bước cho X và Y (thuật toán Bresenham/DDA)
  if (inWin && stepLeft > 0) {
    uint8_t m = 0;
    errX += absX; if (errX >= stepMajor) { errX -= stepMajor; m |= _BV(2); }
    errY += absY; if (errY >= stepMajor) { errY -= stepMajor; m |= _BV(3); }
    stepLeft--;
    if (m) {
      PORTD |= m;                          // STEP lên
      __builtin_avr_delay_cycles(40);      // giữ ~2,5 µs (A4988 cần tối thiểu 1 µs)
      PORTD &= ~m;                         // STEP xuống
    }
  }

  // Cửa sổ đóng mà chưa dịch xong -> kim sắp cắm xuống khi khung còn chạy
  if (!inWin && wasInWin && stepLeft > 0) {
    stepLeft = 0;
    running = false;
    event = EV_FAULT;
  }
  wasInWin = inWin;
}

// ================== SOLENOID CHÂN VỊT: ĐÁ RỒI GIỮ ==================
bool footIsUp = false, kicking = false;
uint32_t kickStart = 0, footOnSince = 0;

void footUp() {
  analogWrite(PIN_SOL, SOL_KICK);
  kicking = true; footIsUp = true;
  kickStart = footOnSince = millis();
  Serial.println(F(">> Nâng chân vịt"));
}
void footDown() {
  analogWrite(PIN_SOL, 0);
  kicking = false; footIsUp = false;
  Serial.println(F(">> Hạ chân vịt"));
}
void footService() {
  if (kicking && millis() - kickStart >= SOL_KICK_MS) {
    analogWrite(PIN_SOL, SOL_HOLD);
    kicking = false;
  }
  if (footIsUp && millis() - footOnSince >= SOL_MAX_ON_MS) {
    Serial.println(F("!! Giữ chân vịt quá lâu, tự nhả cho đỡ nóng"));
    footDown();
  }
}

// ================== CÁC LỆNH ==================
bool driversOn = true;

void printHelp() {
  Serial.println(F("Lệnh: s=chạy  p=dừng  r=về đầu mẫu  f=nâng/hạ chân vịt  e=bật/tắt motor  h=trợ giúp"));
}

void resetPattern() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    running = false; pIdx = 0; pRep = 0; stepLeft = 0; stitchCount = 0;
  }
  Serial.println(F(">> Đã về đầu mẫu"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': {
        bool h; ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { h = homed; }
        if (!h) { Serial.println(F("!! Chưa thấy mốc kim. Quay vô lăng 1 vòng cho miếng chắn đi qua cảm biến chữ U.")); break; }
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          wasInWin = true;      // chờ cửa sổ của vòng sau mới bắt đầu, tránh nạp mũi giữa chừng
          running = true;
        }
        Serial.println(F(">> Bắt đầu. Quay trục kim, mỗi vòng bàn X-Y dịch 1 mũi."));
        break;
      }
      case 'p':
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { running = false; stepLeft = 0; }
        Serial.println(F(">> Dừng"));
        break;
      case 'r': resetPattern(); break;
      case 'f': if (footIsUp) footDown(); else footUp(); break;
      case 'e':
        driversOn = !driversOn;
        digitalWrite(PIN_EN, driversOn ? LOW : HIGH);
        Serial.println(driversOn ? F(">> Motor: bật (có lực giữ)") : F(">> Motor: tắt (xoay tay được)"));
        break;
      case 'h': printHelp(); break;
      default: break;   // bỏ qua ký tự xuống dòng
    }
  }
}

void handleEvents() {
  uint8_t ev;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ev = event; event = EV_NONE; }
  switch (ev) {
    case EV_FOOT_UP:   footUp(); break;
    case EV_FOOT_DOWN: footDown(); break;
    case EV_END:       Serial.println(F("== Xong mẫu. Gõ r rồi s để may lại.")); break;
    case EV_FAULT:
      Serial.println(F("!! LỖI: cửa sổ đã đóng mà bàn X-Y chưa tới nơi."));
      Serial.println(F("   Máy thật lúc này sẽ gãy kim. Quay chậm lại, hoặc giảm độ dài mũi, hoặc mở rộng cửa sổ."));
      break;
    default: break;
  }
}

void printStatus() {
  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();

  int16_t p; bool h, run; uint16_t st, rv;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = encPos; h = homed; run = running; st = stitchCount; rv = revCount; }
  int16_t deg = (int32_t)p * 360 / ENC_CPR;
  bool inWin = (WIN_S > WIN_E) ? (p >= WIN_S || p <= WIN_E) : (p >= WIN_S && p <= WIN_E);

  Serial.print(F("Góc kim: "));  Serial.print(deg);
  Serial.print(F("°  "));        Serial.print(inWin ? F("[CỬA SỔ MỞ]") : F("[kim dưới vải]"));
  Serial.print(F("  mốc: "));    Serial.print(h ? F("có") : F("chưa"));
  Serial.print(F("  vòng: "));   Serial.print(rv);
  Serial.print(F("  mũi: "));    Serial.print(st);
  Serial.println(run ? F("  ĐANG CHẠY") : F(""));
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_X_STEP, OUTPUT); pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);  pinMode(PIN_Y_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);     digitalWrite(PIN_EN, LOW);   // bật driver

  pinMode(PIN_SOL, OUTPUT);    analogWrite(PIN_SOL, 0);

  pinMode(PIN_ENC_A, INPUT_PULLUP);   // encoder NPN cần điện trở kéo lên
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_INDEX, INPUT);          // FC-03 có sẵn mạch kéo lên trên module

  encPrev = (PINB >> 1) & 0x03;
  idxPrev = ((PINC & _BV(0)) != 0) == INDEX_ACTIVE_HIGH;

  // Ngắt đổi trạng thái chân: D9, D10 (PCINT1, PCINT2) và A0 (PCINT8)
  PCICR  |= _BV(PCIE0) | _BV(PCIE1);
  PCMSK0 |= _BV(PCINT1) | _BV(PCINT2);
  PCMSK1 |= _BV(PCINT8);

  // Timer1 chế độ CTC: 16 MHz / 8 = 2 MHz, chia tiếp cho TICK_HZ
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  OCR1A  = (2000000UL / TICK_HZ) - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  Serial.println(F("=== MÁY MAY LẬP TRÌNH - BƯỚC 1 ==="));
  Serial.println(F("Quay vô lăng (hoặc trục encoder) 1 vòng để tìm mốc kim."));
  printHelp();
}

void loop() {
  handleSerial();
  handleEvents();
  footService();
  printStatus();
}
