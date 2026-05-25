/*
 * 1 位共阳数码管 + 按键：按一下数字 +1，0~9 循环
 *
 * 安全：每段必须串 220Ω；勿把 5V 直接接到段脚；勿段脚同时接 5V 和 Arduino。
 * 共阳：脚1→5V；段脚→电阻→Arduino 引脚(LOW 点亮)。坏段只能更换数码管。
 *
 * 段：a→D2脚8  b→D3脚7  c→D4脚6  d→D5脚5  e→D6脚4  f→D7脚2  g→D12脚3
 * 按键：对角两脚 → D9 与 GND
 */

#define WIRING_LINEAR 0
#define DEBUG_BUTTON 0
#define TEST_SEG_G 0          // 1=只亮中间 g（D12→脚3，全亮不测 PWM）
#define FULL_BRIGHTNESS 0     // 1=关闭调光，表笔测电阻约 1~2V 压降

#if WIRING_LINEAR
const int SEG_PIN[] = {8, 7, 6, 5, 4, 2, 3};
#else
// 顺序 a b c d e f g
const int SEG_PIN[] = {2, 3, 4, 5, 6, 7, 12};
#endif

const int SEG_COUNT = 7;
const int BUTTON_PIN = 9;

const byte BRIGHTNESS = 50;
const unsigned long DEBOUNCE_MS = 30;

const byte DIGIT_SEG[10][7] = {
  {1, 1, 1, 1, 1, 1, 0},
  {0, 1, 1, 0, 0, 0, 0},
  {1, 1, 0, 1, 1, 0, 1},
  {1, 1, 1, 1, 0, 0, 1},
  {0, 1, 1, 0, 0, 1, 1},
  {1, 0, 1, 1, 0, 1, 1},
  {1, 0, 1, 1, 1, 1, 1},
  {1, 1, 1, 0, 0, 0, 0},
  {1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 0, 1, 1},
};

int currentDigit = 0;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < SEG_COUNT; i++) {
    pinMode(SEG_PIN[i], OUTPUT);
    digitalWrite(SEG_PIN[i], HIGH);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.print(F("上电 D9="));
  Serial.println(digitalRead(BUTTON_PIN) == LOW ? F("LOW(异常,检查接线)") : F("HIGH(正常松开)"));
  Serial.println(F("按下键时应变 LOW；若一直 HIGH 说明按键未接到 GND"));
}

void loop() {
#if TEST_SEG_G
  // 仅测试中间段 g：应只有横杠亮（D12 → 模块脚 3，串 220Ω）
  segmentsAllOff();
  digitalWrite(SEG_PIN[6], LOW);
  return;
#endif

  for (int seg = 0; seg < SEG_COUNT; seg++) {
    writeSegment(seg, DIGIT_SEG[currentDigit][seg]);
  }

  if (buttonPressedOnce()) {
    currentDigit = (currentDigit + 1) % 10;
    Serial.print(F("→ 数字 "));
    Serial.println(currentDigit);
  }

#if DEBUG_BUTTON
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 400) {
    lastDbg = millis();
    Serial.print(digitalRead(BUTTON_PIN) == LOW ? F("[按下] ") : F("[松开] "));
    Serial.println(currentDigit);
  }
#endif
}

// 按下瞬间触发一次（松开后可再按）
bool buttonPressedOnce() {
  static int lastStable = HIGH;
  static unsigned long debounceAt = 0;
  static int pending = HIGH;

  int reading = digitalRead(BUTTON_PIN);

  if (reading != pending) {
    debounceAt = millis();
    pending = reading;
  }

  if ((millis() - debounceAt) < DEBOUNCE_MS) {
    return false;
  }

  if (pending != lastStable) {
    lastStable = pending;
    if (lastStable == LOW) {
      return true;
    }
  }
  return false;
}

void segmentsAllOff() {
  for (int i = 0; i < SEG_COUNT; i++) {
    digitalWrite(SEG_PIN[i], HIGH);
  }
}

void writeSegment(int seg, bool on) {
  if (!on) {
    digitalWrite(SEG_PIN[seg], HIGH);
    return;
  }
#if FULL_BRIGHTNESS || TEST_SEG_G
  digitalWrite(SEG_PIN[seg], LOW);
#else
  digitalWrite(SEG_PIN[seg], ((micros() >> 2) & 0xFF) < BRIGHTNESS ? LOW : HIGH);
#endif
}
