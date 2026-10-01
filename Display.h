// Display object

#define HT16K33_CMD       0x80
#define HT16K33_ON        1
#define HT16K33_OFF       0
#define HT16K33_1HZ       2
#define HT16K33_2HZ       4
#define HT16K33_HALFHZ    6

#define SEG_RIGHTUP       0x02
#define SEG_RIGHTDOWN     0x04
#define SEG_TOP           0x01
#define SEG_BOTTOM        0x08
#define SEG_LEFTUP        0x20
#define SEG_LEFTDOWN      0x10
#define SEG_MIDDLE        0x40

#define DOT               0x80
#define COLON             0x02  // is controlled by display position 2 on Adafruit 7seg display

class Display {
  public:
    void init(uint8_t, uint8_t, uint8_t);
    void print(const char *);
    void write(uint8_t);
    void clear();

  private:
    const uint8_t segs7[64] = {
      0x00, 0x86, 0x22, 0x7e, 0x6d, 0xd2, 0x46, 0x20, 0x29, 0x0b, 0x21, 0x70, 0x10, 0x40, 0x80, 0x52,
      0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x09, 0x0d, 0x61, 0x48, 0x43, 0xd3,
      0x5f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x3d, 0x76, 0x30, 0x1e, 0x75, 0x38, 0x15, 0x37, 0x3f,
      0x73, 0x6b, 0x33, 0x6d, 0x78, 0x3e, 0x3e, 0x2a, 0x76, 0x6e, 0x5b, 0x39, 0x64, 0x0f, 0x23, 0x08
    };

    void send(uint8_t);
    uint8_t cur = 0;
    uint8_t address;
    uint8_t numDigits;
    char buf[16];
};

// Initialise the display
void Display::init(uint8_t addr, uint8_t digits, uint8_t brightness) {
  address = addr;
  numDigits = digits;

  Wire.beginTransmission(address);
  Wire.write(0x21);                // Normal operation mode
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0xE0 + brightness);   // Set brightness
  Wire.endTransmission();
  clear();
  Wire.beginTransmission(address);
  Wire.write(0x81);                // Display on
  Wire.endTransmission();
}

// Send character to display as two bytes; top bit set = decimal point
void Display::send(uint8_t x) {
  uint16_t segments;

  segments = segs7[(x & 0x7F )- 32];
  segments |= (x & 0x80);
  Wire.write(segments);
  Wire.write(0);
}

// Clear display
void Display::clear() {
  Wire.beginTransmission(address);
  for (int i = 0; i < (2 * numDigits + 1); i++) Wire.write(0);
  Wire.endTransmission();
  cur = 0;
}

// writes a string to display
void Display::print(const char *s) {
  char c;
  uint8_t i = 0;
  uint8_t pos = 0;

  Wire.beginTransmission(address);
  Wire.write(0);

  while (s[i] && pos < numDigits) {
    c = s[i];
    if (s[i + 1] == '.') {
      c |= 0x80;
      i++;
    }
    send(c);
    i++;
    pos++;
  }
  Wire.endTransmission();
}

// Write to the current cursor position and handle scrolling
void Display::write(uint8_t c) {
  if (c == 13) cur = 0;
  if (c == '.') {
    c = buf[cur - 1] | 0x80;
    Wire.beginTransmission(address);
    Wire.write((cur - 1) * 2);
    send(c);
    Wire.endTransmission();
    buf[cur - 1] = c;
  } else if (c >= 32) {          // Printing character
    if (cur == numDigits) {      // Scroll display left
      Wire.beginTransmission(address);
      Wire.write(0);
      for (int i = 0; i < 7; i++) {
        uint8_t d = buf[i + 1];
        send(d);
        buf[i] = d;
      }
      Wire.endTransmission();
      cur--;
    }
    Wire.beginTransmission(address);
    Wire.write(cur * 2);
    send(c);
    Wire.endTransmission();
    buf[cur] = c;
    cur++;
    if (cur == numDigits) delay(250);
  } else if (c == 12) {
    clear();
  }
  return;
}
