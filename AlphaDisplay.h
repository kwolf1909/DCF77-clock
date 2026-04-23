#define HT16K33_BLINK_CMD       0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF       0
#define HT16K33_BLINK_2HZ       1
#define HT16K33_BLINK_1HZ       2
#define HT16K33_BLINK_HALFHZ    3

#define LINE_RIGHTUP    0x02
#define LINE_RIGHTDOWN  0x04
#define LINE_TOP        0x01
#define LINE_BOTTOM     0x08
#define LINE_LEFTUP     0x20
#define LINE_LEFTDOWN   0x10
#define LINE_MIDDLE     0x40
#define COLON           0x80

#ifdef SEG14
const uint16_t segs14[64] = {
  0x0, 0x6, 0x220, 0x12CE, 0x12ED, 0xC24, 0x235D, 0x400, 0x2400, 0x900, 0x3FC0, 0x12C0, 0x800, 0xC0, 0x0, 0xC00,
  0xC3F, 0x6, 0xDB, 0x8F, 0xE6, 0x2069, 0xFD, 0x7, 0xFF, 0xEF, 0x1200, 0xA00, 0x2400, 0xC8, 0x900, 0x1083,
  0x2BB, 0xF7, 0x128F, 0x39, 0x120F, 0xF9, 0x71, 0xBD, 0xF6, 0x1200, 0x1E, 0x2470, 0x38, 0x536, 0x2136, 0x3F,
  0xF3, 0x203F, 0x20F3, 0xED, 0x1201, 0x3E, 0xC30, 0x2836, 0x2D00, 0x1500, 0xC09, 0x39, 0x2100, 0xF, 0xC03, 0x8
};
#else
const uint8_t segs7[64] = {
  0x00, 0x86, 0x22, 0x7e, 0x6d, 0xd2, 0x46, 0x20, 0x29, 0x0b, 0x21, 0x70, 0x10, 0x40, 0x80, 0x52,
  0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x09, 0x0d, 0x61, 0x48, 0x43, 0xd3,
  0x5f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x3d, 0x76, 0x30, 0x1e, 0x75, 0x38, 0x15, 0x37, 0x3f,
  0x73, 0x6b, 0x33, 0x6d, 0x78, 0x3e, 0x3e, 0x2a, 0x76, 0x6e, 0x5b, 0x39, 0x64, 0x0f, 0x23, 0x08
};
#endif

// wrapper functions for Wire or TinyWire
void wireBegin(uint8_t addr) {
#ifdef TINYWIRE
  TinyI2C.start(addr, 0);
#else
  Wire.beginTransmission(addr);
#endif
}

void wireWrite(uint8_t data) {
#ifdef TINYWIRE
  TinyI2C.write(data);
#else
  Wire.write(data);
#endif
}

void wireEnd(void) {
#ifdef TINYWIRE
  TinyI2C.stop();
#else
  Wire.endTransmission();
#endif
}

class AlphaDisplay {
  public:
    void init(uint8_t, uint8_t, uint8_t);
    void print(const char *);
    void write(uint8_t);
    void clear();
  private:
    void send(uint8_t);
    uint8_t cur = 0;
    uint8_t address;
    uint8_t numDigits;
    char buf[16];
};

// Initialise the display
void AlphaDisplay::init(uint8_t addr, uint8_t digits, uint8_t brightness) {
  address = addr;
  numDigits = digits;

  wireBegin(address);
  wireWrite(0x21);                // Normal operation mode
  wireEnd();
  wireBegin(address);
  wireWrite(0xE0 + brightness);   // Set brightness
  wireEnd();
  clear();
  wireBegin(address);
  wireWrite(0x81);                // Display on
  wireEnd();
}

// Send character to display as two bytes; top bit set = decimal point
void AlphaDisplay::send(uint8_t x) {
  uint16_t segments;

#ifdef SEG14
  uint16_t dp = 0;
  if (x & 0x80) {
    dp = 0x4000;
    x &= 0x7F;
  }
  if (x >= 0x60) x = x - 0x20;
  segments = segs14[x - 32] | dp;
  wireWrite(segments);
  wireWrite(segments >> 8);
#else
  segments = segs7[(x & 0x7F )- 32];
  segments |= (x & 0x80);
  wireWrite(segments);
  wireWrite(0);
#endif
}

// Clear display
void AlphaDisplay::clear() {
  wireBegin(address);
  for (int i = 0; i < (2 * numDigits + 1); i++) wireWrite(0);
  wireEnd();
  cur = 0;
}

// writes a string to display
void AlphaDisplay::print(const char *s) {
  char c;
  uint8_t i = 0;
  uint8_t pos = 0;

  wireBegin(address);
  wireWrite(0);

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
  wireEnd();
}

// Write to the current cursor position 0 to 7 and handle scrolling
void AlphaDisplay::write(uint8_t c) {
  if (c == 13) cur = 0;
  if (c == '.') {
    c = buf[cur - 1] | 0x80;
    wireBegin(address);
    wireWrite((cur - 1) * 2);
    send(c);
    wireEnd();
    buf[cur - 1] = c;
  } else if (c >= 32) {          // Printing character
    if (cur == numDigits) {      // Scroll display left
      wireBegin(address);
      wireWrite(0);
      for (int i = 0; i < 7; i++) {
        uint8_t d = buf[i + 1];
        send(d);
        buf[i] = d;
      }
      wireEnd();
      cur--;
    }
    wireBegin(address);
    wireWrite(cur * 2);
    send(c);
    wireEnd();
    buf[cur] = c;
    cur++;
    if (cur == numDigits) delay(250);
  } else if (c == 12) {
    clear();
  }
  return;
}
