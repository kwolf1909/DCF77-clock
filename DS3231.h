// DS3231 with DateTime and TimeSpan objects - extract of RTClib

#define SECONDS_PER_DAY 86400L // 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800 // Unixtime for 2000-01-01 00:00:00, useful for initialization

class TimeSpan;

class DateTime {
  public:
    DateTime(uint32_t t = SECONDS_FROM_1970_TO_2000);
    DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
             uint8_t min = 0, uint8_t sec = 0);
    DateTime(const DateTime &copy);
    DateTime(const char *date, const char *time);
    DateTime(const __FlashStringHelper *date, const __FlashStringHelper *time);
    DateTime(const char *iso8601date);
    bool isValid() const;
    char *toString(char *buffer) const;

    uint16_t year() const {
      return 2000U + yOff;
    }
    uint8_t month() const {
      return m;
    }
    uint8_t day() const {
      return d;
    }
    uint8_t hour() const {
      return hh;
    }
    uint8_t twelveHour() const;
    uint8_t isPM() const {
      return hh >= 12;
    }
    uint8_t minute() const {
      return mm;
    }
    uint8_t second() const {
      return ss;
    }
    uint8_t dayOfTheWeek() const;
    uint32_t secondstime() const;
    uint32_t unixtime(void) const;

    enum timestampOpt {
      TIMESTAMP_FULL, //!< `YYYY-MM-DDThh:mm:ss`
      TIMESTAMP_TIME, //!< `hh:mm:ss`
      TIMESTAMP_DATE  //!< `YYYY-MM-DD`
    };
    String timestamp(timestampOpt opt = TIMESTAMP_FULL) const;

    DateTime operator+(const TimeSpan &span) const;
    DateTime operator-(const TimeSpan &span) const;
    TimeSpan operator-(const DateTime &right) const;
    bool operator<(const DateTime &right) const;

    bool operator>(const DateTime &right) const {
      return right < *this;
    }

    bool operator<=(const DateTime &right) const {
      return !(*this > right);
    }

    bool operator>=(const DateTime &right) const {
      return !(*this < right);
    }
    bool operator==(const DateTime &right) const;

    bool operator!=(const DateTime &right) const {
      return !(*this == right);
    }

  protected:
    uint8_t yOff; // Year offset from 2000
    uint8_t m;    // Month 1-12
    uint8_t d;    // Day 1-31
    uint8_t hh;   // Hours 0-23
    uint8_t mm;   // Minutes 0-59
    uint8_t ss;   // Seconds 0-59
};

class TimeSpan {
  public:
    TimeSpan(int32_t seconds = 0);
    TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
    TimeSpan(const TimeSpan &copy);

    int16_t days() const {
      return _seconds / 86400L;
    }
    int8_t hours() const {
      return _seconds / 3600 % 24;
    }
    int8_t minutes() const {
      return _seconds / 60 % 60;
    }
    int8_t seconds() const {
      return _seconds % 60;
    }
    int32_t totalseconds() const {
      return _seconds;
    }
    TimeSpan operator+(const TimeSpan &right) const;
    TimeSpan operator-(const TimeSpan &right) const;

  protected:
    int32_t _seconds; // Actual TimeSpan value is stored as seconds
};

const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30};

static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
  if (y >= 2000U)
    y -= 2000U;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

static uint32_t time2ulong(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
  return ((days * 24UL + h) * 60 + m) * 60 + s;
}

DateTime::DateTime(uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000; // bring to 2000 timestamp from 1970

  ss = t % 60;
  t /= 60;
  mm = t % 60;
  t /= 60;
  hh = t % 24;
  uint16_t days = t / 24;
  uint8_t leap;
  for (yOff = 0;; ++yOff) {
    leap = yOff % 4 == 0;
    if (days < 365U + leap)
      break;
    days -= 365 + leap;
  }
  for (m = 1; m < 12; ++m) {
    uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
    if (leap && m == 2)
      ++daysPerMonth;
    if (days < daysPerMonth)
      break;
    days -= daysPerMonth;
  }
  d = days + 1;
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour,
                   uint8_t min, uint8_t sec) {
  if (year >= 2000U)
    year -= 2000U;
  yOff = year;
  m = month;
  d = day;
  hh = hour;
  mm = min;
  ss = sec;
}

DateTime::DateTime(const DateTime &copy)
  : yOff(copy.yOff), m(copy.m), d(copy.d), hh(copy.hh), mm(copy.mm),
    ss(copy.ss) {}

static uint8_t conv2d(const char *p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

DateTime::DateTime(const char *date, const char *time) {
  yOff = conv2d(date + 9);
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  switch (date[0]) {
    case 'J':
      m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7);
      break;
    case 'F':
      m = 2;
      break;
    case 'A':
      m = date[2] == 'r' ? 4 : 8;
      break;
    case 'M':
      m = date[2] == 'r' ? 3 : 5;
      break;
    case 'S':
      m = 9;
      break;
    case 'O':
      m = 10;
      break;
    case 'N':
      m = 11;
      break;
    case 'D':
      m = 12;
      break;
  }
  d = conv2d(date + 4);
  hh = conv2d(time);
  mm = conv2d(time + 3);
  ss = conv2d(time + 6);
}

DateTime::DateTime(const __FlashStringHelper *date,
                   const __FlashStringHelper *time) {
  char buff[11];
  memcpy_P(buff, date, 11);
  yOff = conv2d(buff + 9);
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  switch (buff[0]) {
    case 'J':
      m = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7);
      break;
    case 'F':
      m = 2;
      break;
    case 'A':
      m = buff[2] == 'r' ? 4 : 8;
      break;
    case 'M':
      m = buff[2] == 'r' ? 3 : 5;
      break;
    case 'S':
      m = 9;
      break;
    case 'O':
      m = 10;
      break;
    case 'N':
      m = 11;
      break;
    case 'D':
      m = 12;
      break;
  }
  d = conv2d(buff + 4);
  memcpy_P(buff, time, 8);
  hh = conv2d(buff);
  mm = conv2d(buff + 3);
  ss = conv2d(buff + 6);
}

DateTime::DateTime(const char *iso8601dateTime) {
  char ref[] = "2000-01-01T00:00:00";
  memcpy(ref, iso8601dateTime, min(strlen(ref), strlen(iso8601dateTime)));
  yOff = conv2d(ref + 2);
  m = conv2d(ref + 5);
  d = conv2d(ref + 8);
  hh = conv2d(ref + 11);
  mm = conv2d(ref + 14);
  ss = conv2d(ref + 17);
}

bool DateTime::isValid() const {
  if (yOff >= 100)
    return false;
  DateTime other(unixtime());
  return yOff == other.yOff && m == other.m && d == other.d && hh == other.hh &&
         mm == other.mm && ss == other.ss;
}

char *DateTime::toString(char *buffer) const {
  uint8_t apTag =
    (strstr(buffer, "ap") != nullptr) || (strstr(buffer, "AP") != nullptr);
  uint8_t hourReformatted = 0, isPM = false;
  if (apTag) {     // 12 Hour Mode
    if (hh == 0) { // midnight
      isPM = false;
      hourReformatted = 12;
    } else if (hh == 12) { // noon
      isPM = true;
      hourReformatted = 12;
    } else if (hh < 12) { // morning
      isPM = false;
      hourReformatted = hh;
    } else { // 1 o'clock or after
      isPM = true;
      hourReformatted = hh - 12;
    }
  }

  for (size_t i = 0; i < strlen(buffer) - 1; i++) {
    if (buffer[i] == 'h' && buffer[i + 1] == 'h') {
      if (!apTag) { // 24 Hour Mode
        buffer[i] = '0' + hh / 10;
        buffer[i + 1] = '0' + hh % 10;
      } else { // 12 Hour Mode
        buffer[i] = '0' + hourReformatted / 10;
        buffer[i + 1] = '0' + hourReformatted % 10;
      }
    }
    if (buffer[i] == 'm' && buffer[i + 1] == 'm') {
      buffer[i] = '0' + mm / 10;
      buffer[i + 1] = '0' + mm % 10;
    }
    if (buffer[i] == 's' && buffer[i + 1] == 's') {
      buffer[i] = '0' + ss / 10;
      buffer[i + 1] = '0' + ss % 10;
    }
    if (buffer[i] == 'D' && buffer[i + 1] == 'D' && buffer[i + 2] == 'D') {
      static PROGMEM const char day_names[] = "SunMonTueWedThuFriSat";
      const char *p = &day_names[3 * dayOfTheWeek()];
      buffer[i] = pgm_read_byte(p);
      buffer[i + 1] = pgm_read_byte(p + 1);
      buffer[i + 2] = pgm_read_byte(p + 2);
    } else if (buffer[i] == 'D' && buffer[i + 1] == 'D') {
      buffer[i] = '0' + d / 10;
      buffer[i + 1] = '0' + d % 10;
    }
    if (buffer[i] == 'M' && buffer[i + 1] == 'M' && buffer[i + 2] == 'M') {
      static PROGMEM const char month_names[] =
        "JanFebMarAprMayJunJulAugSepOctNovDec";
      const char *p = &month_names[3 * (m - 1)];
      buffer[i] = pgm_read_byte(p);
      buffer[i + 1] = pgm_read_byte(p + 1);
      buffer[i + 2] = pgm_read_byte(p + 2);
    } else if (buffer[i] == 'M' && buffer[i + 1] == 'M') {
      buffer[i] = '0' + m / 10;
      buffer[i + 1] = '0' + m % 10;
    }
    if (buffer[i] == 'Y' && buffer[i + 1] == 'Y' && buffer[i + 2] == 'Y' &&
        buffer[i + 3] == 'Y') {
      buffer[i] = '2';
      buffer[i + 1] = '0';
      buffer[i + 2] = '0' + (yOff / 10) % 10;
      buffer[i + 3] = '0' + yOff % 10;
    } else if (buffer[i] == 'Y' && buffer[i + 1] == 'Y') {
      buffer[i] = '0' + (yOff / 10) % 10;
      buffer[i + 1] = '0' + yOff % 10;
    }
    if (buffer[i] == 'A' && buffer[i + 1] == 'P') {
      if (isPM) {
        buffer[i] = 'P';
        buffer[i + 1] = 'M';
      } else {
        buffer[i] = 'A';
        buffer[i + 1] = 'M';
      }
    } else if (buffer[i] == 'a' && buffer[i + 1] == 'p') {
      if (isPM) {
        buffer[i] = 'p';
        buffer[i + 1] = 'm';
      } else {
        buffer[i] = 'a';
        buffer[i + 1] = 'm';
      }
    }
  }
  return buffer;
}

uint8_t DateTime::twelveHour() const {
  if (hh == 0 || hh == 12) { // midnight or noon
    return 12;
  } else if (hh > 12) { // 1 o'clock or later
    return hh - 12;
  } else { // morning
    return hh;
  }
}

uint8_t DateTime::dayOfTheWeek() const {
  uint16_t day = date2days(yOff, m, d);
  return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2ulong(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000; // seconds from 1970 to 2000

  return t;
}

uint32_t DateTime::secondstime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2ulong(days, hh, mm, ss);
  return t;
}

DateTime DateTime::operator+(const TimeSpan &span) const {
  return DateTime(unixtime() + span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan &span) const {
  return DateTime(unixtime() - span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime &right) const {
  return TimeSpan(unixtime() - right.unixtime());
}

bool DateTime::operator<(const DateTime &right) const {
  return (yOff + 2000U < right.year() ||
          (yOff + 2000U == right.year() &&
           (m < right.month() ||
            (m == right.month() &&
             (d < right.day() ||
              (d == right.day() &&
               (hh < right.hour() ||
                (hh == right.hour() &&
                 (mm < right.minute() ||
                  (mm == right.minute() && ss < right.second()))))))))));
}

bool DateTime::operator==(const DateTime &right) const {
  return (right.year() == yOff + 2000U && right.month() == m &&
          right.day() == d && right.hour() == hh && right.minute() == mm && right.second() == ss);
}

String DateTime::timestamp(timestampOpt opt) const {
  char buffer[25]; // large enough for any DateTime, including invalid ones

  // Generate timestamp according to opt
  switch (opt) {
    case TIMESTAMP_TIME:
      // Only time
      sprintf(buffer, "%02d:%02d:%02d", hh, mm, ss);
      break;
    case TIMESTAMP_DATE:
      // Only date
      sprintf(buffer, "%u-%02d-%02d", 2000U + yOff, m, d);
      break;
    default:
      // Full
      sprintf(buffer, "%u-%02d-%02dT%02d:%02d:%02d", 2000U + yOff, m, d, hh, mm, ss);
  }
  return String(buffer);
}

TimeSpan::TimeSpan(int32_t seconds) : _seconds(seconds) {}

TimeSpan::TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds)
  : _seconds((int32_t)days * 86400L + (int32_t)hours * 3600 + (int32_t)minutes * 60 + seconds) {}

TimeSpan::TimeSpan(const TimeSpan &copy) : _seconds(copy._seconds) {}

TimeSpan TimeSpan::operator+(const TimeSpan &right) const {
  return TimeSpan(_seconds + right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan &right) const {
  return TimeSpan(_seconds - right._seconds);
}

class RTC_Millis {
  public:
    void begin(const DateTime &dt) {
      adjust(dt);
    }
    void adjust(const DateTime &dt);
    DateTime now();

  protected:
    uint32_t lastUnix;
    uint32_t lastMillis;
};

void RTC_Millis::adjust(const DateTime &dt) {
  lastMillis = millis();
  lastUnix = dt.unixtime();
}

DateTime RTC_Millis::now() {
  uint32_t elapsedSeconds = (millis() - lastMillis) / 1000;
  lastMillis += elapsedSeconds * 1000;
  lastUnix += elapsedSeconds;
  return lastUnix;
}

class RTC_Micros {
  public:
    void begin(const DateTime &dt) {
      adjust(dt);
    }
    void adjust(const DateTime &dt);
    void adjustDrift(int ppm);
    DateTime now();

  protected:
    uint32_t microsPerSecond = 1000000;
    uint32_t lastUnix;
    uint32_t lastMicros;
};

void RTC_Micros::adjust(const DateTime &dt) {
  lastMicros = micros();
  lastUnix = dt.unixtime();
}

void RTC_Micros::adjustDrift(int ppm) {
  microsPerSecond = 1000000 - ppm;
}

DateTime RTC_Micros::now() {
  uint32_t elapsedSeconds = (micros() - lastMicros) / microsPerSecond;
  lastMicros += elapsedSeconds * microsPerSecond;
  lastUnix += elapsedSeconds;
  return lastUnix;
}

#define DS3231_ADDRESS 0x68   // I2C address for DS3231
#define DS3231_TIME 0x00      // Time register
#define DS3231_ALARM1 0x07    // Alarm 1 register
#define DS3231_ALARM2 0x0B    // Alarm 2 register
#define DS3231_CONTROL 0x0E   // Control register
#define DS3231_STATUSREG 0x0F // Status register
#define DS3231_TEMPERATUREREG 0x11  // Temperature register (high byte - low byte is at 0x12), 10-bit
                                    // temperature value

class RTC_I2C {
  protected:
    static uint8_t bcd2bin(uint8_t val) {
      return val - 6 * (val >> 4);
    }
    static uint8_t bin2bcd(uint8_t val) {
      return val + 6 * (val / 10);
    }
    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t val);
    bool write_then_read(const uint8_t *, size_t, uint8_t *, size_t);
};

void RTC_I2C::write_register(uint8_t reg, uint8_t val) {
  uint8_t buffer[2] = {reg, val};

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(buffer, 2);
  Wire.endTransmission();
}

uint8_t RTC_I2C::read_register(uint8_t reg) {
  uint8_t buffer[1];

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.requestFrom(DS3231_ADDRESS, 1);
  buffer[0] = Wire.read();
  Wire.endTransmission();

  return buffer[0];
}

bool RTC_I2C::write_then_read(const uint8_t *write_buffer,
                              size_t write_len, uint8_t *read_buffer,
                              size_t read_len) {

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(write_buffer, write_len);
  Wire.endTransmission();

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.requestFrom(DS3231_ADDRESS, (int)read_len);
  for (uint8_t i = 0; i < read_len; i++) read_buffer[i] = Wire.read();
  Wire.endTransmission();

  return true;
}

/** DS3231 SQW pin mode settings */
enum Ds3231SqwPinMode {
  DS3231_OFF = 0x1C,
  DS3231_SquareWave1Hz = 0x00,
  DS3231_SquareWave1kHz = 0x08,
  DS3231_SquareWave4kHz = 0x10,
  DS3231_SquareWave8kHz = 0x18
};

/** DS3231 Alarm modes for alarm 1 */
enum Ds3231Alarm1Mode {
  DS3231_A1_PerSecond = 0x0F,
  DS3231_A1_Second = 0x0E,
  DS3231_A1_Minute = 0x0C,
  DS3231_A1_Hour = 0x08,
  DS3231_A1_Date = 0x00,
  DS3231_A1_Day = 0x10
};

/** DS3231 Alarm modes for alarm 2 */
enum Ds3231Alarm2Mode {
  DS3231_A2_PerMinute = 0x7,
  DS3231_A2_Minute = 0x6,
  DS3231_A2_Hour = 0x4,
  DS3231_A2_Date = 0x0,
  DS3231_A2_Day = 0x8
};

class RTC_DS3231 : RTC_I2C {
  public:
    bool begin(void);
    void adjust(const DateTime &dt);
    bool lostPower(void);
    DateTime now();
    Ds3231SqwPinMode readSqwPinMode();
    void writeSqwPinMode(Ds3231SqwPinMode mode);
    bool setAlarm1(const DateTime &dt, Ds3231Alarm1Mode alarm_mode);
    bool setAlarm2(const DateTime &dt, Ds3231Alarm2Mode alarm_mode);
    DateTime getAlarm1();
    DateTime getAlarm2();
    Ds3231Alarm1Mode getAlarm1Mode();
    Ds3231Alarm2Mode getAlarm2Mode();
    void disableAlarm(uint8_t alarm_num);
    void clearAlarm(uint8_t alarm_num);
    bool alarmFired(uint8_t alarm_num);
    void enable32K(void);
    void disable32K(void);
    bool isEnabled32K(void);
    int16_t getTemperature(); // in Celsius degree
    static uint8_t dowToDS3231(uint8_t d) {
      return d == 0 ? 7 : d;
    }
};

bool RTC_DS3231::begin() {
  // A basic scanner, see if it ACK's
  Wire.beginTransmission(DS3231_ADDRESS);
  if (Wire.endTransmission() == 0) return true;
  return false;
}

bool RTC_DS3231::lostPower(void) {
  return read_register(DS3231_STATUSREG) >> 7;
}

void RTC_DS3231::adjust(const DateTime &dt) {
  uint8_t buffer[8] = {DS3231_TIME,
                       bin2bcd(dt.second()),
                       bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()),
                       bin2bcd(dowToDS3231(dt.dayOfTheWeek())),
                       bin2bcd(dt.day()),
                       bin2bcd(dt.month()),
                       bin2bcd(dt.year() - 2000U)
                      };

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(buffer, 8);
  Wire.endTransmission();

//uint8_t statreg = read_register(DS3231_STATUSREG);
//statreg &= ~0x80; // clear OSF bit
//write_register(DS3231_STATUSREG, statreg);
}

DateTime RTC_DS3231::now() {
  uint8_t buffer[7];
  buffer[0] = 0;

  write_then_read(buffer, 1, buffer, 7);

  return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
                  bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                  bcd2bin(buffer[0] & 0x7F));
}

Ds3231SqwPinMode RTC_DS3231::readSqwPinMode() {
  int mode;
  mode = read_register(DS3231_CONTROL) & 0x1C;
  if (mode & 0x04)
    mode = DS3231_OFF;
  return static_cast<Ds3231SqwPinMode>(mode);
}

void RTC_DS3231::writeSqwPinMode(Ds3231SqwPinMode mode) {
  uint8_t ctrl = read_register(DS3231_CONTROL);

  ctrl &= ~0x04; // turn off INTCON
  ctrl &= ~0x18; // set freq bits to 0

  write_register(DS3231_CONTROL, ctrl | mode);
}

int16_t RTC_DS3231::getTemperature() {
  uint8_t buffer[2] = {DS3231_TEMPERATUREREG, 0};
  write_then_read(buffer, 1, buffer, 2);
  return (int16_t)(buffer[0] * 100 + (buffer[1] >> 6) * 25);
}

bool RTC_DS3231::setAlarm1(const DateTime &dt, Ds3231Alarm1Mode alarm_mode) {
  uint8_t ctrl = read_register(DS3231_CONTROL);
  if (!(ctrl & 0x04)) {
    return false;
  }
  uint8_t A1M1 = (alarm_mode & 0x01) << 7; // Seconds bit 7.
  uint8_t A1M2 = (alarm_mode & 0x02) << 6; // Minutes bit 7.
  uint8_t A1M3 = (alarm_mode & 0x04) << 5; // Hour bit 7.
  uint8_t A1M4 = (alarm_mode & 0x08) << 4; // Day/Date bit 7.
  uint8_t DY_DT = (alarm_mode & 0x10)
                  << 2; // Day/Date bit 6. Date when 0, day of week when 1.
  uint8_t day = (DY_DT) ? dowToDS3231(dt.dayOfTheWeek()) : dt.day();

  uint8_t buffer[5] = {DS3231_ALARM1, uint8_t(bin2bcd(dt.second()) | A1M1),
                       uint8_t(bin2bcd(dt.minute()) | A1M2),
                       uint8_t(bin2bcd(dt.hour()) | A1M3),
                       uint8_t(bin2bcd(day) | A1M4 | DY_DT)
                      };

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(buffer, 5);
  Wire.endTransmission();
  write_register(DS3231_CONTROL, ctrl | 0x01); // AI1E
  return true;
}

bool RTC_DS3231::setAlarm2(const DateTime &dt, Ds3231Alarm2Mode alarm_mode) {
  uint8_t ctrl = read_register(DS3231_CONTROL);
  if (!(ctrl & 0x04)) {
    return false;
  }

  uint8_t A2M2 = (alarm_mode & 0x01) << 7; // Minutes bit 7.
  uint8_t A2M3 = (alarm_mode & 0x02) << 6; // Hour bit 7.
  uint8_t A2M4 = (alarm_mode & 0x04) << 5; // Day/Date bit 7.
  uint8_t DY_DT = (alarm_mode & 0x08)
                  << 3; // Day/Date bit 6. Date when 0, day of week when 1.
  uint8_t day = (DY_DT) ? dowToDS3231(dt.dayOfTheWeek()) : dt.day();

  uint8_t buffer[4] = {DS3231_ALARM2, uint8_t(bin2bcd(dt.minute()) | A2M2),
                       uint8_t(bin2bcd(dt.hour()) | A2M3),
                       uint8_t(bin2bcd(day) | A2M4 | DY_DT)
                      };

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(buffer, 4);
  Wire.endTransmission();
  write_register(DS3231_CONTROL, ctrl | 0x02); // AI2E
  return true;
}

DateTime RTC_DS3231::getAlarm1() {
  uint8_t buffer[5] = {DS3231_ALARM1, 0, 0, 0, 0};
  write_then_read(buffer, 1, buffer, 5);

  uint8_t seconds = bcd2bin(buffer[0] & 0x7F);
  uint8_t minutes = bcd2bin(buffer[1] & 0x7F);
  uint8_t hour = bcd2bin(buffer[2] & 0x3F);

  bool isDayOfWeek = (buffer[3] & 0x40) >> 6;
  uint8_t day;
  if (isDayOfWeek) {
    // Alarm set to match on day of the week
    day = bcd2bin(buffer[3] & 0x0F);
  } else {
    // Alarm set to match on day of the month
    day = bcd2bin(buffer[3] & 0x3F);
  }
  return DateTime(2000, 5, day, hour, minutes, seconds);
}

DateTime RTC_DS3231::getAlarm2() {
  uint8_t buffer[4] = {DS3231_ALARM2, 0, 0, 0};
  write_then_read(buffer, 1, buffer, 4);

  uint8_t minutes = bcd2bin(buffer[0] & 0x7F);
  uint8_t hour = bcd2bin(buffer[1] & 0x3F);

  bool isDayOfWeek = (buffer[2] & 0x40) >> 6;
  uint8_t day;
  if (isDayOfWeek) {
    // Alarm set to match on day of the week
    day = bcd2bin(buffer[2] & 0x0F);
  } else {
    // Alarm set to match on day of the month
    day = bcd2bin(buffer[2] & 0x3F);
  }
  return DateTime(2000, 5, day, hour, minutes, 0);
}

Ds3231Alarm1Mode RTC_DS3231::getAlarm1Mode() {
  uint8_t buffer[5] = {DS3231_ALARM1, 0, 0, 0, 0};
  write_then_read(buffer, 1, buffer, 5);

  uint8_t alarm_mode = (buffer[0] & 0x80) >> 7    // A1M1 - Seconds bit
                       | (buffer[1] & 0x80) >> 6  // A1M2 - Minutes bit
                       | (buffer[2] & 0x80) >> 5  // A1M3 - Hour bit
                       | (buffer[3] & 0x80) >> 4  // A1M4 - Day/Date bit
                       | (buffer[3] & 0x40) >> 2; // DY_DT

  // Determine which mode the fetched alarm bits map to
  switch (alarm_mode) {
    case DS3231_A1_PerSecond:
    case DS3231_A1_Second:
    case DS3231_A1_Minute:
    case DS3231_A1_Hour:
    case DS3231_A1_Date:
    case DS3231_A1_Day:
      return (Ds3231Alarm1Mode)alarm_mode;
    default:
      // Default if the alarm mode cannot be read
      return DS3231_A1_Date;
  }
}

Ds3231Alarm2Mode RTC_DS3231::getAlarm2Mode() {
  uint8_t buffer[4] = {DS3231_ALARM2, 0, 0, 0};
  write_then_read(buffer, 1, buffer, 4);

  uint8_t alarm_mode = (buffer[0] & 0x80) >> 7    // A2M2 - Minutes bit
                       | (buffer[1] & 0x80) >> 6  // A2M3 - Hour bit
                       | (buffer[2] & 0x80) >> 5  // A2M4 - Day/Date bit
                       | (buffer[2] & 0x40) >> 3; // DY_DT

  // Determine which mode the fetched alarm bits map to
  switch (alarm_mode) {
    case DS3231_A2_PerMinute:
    case DS3231_A2_Minute:
    case DS3231_A2_Hour:
    case DS3231_A2_Date:
    case DS3231_A2_Day:
      return (Ds3231Alarm2Mode)alarm_mode;
    default:
      // Default if the alarm mode cannot be read
      return DS3231_A2_Date;
  }
}

void RTC_DS3231::disableAlarm(uint8_t alarm_num) {
  uint8_t ctrl = read_register(DS3231_CONTROL);
  ctrl &= ~(1 << (alarm_num - 1));
  write_register(DS3231_CONTROL, ctrl);
}

void RTC_DS3231::clearAlarm(uint8_t alarm_num) {
  uint8_t status = read_register(DS3231_STATUSREG);
  status &= ~(0x1 << (alarm_num - 1));
  write_register(DS3231_STATUSREG, status);
}

bool RTC_DS3231::alarmFired(uint8_t alarm_num) {
  return (read_register(DS3231_STATUSREG) >> (alarm_num - 1)) & 0x1;
}

void RTC_DS3231::enable32K(void) {
  uint8_t status = read_register(DS3231_STATUSREG);
  status |= (0x1 << 0x03);
  write_register(DS3231_STATUSREG, status);
}

void RTC_DS3231::disable32K(void) {
  uint8_t status = read_register(DS3231_STATUSREG);
  status &= ~(0x1 << 0x03);
  write_register(DS3231_STATUSREG, status);
}

bool RTC_DS3231::isEnabled32K(void) {
  return (read_register(DS3231_STATUSREG) >> 0x03) & 0x01;
}
