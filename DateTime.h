// DateTime and TimeSpan objects - extract of RTClib

#define SECONDS_PER_DAY 86400L // 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800 // Unixtime for 2000-01-01 00:00:00, useful for initialization

uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}
uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}

const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

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

static uint8_t conv2d(const char *p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

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
      TIMESTAMP_FULL, // `YYYY-MM-DDThh:mm:ss`
      TIMESTAMP_TIME, // `hh:mm:ss`
      TIMESTAMP_DATE  // `YYYY-MM-DD`
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

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
  if (year >= 2000U)
    year -= 2000U;
  yOff = year;
  m = month;
  d = day;
  hh = hour;
  mm = min;
  ss = sec;
}

DateTime::DateTime(const DateTime & copy)
: yOff(copy.yOff), m(copy.m), d(copy.d), hh(copy.hh), mm(copy.mm),
ss(copy.ss)
{}

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

DateTime::DateTime(const __FlashStringHelper * date, const __FlashStringHelper * time) {
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

DateTime DateTime::operator+(const TimeSpan & span) const {
  return DateTime(unixtime() + span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan & span) const {
  return DateTime(unixtime() - span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime & right) const {
  return TimeSpan(unixtime() - right.unixtime());
}

bool DateTime::operator<(const DateTime & right) const {
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

bool DateTime::operator==(const DateTime & right) const {
  return (right.year() == yOff + 2000U && right.month() == m &&
  right.day() == d && right.hour() == hh && right.minute() == mm &&
  right.second() == ss);
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
      sprintf(buffer, "%u-%02d-%02dT%02d:%02d:%02d", 2000U + yOff, m, d, hh, mm,
      ss);
  }
  return String(buffer);
}

TimeSpan::TimeSpan(int32_t seconds) : _seconds(seconds) {}

TimeSpan::TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds)
: _seconds((int32_t)days * 86400L + (int32_t)hours * 3600 +
           (int32_t)minutes * 60 + seconds) {}

TimeSpan::TimeSpan(const TimeSpan & copy) : _seconds(copy._seconds) {}

TimeSpan TimeSpan::operator+(const TimeSpan & right) const {
  return TimeSpan(_seconds + right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan & right) const {
  return TimeSpan(_seconds - right._seconds);
}
