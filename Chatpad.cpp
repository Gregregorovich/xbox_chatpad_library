<<<<<<< HEAD
/*
 * Copyright (C) 2011 Cliff L. Biffle, all rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "Chatpad.h"
#include "HardwareSerial.h"
#include <Arduino.h>

#include <avr/pgmspace.h>

// Masks for modifier bits
static const byte kShiftMask = (1 << 0);
static const byte kGreenSquareMask = (1 << 1);
static const byte kOrangeCircleMask = (1 << 2);
static const byte kPeopleMask = (1 << 3);

// Voodoo protocol messages
static const byte kInitMessage[] = { 0x87, 0x02, 0x8C, 0x1F, 0xCC };
static const byte kAwakeMessage[] = { 0x87, 0x02, 0x8C, 0x1B, 0xD0 };

void Chatpad::init(HardwareSerial &serial, Chatpad::callback_t callback) {
  _serial = &serial;
  _callback = callback;
  _last_modifiers = 0;
  _last_key0 = 0;
  _last_key1 = 0;
  _last_ping = 0;
  _serial->begin(19200);
  _serial->write(kInitMessage, sizeof(kInitMessage));
}

void Chatpad::poll() {


  // Only act if a full message is available.
  if (_serial->available() >= 8) {
    for (int i = 0; i < 8; i++) {
      _buffer[i] = _serial->read();
    }

    //Serial.println("no pool..");

    // We expect "status report" packets beginning with 0xA5, but don't know
    // what to do with them -- so we silently discard them.
    if (_buffer[0] == 0xA5) return;

    // We *do not* expect other types of packets.  If we find one, complain
    // to the user.
    if (_buffer[0] != 0xB4) {
      Serial.print("Unexpected packet type: ");
      Serial.println(_buffer[0], HEX);
      return;
    }
    if (_buffer[1] != 0xC5) {
      Serial.print("Unexpected second byte: ");
      Serial.println(_buffer[1], HEX);
      return;
    }

    // Check the checksum.
    unsigned char checksum = _buffer[0];
    for (int i = 1; i < 7; i++) checksum += _buffer[i];
    checksum = -checksum;
    if (checksum != _buffer[7]) {
      Serial.println("Checksum failure");
      return;
    }

    // Packet looks good!
    // Dissect the parts we care about:
    byte modifiers = _buffer[3];
    byte key0 = _buffer[4];
    byte key1 = _buffer[5];

    // Check for changes in the modifiers.
    byte modifier_changes = modifiers ^ _last_modifiers;
    if (modifier_changes & kShiftMask) {
      dispatch(KeyShift, modifiers & kShiftMask);
    }
    if (modifier_changes & kGreenSquareMask) {
      dispatch(KeyGreenSquare, modifiers & kGreenSquareMask);
    }
    if (modifier_changes & kOrangeCircleMask) {
      dispatch(KeyOrangeCircle, modifiers & kOrangeCircleMask);
    }
    if (modifier_changes & kPeopleMask) {
      dispatch(KeyPeople, modifiers & kPeopleMask);
    }
    _last_modifiers = modifiers;

    // Check for changes in the other keys
    if (key0 && key0 != _last_key0 && key0 != _last_key1) {
      dispatch(key0, Down);
    }
    if (key1 && key1 != _last_key0 && key1 != _last_key1) {
      dispatch(key1, Down);
    }
    if (_last_key0 && _last_key0 != key0 && _last_key0 != key1) {
      dispatch(_last_key0, Up);
    }
    if (_last_key1 && _last_key1 != key0 && _last_key1 != key1) {
      dispatch(_last_key1, Up);
    }
    _last_key0 = key0;
    _last_key1 = key1;
  }

  uint32_t time = millis();
  if (time - _last_ping > 1000) {
    _last_ping = time;
    _serial->write(kAwakeMessage, sizeof(kAwakeMessage));
  }
}

bool Chatpad::isShiftDown() const {
  return _last_modifiers & kShiftMask;
}

bool Chatpad::isGreenSquareDown() const {
  return _last_modifiers & kGreenSquareMask;
}

bool Chatpad::isOrangeCircleDown() const {
  return _last_modifiers & kOrangeCircleMask;
}

bool Chatpad::isPeopleDown() const {
  return _last_modifiers & kPeopleMask;
}

void Chatpad::dispatch(uint8_t code, int is_down) {
  _callback(*this, (keycode_t) code, is_down? Down : Up);
}

/**************
 * Translation
 */

// These tables have been compacted and must be accessed using this formula:
//  index = (((keycode & 0xF0) - 0x10) >> 1) | ((keycode & 0x0F) - 1)
static const char kAsciiTable[] PROGMEM = {
  '7', /* 11 Key7 */
  '6', /* 12 Key6 */
  '5', /* 13 Key5 */
  '4', /* 14 Key4 */
  '3', /* 15 Key3 */
  '2', /* 16 Key2 */
  '1', /* 17 Key1 */
  0, /* 18 Unused */

  'u', /* 21 KeyU */
  'y', /* 22 KeyY */
  't', /* 23 KeyT */
  'r', /* 24 KeyR */
  'e', /* 25 KeyE */
  'w', /* 26 KeyW */
  'q', /* 27 KeyQ */
  0, /* 28 Unused */

  'j', /* 31 KeyJ */
  'h', /* 32 KeyH */
  'g', /* 33 KeyG */
  'f', /* 34 KeyF */
  'd', /* 35 KeyD */
  's', /* 36 KeyS */
  'a', /* 37 KeyA */
  0, /* 38 Unused */

  'n', /* 41 KeyN */
  'b', /* 42 KeyB */
  'v', /* 43 KeyV */
  'c', /* 44 KeyC */
  'x', /* 45 KeyX */
  'z', /* 46 KeyZ */
  0, /* 47 Unused */
  0, /* 48 Unused */

  '\t', /* 51 KeyRight */
  'm', /* 52 KeyM */
  '.', /* 53 KeyPeriod */
  ' ', /* 54 KeySpace */
  '\v', /* 55 KeyLeft */
  0, /* 56 Unused */
  0, /* 57 Unused */
  0, /* 58 Unused */

  0, /* 61 Unused */
  ',', /* 62 KeyComma */
  '\n', /* 63 KeyEnter */
  'p', /* 64 KeyP */
  '0', /* 65 Key0 */
  '9', /* 66 Key9 */
  '8', /* 67 Key8 */
  0, /* 68 Unused */

  '\b', /* 71 KeyBackspace */
  'l', /* 72 KeyL */
  0, /* 73 Unused */
  0, /* 74 Unused */
  'o', /* 75 KeyO */
  'i', /* 76 KeyI */
  'k', /* 77 KeyK */
  0, /* 78 Unused */
};

static const char kAsciiTable_Shifted[] PROGMEM = {
  '&', /* 11 Key7 */
  '^', /* 12 Key6 */
  '%', /* 13 Key5 */
  '$', /* 14 Key4 */
  '#', /* 15 Key3 */
  '@', /* 16 Key2 */
  '!', /* 17 Key1 */
  0, /* 18 Unused */

  'U', /* 21 KeyU */
  'Y', /* 22 KeyY */
  'T', /* 23 KeyT */
  'R', /* 24 KeyR */
  'E', /* 25 KeyE */
  'W', /* 26 KeyW */
  'Q', /* 27 KeyQ */
  0, /* 28 Unused */

  'J', /* 31 KeyJ */
  'H', /* 32 KeyH */
  'G', /* 33 KeyG */
  'F', /* 34 KeyF */
  'D', /* 35 KeyD */
  'S', /* 36 KeyS */
  'A', /* 37 KeyA */
  0, /* 38 Unused */

  'N', /* 41 KeyN */
  'B', /* 42 KeyB */
  'V', /* 43 KeyV */
  'C', /* 44 KeyC */
  'X', /* 45 KeyX */
  'Z', /* 46 KeyZ */
  0, /* 47 Unused */
  0, /* 48 Unused */

  '\t', /* 51 KeyRight */
  'M', /* 52 KeyM */
  '>', /* 53 KeyPeriod */
  ' ', /* 54 KeySpace */
  '\v', /* 55 KeyLeft */
  0, /* 56 Unused */
  0, /* 57 Unused */
  0, /* 58 Unused */

  0, /* 61 Unused */
  '<', /* 62 KeyComma */
  '\n', /* 63 KeyEnter */
  'P', /* 64 KeyP */
  ')', /* 65 Key0 */
  '(', /* 66 Key9 */
  '*', /* 67 Key8 */
  0, /* 68 Unused */

  '\b', /* 71 KeyBackspace */
  'L', /* 72 KeyL */
  0, /* 73 Unused */
  0, /* 74 Unused */
  'O', /* 75 KeyO */
  'I', /* 76 KeyI */
  'K', /* 77 KeyK */
  0, /* 78 Unused */
};

static const char kAsciiTable_Orange[] PROGMEM = {
  0, /* 11 Key7 */
  0, /* 12 Key6 */
  0, /* 13 Key5 */
  0, /* 14 Key4 */
  0, /* 15 Key3 */
  0, /* 16 Key2 */
  0, /* 17 Key1 */
  0, /* 18 Unused */

  0, /* 21 KeyU */
  0, /* 22 KeyY */
  0, /* 23 KeyT */
  '$', /* 24 KeyR */
  0, /* 25 KeyE */
  0, /* 26 KeyW */
  0, /* 27 KeyQ */
  0, /* 28 Unused */

  '"', /* 31 KeyJ */
  '\\', /* 32 KeyH */
  0, /* 33 KeyG */
  0, /* 34 KeyF */
  0, /* 35 KeyD */
  0, /* 36 KeyS */
  0, /* 37 KeyA */
  0, /* 38 Unused */

  0, /* 41 KeyN */
  '+', /* 42 KeyB */
  '-', /* 43 KeyV */
  0, /* 44 KeyC */
  0, /* 45 KeyX */
  0, /* 46 KeyZ */
  0, /* 47 Unused */
  0, /* 48 Unused */

  '\t', /* 51 KeyRight */
  0, /* 52 KeyM */
  0, /* 53 KeyPeriod */
  ' ', /* 54 KeySpace */
  '\v', /* 55 KeyLeft */
  0, /* 56 Unused */
  0, /* 57 Unused */
  0, /* 58 Unused */

  0, /* 61 Unused */
  ';', /* 62 KeyComma */
  '\n', /* 63 KeyEnter */
  '=', /* 64 KeyP */
  0, /* 65 Key0 */
  0, /* 66 Key9 */
  0, /* 67 Key8 */
  0, /* 68 Unused */

  '\b', /* 71 KeyBackspace */
  0, /* 72 KeyL */
  0, /* 73 Unused */
  0, /* 74 Unused */
  0, /* 75 KeyO */
  0, /* 76 KeyI */
  0, /* 77 KeyK */
  0, /* 78 Unused */
};

static const char kAsciiTable_Green[] PROGMEM = {
  0, /* 11 Key7 */
  0, /* 12 Key6 */
  0, /* 13 Key5 */
  0, /* 14 Key4 */
  0, /* 15 Key3 */
  0, /* 16 Key2 */
  0, /* 17 Key1 */
  0, /* 18 Unused */

  '&', /* 21 KeyU */
  '^', /* 22 KeyY */
  '%', /* 23 KeyT */
  '#', /* 24 KeyR */
  0, /* 25 KeyE */
  '@', /* 26 KeyW */
  '!', /* 27 KeyQ */
  0, /* 28 Unused */

  '\'', /* 31 KeyJ */
  '/', /* 32 KeyH */
  0, /* 33 KeyG */
  '}', /* 34 KeyF */
  '{', /* 35 KeyD */
  0, /* 36 KeyS */
  '~', /* 37 KeyA */
  0, /* 38 Unused */

  '<', /* 41 KeyN */
  '|', /* 42 KeyB */
  '_', /* 43 KeyV */
  0, /* 44 KeyC */
  0, /* 45 KeyX */
  '`', /* 46 KeyZ */
  0, /* 47 Unused */
  0, /* 48 Unused */

  '\t', /* 51 KeyRight */
  '>', /* 52 KeyM */
  '?', /* 53 KeyPeriod */
  ' ', /* 54 KeySpace */
  '\v', /* 55 KeyLeft */
  0, /* 56 Unused */
  0, /* 57 Unused */
  0, /* 58 Unused */

  0, /* 61 Unused */
  ':', /* 62 KeyComma */
  '\n', /* 63 KeyEnter */
  ')', /* 64 KeyP */
  0, /* 65 Key0 */
  0, /* 66 Key9 */
  0, /* 67 Key8 */
  0, /* 68 Unused */

  '\b', /* 71 KeyBackspace */
  ']', /* 72 KeyL */
  0, /* 73 Unused */
  0, /* 74 Unused */
  '(', /* 75 KeyO */
  '*', /* 76 KeyI */
  '[', /* 77 KeyK */
  0, /* 78 Unused */
};

char Chatpad::toAscii(keycode_t code) {
  byte index = (((code - 0x11) & 0x70) >> 1) | ((code - 0x11) & 0x7);
  if (index >= sizeof(kAsciiTable)) return 0;

  if (isShiftDown()) {
    return pgm_read_byte_near(kAsciiTable_Shifted + index);
  } else if (isOrangeCircleDown() || isPeopleDown() ) {
    return pgm_read_byte_near(kAsciiTable_Orange + index);
  } else if (isGreenSquareDown()) {
    return pgm_read_byte_near(kAsciiTable_Green + index);
  } else {
    return pgm_read_byte_near(kAsciiTable + index);
  }
=======
#include "Chatpad.h"

Chatpad::Chatpad() {}

void Chatpad::Init(Stream &serialstream) {
  m_serialstream = &serialstream;
  m_commandkeys = 0;
  m_timer.init(1000);

  chatpad_init();
  chatpad_keep_awake();
}

void Chatpad::Init(Stream &serialstream, void (*f_press)(char),
                   void (*f_release)(char)) {
  pressHandler(f_press);
  releaseHandler(f_release);

  Init(serialstream);
}

void Chatpad::pressHandler(void (*f_press)(char)) { handler_onPress = f_press; }

void Chatpad::releaseHandler(void (*f_release)(char)) {
  handler_onRelease = f_release;
}

void Chatpad::bufferUpdateHandler(void (*f_buffer_update)(uint8_t[8])) {
  handler_onBufferUpdate = f_buffer_update;
}

void Chatpad::run() {
  if (m_timer.getTimer()) {
    chatpad_keep_awake();
  }

  if (m_serialstream->available() >= 8) {
    m_serialstream->readBytes(m_buffer, 8);
    if ((m_buffer[0] != 0xB4) && (m_buffer[0] != 0xA5)) {
      chatpad_fix_stream();
      return;
    }
    if (m_buffer[0] == 0xB4) {
      chatpad_onBufferUpdate(m_buffer);
      buffer_evaluate();
    }
  }
}

void Chatpad::commandkeys_evaluate(uint8_t modifiers) {
  static uint8_t mods[] = {MODIFIER_SHIFT | MODIFIER_RIGHTCTRL, MODIFIER_SHIFT, MODIFIER_MESSENGER};
  static uint8_t com[] = {COMMANDKEY_CAPS_LOCK, COMMANDKEY_SHIFT, COMMANDKEY_CTRL};
  static uint8_t keys[] = {KEY_CAPS_LOCK, KEY_LEFT_SHIFT, KEY_LEFT_CTRL};

  for (int i = 0; i < 3; i++) {
    if (((modifiers & mods[i]) == mods[i]) && !(m_commandkeys & com[i])) {
      m_commandkeys |= com[i];
      chatpad_onPress(keys[i]);
    } else if (!((modifiers & mods[i]) == mods[i]) && (m_commandkeys & com[i])) {
      m_commandkeys &= ~com[i];
      chatpad_onRelease(keys[i]);
    }
  }
}

void Chatpad::buffer_evaluate() {
  commandkeys_evaluate(m_buffer[3]);

  char k = chatpad_to_char(m_buffer[4], m_buffer[3]);

  if (k != 0 && m_lastkeys[0] == 0) { // k1 is pressed
    chatpad_onPress(k);
    m_lastkeys[0] = k;
    return;
  }
  if (m_lastkeys[0] != 0 && k == 0) { // k1 is released
    chatpad_onRelease(m_lastkeys[0]);
    m_lastkeys[0] = 0;
    return;
  }
  if (k != 0 && k == m_lastkeys[1]) { // k2 switched to k1
    chatpad_onRelease(m_lastkeys[0]);
    m_lastkeys[0] = m_lastkeys[1];
    m_lastkeys[1] = 0;
    return;
  }
  if (k != 0 && m_lastkeys[0] != 0 &&
      k != m_lastkeys[0]) { // k1 changed char (maybe caps?..)
    chatpad_onRelease(m_lastkeys[0]);
    chatpad_onPress(k);
    m_lastkeys[0] = k;
    return;
  }

  k = chatpad_to_char(m_buffer[5], m_buffer[3]);

  if (k != 0 && m_lastkeys[1] == 0) { // k2 is pressed
    chatpad_onPress(k);
    m_lastkeys[1] = k;
    return;
  }
  if (m_lastkeys[1] != 0 && k == 0) { // k2 is released
    chatpad_onRelease(m_lastkeys[1]);
    m_lastkeys[1] = 0;
  }
}

void Chatpad::chatpad_onPress(char key) {
  if (handler_onPress)
    handler_onPress(key);
}

void Chatpad::chatpad_onRelease(char key) {
  if (handler_onRelease)
    handler_onRelease(key);
}

void Chatpad::chatpad_onBufferUpdate(uint8_t buf[8]) {
  if (handler_onBufferUpdate)
    handler_onBufferUpdate(buf);
}

void Chatpad::chatpad_fix_stream() {
  for (int i = 1; i < 8; i++) {
    if ((m_buffer[i] == 0xB4) || (m_buffer[i] == 0xA5)) {
      while (m_serialstream->available() < i)
        ;
      m_serialstream->readBytes(m_buffer, i);
      break;
    }
  }
}

void Chatpad::chatpad_init() {
  m_serialstream->write(0x87);
  m_serialstream->write(0x02);
  m_serialstream->write(0x8C);
  m_serialstream->write(0x1F);
  m_serialstream->write(0xCC);
  m_serialstream->flush();
}

void Chatpad::chatpad_keep_awake() {
  m_serialstream->write(0x87);
  m_serialstream->write(0x02);
  m_serialstream->write(0x8C);
  m_serialstream->write(0x1B);
  m_serialstream->write(0xD0);
  m_serialstream->flush();
}

char Chatpad::chatpad_to_char(uint8_t chatpad_byte, uint8_t modifier) {
  uint8_t c = 0;
  switch (chatpad_byte) {
  case 0x65:
    return '0';
  case 0x17:
    return '1';
  case 0x16:
    return '2';
  case 0x15:
    return '3';
  case 0x14:
    return '4';
  case 0x13:
    return '5';
  case 0x12:
    return '6';
  case 0x11:
    return '7';
  case 0x67:
    return '8';
  case 0x66:
    return '9';
  case 0x37:
    c = 'a';
    break;
  case 0x42:
    c = 'b';
    break;
  case 0x44:
    c = 'c';
    break;
  case 0x35:
    c = 'd';
    break;
  case 0x25:
    c = 'e';
    break;
  case 0x34:
    c = 'f';
    break;
  case 0x33:
    c = 'g';
    break;
  case 0x32:
    c = 'h';
    break;
  case 0x76:
    c = 'i';
    break;
  case 0x31:
    c = 'j';
    break;
  case 0x77:
    c = 'k';
    break;
  case 0x72:
    c = 'l';
    break;
  case 0x52:
    c = 'm';
    break;
  case 0x41:
    c = 'n';
    break;
  case 0x75:
    c = 'o';
    break;
  case 0x64:
    c = 'p';
    break;
  case 0x27:
    c = 'q';
    break;
  case 0x24:
    c = 'r';
    break;
  case 0x36:
    c = 's';
    break;
  case 0x23:
    c = 't';
    break;
  case 0x21:
    c = 'u';
    break;
  case 0x43:
    c = 'v';
    break;
  case 0x26:
    c = 'w';
    break;
  case 0x45:
    c = 'x';
    break;
  case 0x22:
    c = 'y';
    break;
  case 0x46:
    c = 'z';
    break;
  case 0x54:
    return ' ';
  case 0x62:
    c = ',';
    break;
  case 0x53:
    c = '.';
    break;
  case 0x71:
    return KEY_BACKSPACE;
  case 0x51:
    c = KEY_RIGHT_ARROW;
    break;
  case 0x55:
    c = KEY_LEFT_ARROW;
    break;
  case 0x63:
    return KEY_RETURN;
  default:
    return 0;
  }

  if (modifier & MODIFIER_SHIFT) {
    if (c >= 'a' && c <= 'z')
      return c - 32;
  }

  if (modifier & MODIFIER_LEFTCTRL) {
    switch (c) {
    case 'q':
      return '!';
    case 'w':
      return '@';
    case 'r':
      return '#';
    case 't':
      return '%';
    case 'y':
      return '^';
    case 'u':
      return '&';
    case 'i':
      return '*';
    case 'o':
      return '(';
    case 'p':
      return ')';
    case 'a':
      return '~';
    case 'd':
      return '{';
    case 'f':
      return '}';
    case 'h':
      return '/';
    case 'j':
      return '\'';
    case 'k':
      return '[';
    case 'l':
      return ']';
    case ',':
      return ':';
    case 'z':
      return '`';
    case 'v':
      return '-';
    case 'b':
      return '|';
    case 'n':
      return '<';
    case 'm':
      return '>';
    case '.':
      return '?';
    case KEY_RIGHT_ARROW:
      return KEY_DOWN_ARROW;
    case KEY_LEFT_ARROW:
      return KEY_UP_ARROW;
    }
  }

  if (modifier & MODIFIER_RIGHTCTRL) {
    switch (c) {
    case 'r':
      return '$';
    case 'p':
      return '=';
    case 'h':
      return '\\';
    case 'j':
      return '"';
    case ',':
      return ';';
    case 'v':
      return '_';
    case 'b':
      return '+';
    case KEY_RIGHT_ARROW:
      return KEY_DOWN_ARROW;
    case KEY_LEFT_ARROW:
      return KEY_UP_ARROW;
    }
  }

  return c;
>>>>>>> 6ee0d234b298f4f2b0b331fd1c4ba6291add3504
}
