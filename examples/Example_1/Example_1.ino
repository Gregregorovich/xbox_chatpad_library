#include <Chatpad.h>

Chatpad pad;

void print_keys(Chatpad &pad, Chatpad::keycode_t code,
    Chatpad::eventtype_t type) {
  if (type == Chatpad::Down) {
    char a = pad.toAscii(code);
    if (a =='\t'){
      Serial.println("Pressed right");
    }else if(a=='\v'){
      Serial.println("Pressed left");
    } else if (a != 0){
      Serial.print(a);
      Keyboard.write(a);
      digitalWrite(13, HIGH);
    }   
  }
  if (type == Chatpad::Up) {
      digitalWrite(13, LOW);
  }
}

void setup() {
  delay(1000);
  Serial.begin(57600);
  pad.init(Serial1, print_keys);
  pinMode(13, OUTPUT);
}

void loop() {
  pad.poll();
}
