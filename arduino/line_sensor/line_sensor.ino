void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned char humanReadable[128];
  unsigned char buffer[8];
  int leftSensorValue = analogRead(A0);
  int centerSensorValue = analogRead(A1);
  int rightSensorValue = analogRead(A2);
  buffer[0] = (unsigned char)((leftSensorValue >> 0) & 0xff);
  buffer[1] = (unsigned char)((leftSensorValue >> 8) & 0xff);
  buffer[2] = (unsigned char)((centerSensorValue >> 0) & 0xff);
  buffer[3] = (unsigned char)((centerSensorValue >> 8) & 0xff);
  buffer[4] = (unsigned char)((rightSensorValue >> 0) & 0xff);
  buffer[5] = (unsigned char)((rightSensorValue >> 8) & 0xff);
  buffer[6] = '\r';
  buffer[7] = '\n';

  snprintf(humanReadable, 128, "l: %04d, c: %04d, r: %04d", leftSensorValue, centerSensorValue, rightSensorValue);
  //  Serial.write(buffer, 8);
  Serial.write(humanReadable, strlen(humanReadable));
  Serial.println();
  delay(20);
}
