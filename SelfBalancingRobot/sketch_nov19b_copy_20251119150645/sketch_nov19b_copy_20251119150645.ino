int motor1A = 14;
int motor2A = 27;


int motor1B = 26;
int motor2B = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1A, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2B, OUTPUT);

}

void loop() {


  digitalWrite(motor1A, HIGH);
  digitalWrite(motor2A, LOW);
  digitalWrite(motor1B, HIGH);
  digitalWrite(motor2B, LOW);
  delay(2000);

}
