int motorright = 9;
int motorrightdir = 7;
int motorleft = 10;
int motorleftdir = 8;
// defines pins numbers
const int echoPinFront = 5;
const int trigPinFront = A4;
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <SoftwareSerial.h>
//ros::NodeHandle nh;

class BlueROS : public ArduinoHardware
{
  protected:

  private:

  SoftwareSerial *mySerial;

  public:
  BlueRos(){}

  void init()
  {
      mySerial = new SoftwareSerial(A0,11);
      mySerial->begin(9600);  
  }

  int read()
  {
      return mySerial->read();
  };

  void write(uint8_t* data, int length)
  {
      for(int i=0; i<length; i++)
      {
        mySerial->write(data[i]);
      }
  }

    
};

ros::NodeHandle_<BlueROS, 3, 3, 100, 100> nh;

std_msgs::Int16 msg;
ros::Publisher pub("sensor", &msg);
void ros_handler( const geometry_msgs::Twist& cmd_msg) {
float x = cmd_msg.linear.x;
float z = cmd_msg.angular.z;
if(x ==-2.0) backward(500);
if(x == 2.0) forward(500);
if(z == 2.0) left(500);
if(z ==-2.0) right(500);
}
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", ros_handler);
// defines variables
long duration;
int distance;
void setup() {
pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
pinMode(motorright, OUTPUT);

pinMode(motorleft, OUTPUT);
pinMode(motorrightdir, OUTPUT);
pinMode(motorleftdir, OUTPUT);
nh.initNode();
nh.subscribe(sub);
nh.advertise(pub);
}
void loop()
{
int distancecenter = ultrasonic(echoPinFront,trigPinFront);
msg.data = distancecenter;
pub.publish( &msg);
nh.spinOnce();
delay(100); //wait for 1/10th of a second.
}
//direction is controlled by the digital pin 7 and 8.
// HIGH is backward, LOW is forward
// Pins 9 and 10 control speed.
// Length of time controls the distance
void forward(int time)
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,180);
digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, 180);
delay(time);
stop();
}

void left(int time)
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,180);

delay(time);
stop();
}
void right(int time)
{

digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, 180);
delay(time);
stop();
}
void backward(int time)
{
digitalWrite(motorrightdir, HIGH);
analogWrite(motorright,180);
digitalWrite(motorleftdir, HIGH);
analogWrite(motorleft, 180);
delay(time);
stop();
}

void stop()
{
analogWrite(motorright, 0);
analogWrite(motorleft, 0);
}
int ultrasonic(int echoPin, int trigPin) {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
return distance;
}
