/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include <Controllino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <raibo_msgs/interlock_fb.h>

#define relay_break CONTROLLINO_R0
#define relay_direction CONTROLLINO_R1

#define power_break CONTROLLINO_D0
#define power_unbreak CONTROLLINO_D1
#define vdc_break CONTROLLINO_R2
#define gnd_unbreak CONTROLLINO_R3
#define vdc_unbreak CONTROLLINO_R4
#define gnd_break CONTROLLINO_R5
 
#define breaked_in CONTROLLINO_A1
#define breaked_out CONTROLLINO_D3
#define unbreaked_in CONTROLLINO_A0
#define unbreaked_out CONTROLLINO_D2

ros::NodeHandle  nh;
raibo_msgs::interlock_fb str_interlock_fb;

bool initial_state = true;
bool car_direction = false;
bool car_direction_tmp;
bool break_stop = false;

int breaked_int = 0;
int unbreaked_int = 0;

void breaking(){
  while(breaked_int != 0){
    digitalWrite(power_break, HIGH);
    digitalWrite(vdc_break, HIGH);
    digitalWrite(gnd_break, HIGH);
    digitalWrite(power_unbreak, LOW);
    digitalWrite(gnd_unbreak, LOW);
    digitalWrite(vdc_unbreak, LOW);
    
    breaked_int = digitalRead(breaked_in);
    unbreaked_int = digitalRead(unbreaked_in);
  }
  skip_breaking();
}

void unbreaking(){
  while(unbreaked_int != 0){
    digitalWrite(power_break, LOW);
    digitalWrite(vdc_break, LOW);
    digitalWrite(gnd_break, LOW);
    digitalWrite(power_unbreak, HIGH);
    digitalWrite(gnd_unbreak, HIGH);
    digitalWrite(vdc_unbreak, HIGH);   
    
    breaked_int = digitalRead(breaked_in);
    unbreaked_int = digitalRead(unbreaked_in);
  }
  skip_breaking();
}

void skip_breaking(){
  digitalWrite(power_break, LOW);
  digitalWrite(vdc_break, LOW);
  digitalWrite(gnd_break, LOW);
  digitalWrite(power_unbreak, LOW);
  digitalWrite(gnd_unbreak, LOW);
  digitalWrite(vdc_unbreak, LOW);  
}

void callback( const geometry_msgs::Twist& msg){
  if((double)msg.linear.x  == 0.0 || (double)msg.linear.x  == -0.0){
    break_stop = true;
  }
  if((double)msg.linear.x  > 0.0){
    car_direction = true;
    break_stop = false;
  }
  if((double)msg.linear.x  < 0.0){
    car_direction = false;
    break_stop = false;
  }
  str_interlock_fb.Motor_Break = break_stop;
  str_interlock_fb.Motor_Forward = car_direction;
}


ros::Subscriber<geometry_msgs::Twist> contollino_fb_sub("/cmd_vel", callback );

ros::Publisher contollino_fb_pub("/micro/interlock", &str_interlock_fb);

void setup()
{
  Serial.begin(9600);
  pinMode(relay_break, OUTPUT);
  pinMode(relay_direction, OUTPUT);

  pinMode(power_break, OUTPUT);
  pinMode(power_unbreak, OUTPUT);
  pinMode(vdc_break, OUTPUT);
  pinMode(gnd_unbreak, OUTPUT);
  pinMode(vdc_unbreak, OUTPUT);
  pinMode(gnd_break, OUTPUT);

  pinMode(breaked_in, INPUT);
  pinMode(unbreaked_in, INPUT);
  pinMode(breaked_out, OUTPUT);
  pinMode(unbreaked_out, OUTPUT);

  digitalWrite(breaked_out, HIGH);
  digitalWrite(unbreaked_out, HIGH);
  nh.initNode();
  nh.advertise(contollino_fb_pub);
  nh.subscribe(contollino_fb_sub);
}

void loop()
{
  breaked_int = digitalRead(breaked_in);
  unbreaked_int = digitalRead(unbreaked_in);
  if(initial_state){car_direction_tmp = car_direction; initial_state = false;}
  
  if(break_stop){
    digitalWrite(relay_break, HIGH);
    breaking();
  }
   if(car_direction && !break_stop){
    digitalWrite(relay_direction, HIGH);
    digitalWrite(relay_break, LOW);
    unbreaking();
  }
  else if(!car_direction && !break_stop){
    digitalWrite(relay_direction, LOW);
    digitalWrite(relay_break, LOW);
    unbreaking();
  }
  else{
    digitalWrite(relay_break, HIGH);
    breaking();
  }
  
  if(car_direction_tmp != car_direction){
    if(car_direction){
      digitalWrite(relay_direction, HIGH);
      delay(75);
      digitalWrite(relay_direction, LOW);
      delay(75);
      unbreaking();
    }
    else{
      digitalWrite(relay_direction, LOW);
      delay(75);
      digitalWrite(relay_direction, HIGH);
      delay(75);
      unbreaking();
    }
  }
  car_direction_tmp = car_direction;
  
  contollino_fb_pub.publish( &str_interlock_fb );
  nh.spinOnce();
  delay(50);
}
