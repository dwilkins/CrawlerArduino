#include <Arduino.h>
#include <math.h>
#include "aJSON.h"
#include "main.h"


extern "C" void __cxa_pure_virtual(void) {
    while(1);
}

#include <limits.h>
#include "CrawlerCommand.hpp"

#define IN1 7
#define IN2 5
#define INH 4

#define DRIVE_IN1 24
#define DRIVE_IN2 26
#define DRIVE_INH 28
#define DRIVE_PWM 2
#define DRIVE_TRAIN_MAX_ADJUSTMENT 2

#define STEER_IN1 38
#define STEER_IN2 40
#define STEER_INH 42
#define STEER_PWM 3
#define STEER_MAX_ADJUSTMENT 2

#define STEER_COMMON_PIN A1
#define STEER_LEFT_PIN A2
#define STEER_RIGHT_PIN A3


#define RAMPS_PER_MS (255.0/5.0/1000.0)
#define MS_BETWEEN_RAMPS (1.0/RAMPS_PER_MS)



typedef enum {FORWARD,REVERSE,STOP, LEFT, RIGHT, CENTER} direction_t;
aJsonStream serial_stream(&Serial);

char command_buffer[128];



struct pwm_t {
  int current_pwm;
  int target_pwm;
  int target_direction_pwm;
  direction_t current_direction;
  direction_t target_direction;
  direction_t previous_direction;
  long last_command_time;
  long next_activity_time;
  long next_pwm_adjustment;
  long pwm_max_adjustment;
  int inh_pin;
  int in1_pin;
  int in2_pin;
  int pwm_pin;
};


struct sensors_t {
  int left_actual;
  int right_actual;
  int left_target;
  int right_target;
  long shutoff_time;
};


// Forward declarations

int stop(int num_parameters, CrawlerCommand::parameter_t parameters[]);
int dir(int num_parameters, CrawlerCommand::parameter_t parameters[]);
int speed(int num_parameters, CrawlerCommand::parameter_t parameters[]);
int turn(int num_parameters, CrawlerCommand::parameter_t parameters[]);

void stop_motor(pwm_t *);
void change_motor_direction(pwm_t *, const char *direction);
void change_motor_speed(pwm_t *, float speed);


void process_pwm(pwm_t *);
void process_sensors();

void init_pwm(pwm_t *pwm);
void forward(pwm_t *pwm);
void reverse(pwm_t *pwm);
void brake(pwm_t *pwm);
void json_status();


pwm_t drive_train;
pwm_t steering;
sensors_t sensors;

#define PWMMAX 100
#define PWMMIN 30
#define PWMDELAY 50
#define PWMPIN 6

#define MAX_COMMANDS 5

CrawlerCommand *commands[MAX_COMMANDS] = {NULL};

CrawlerCommand stop_cmd("stop",0,NULL);
// dir fwd;
// dir rev;
CrawlerCommand dir_cmd("dir",1,(int[]){CrawlerCommand::PT_STRING});
// speed .5;
CrawlerCommand speed_cmd("speed",1,(int []){CrawlerCommand::PT_FLOAT});
// turn right .75;
CrawlerCommand turn_cmd("turn",2,(int[]){CrawlerCommand::PT_STRING,CrawlerCommand::PT_FLOAT});


void setup() {
  Serial.begin(115200);

  myPrintln("Startup...");

  stop_cmd.set_command_function(&stop);
  dir_cmd.set_command_function(&dir);
  speed_cmd.set_command_function(&speed);
  turn_cmd.set_command_function(&turn);

  commands[0] = &stop_cmd;
  commands[1] = &dir_cmd;
  commands[2] = &speed_cmd;
  commands[3] = &turn_cmd;

  drive_train.current_pwm = 0;
  drive_train.target_pwm = 0;
  drive_train.current_direction = STOP;
  drive_train.target_direction = STOP;
  drive_train.last_command_time = 0;
  drive_train.next_activity_time = 500;
  drive_train.next_pwm_adjustment = 0;
  drive_train.pwm_max_adjustment = DRIVE_TRAIN_MAX_ADJUSTMENT;
  drive_train.next_pwm_adjustment = drive_train.pwm_max_adjustment;
  drive_train.inh_pin = DRIVE_INH;
  drive_train.in1_pin = DRIVE_IN1;
  drive_train.in2_pin = DRIVE_IN2;
  drive_train.pwm_pin = DRIVE_PWM;

  steering.current_pwm = 0;
  steering.target_pwm = 0;
  steering.current_direction = STOP;
  steering.target_direction = STOP;
  steering.last_command_time = 0;
  steering.next_activity_time = 500;
  steering.pwm_max_adjustment = STEER_MAX_ADJUSTMENT;
  steering.next_pwm_adjustment = steering.pwm_max_adjustment;
  steering.inh_pin = STEER_INH;
  steering.in1_pin = STEER_IN1;
  steering.in2_pin = STEER_IN2;
  steering.pwm_pin = STEER_PWM;



  init_pwm(&drive_train);
  init_pwm(&steering);

  forward(&drive_train);

  myPrintln("Startup Done...");

  json_status();

}

void loop() {
  CrawlerCommand::command_buffer_t *cbp = NULL;
  if((cbp = CrawlerCommand::read_command())) {
    for(int i = 0;i < cbp->num_commands;i++) {
      myPrint("Got a command ---> ");
      myPrintln(cbp->command_strings[i]);
      for(int cmd_num = 0;cmd_num < 4;  cmd_num++) {
        if(commands[cmd_num]->is_match(cbp->command_strings[i])) {
          commands[cmd_num]->execute();
        }
      }
    }
    json_status();
  }
  process_sensors();
  process_pwm(&drive_train);
  process_pwm(&steering);
  // process pwm
}

void init_pwm(pwm_t *pwm) {

  pinMode(pwm->in1_pin,OUTPUT);
  pinMode(pwm->in2_pin,OUTPUT);
  pinMode(pwm->inh_pin,OUTPUT);
  pinMode(pwm->pwm_pin,OUTPUT);

  digitalWrite(pwm->in1_pin,LOW);
  digitalWrite(pwm->in2_pin,LOW);
  digitalWrite(pwm->inh_pin,LOW);
  digitalWrite(pwm->pwm_pin,HIGH);
  analogWrite(pwm->pwm_pin,0);
}

void forward(pwm_t *pwm) {
  digitalWrite(pwm->in1_pin, HIGH);
  digitalWrite(pwm->in2_pin, LOW);
  digitalWrite(pwm->inh_pin, LOW);
}

void reverse(pwm_t *pwm) {
  digitalWrite(pwm->in1_pin, LOW);
  digitalWrite(pwm->in2_pin, HIGH);
  digitalWrite(pwm->inh_pin, LOW);
}

void brake(pwm_t *pwm) {
  analogWrite(pwm->pwm_pin,0);
  digitalWrite(pwm->in1_pin, HIGH);
  digitalWrite(pwm->in2_pin, HIGH);
  digitalWrite(pwm->inh_pin, HIGH);
}

void process_pwm(pwm_t *pwm) {
  long current_time = millis();
  static long last_time = 0;
  if(current_time >= pwm->next_activity_time && pwm->next_activity_time > 0) {
    if(pwm->current_pwm != pwm->target_pwm) {
      // if(current_time > last_time + 1000) {
      //   myPrint("process pwm - ");
      //   myPrintln(current_time);
      //   last_time = current_time;
      // }
      if(pwm->current_pwm > pwm->target_pwm) {
        pwm->current_pwm -= pwm->next_pwm_adjustment;
        if(pwm->current_pwm < pwm->target_pwm) {pwm->current_pwm = pwm->target_pwm;}
      } else if (pwm->current_pwm < pwm->target_pwm) {
        pwm->current_pwm += pwm->next_pwm_adjustment;
        if(pwm->current_pwm > pwm->target_pwm) {pwm->current_pwm = pwm->target_pwm;}
      }
      analogWrite(pwm->pwm_pin,pwm->current_pwm);
      pwm->next_activity_time = current_time + MS_BETWEEN_RAMPS;
    }
  }
  if(pwm->current_pwm == pwm->target_pwm &&
     pwm->current_direction != pwm->target_direction) {
    if(pwm->target_direction == FORWARD || pwm->current_direction == LEFT) {
      forward(pwm);
      pwm->current_direction = pwm->target_direction;
    }
    if(pwm->target_direction == REVERSE || pwm->current_direction == RIGHT) {
      reverse(pwm);
      pwm->current_direction = pwm->target_direction;
    }
    if(pwm->target_direction == STOP || pwm->target_direction == CENTER) {
      brake(pwm);
      pwm->previous_direction = pwm->current_direction;
      pwm->current_direction = pwm->target_direction;
    }
    pwm->target_pwm = pwm->target_direction_pwm;
    pwm->next_activity_time = current_time;
  }
}

void process_sensors() {
  long current_time = millis();
  static long last_read = 0;
  if(current_time >= sensors.shutoff_time) {
    stop_motor(&steering);
     steering.next_activity_time = current_time;
  }

  if((current_time - last_read) > 250) {
    digitalWrite(STEER_COMMON_PIN,HIGH);
    delay(20);
    sensors.left_actual = analogRead(STEER_LEFT_PIN);
    delay(20);
    sensors.right_actual = analogRead(STEER_RIGHT_PIN);
    digitalWrite(STEER_COMMON_PIN,LOW);
    last_read = current_time;
  }



}




int stop(int num_parameters, CrawlerCommand::parameter_t parameters[]) {
  myPrintln("Stopping...");
  stop_motor(&drive_train);
 return 0;
}

void stop_motor(pwm_t *pwm) {
  pwm->target_direction = STOP;
  pwm->target_pwm = 0;
  pwm->next_activity_time = millis();
}


int dir(int num_parameters, CrawlerCommand::parameter_t parameters[]) {
  char *direction = NULL;
  myPrintln("Direction...");
  if(parameters != NULL && num_parameters) {
    direction = (char *)parameters[0].val.s;
    change_motor_direction(&drive_train, direction);
  } else {
    myPrint("dir - Not enough parameters = ");
    myPrintln(num_parameters,DEC);
  }
  return 0;
}

void change_motor_direction(pwm_t *pwm, const char *direction) {
  if(!strcasecmp(direction,"fwd")) {
    pwm->target_direction = FORWARD;
  } else if (!strcasecmp(direction,"left")) {
    pwm->target_direction = LEFT;
  } else if (!strcasecmp(direction,"rev")) {
    pwm->target_direction = REVERSE;
  } else if (!strcasecmp(direction,"right")) {
    pwm->target_direction = RIGHT;
  } else if (!strcasecmp(direction,"center")) {
    pwm->target_direction = CENTER;
  } else if (!strcasecmp(direction,"stop")) {
    pwm->target_direction = STOP;
  }
  pwm->next_activity_time = millis();
  if(pwm->target_direction != pwm->current_direction) {
    pwm->target_pwm = pwm->target_direction_pwm = 0;
  }
}



int speed(int num_parameters, CrawlerCommand::parameter_t parameters[]) {
  myPrintln("Change Speed");
  if(parameters != NULL && num_parameters) {
    float speed = parameters[0].val.f;
    myPrint("Speed is ");
    myPrint(speed,4);
    myPrint(" - pwm will be ");
    myPrintln(ceil((255 * speed)),DEC);
    if (speed >= 0.0 || speed <= 1.0) {
      change_motor_speed(&drive_train,speed);
    }
  } else {
    myPrint("speed - Not enough parameters = ");
    myPrintln(num_parameters,DEC);
  }
  return 0;
}

void change_motor_speed(pwm_t *pwm, float speed) {
  if(pwm->current_direction == pwm->target_direction){
    pwm->target_pwm = ceil((255 * speed));
  } else {
    pwm->target_direction_pwm = ceil((255 * speed));
  }
  if(pwm->current_direction == STOP && pwm->previous_direction != STOP) {
    pwm->target_direction = pwm->previous_direction;
  }
  pwm->next_activity_time = millis();
  pwm->next_pwm_adjustment = pwm->pwm_max_adjustment;

}




int turn(int num_parameters, CrawlerCommand::parameter_t parameters[]) {
  myPrintln("Turning");
  char *direction;
  float percentage;
  if(parameters != NULL && num_parameters == 2) {
    direction = parameters[0].val.s;
    percentage = parameters[1].val.f;
    if(!strcasecmp(direction,"left")) {
      sensors.left_target = percentage;
      sensors.right_target = 0;
      sensors.shutoff_time = millis() + 10000;
      change_motor_direction(&steering,"left");
      change_motor_speed(&steering,1.0);
      steering.next_activity_time = millis();
    } else if (!strcasecmp(direction,"right")) {
      sensors.right_target = percentage;
      sensors.left_target = 0;
      sensors.shutoff_time = millis() + 10000;
      change_motor_direction(&steering,"right");
      change_motor_speed(&steering,1.0);
      steering.next_activity_time = millis();
    } else if (!strcasecmp(direction,"center")) {
      sensors.right_target = 0;
      sensors.left_target = 0;
      sensors.shutoff_time = millis() + 10000;
      change_motor_direction(&steering,"center");
      change_motor_speed(&steering,1.0);
      steering.next_activity_time = millis();
    }
  } else {
    myPrint("turn - Not enough parameters = ");
    myPrintln(num_parameters,DEC);
  }
  return 0;
}




int main(void)
{
  init();
  setup();
  for (;;){
    loop();
  }
  return 0;
}

void json_status() {
  aJsonObject *root;
  aJsonObject *json_sensors;
  aJsonObject *json_commands;
  pwm_t *pwms[] = { &drive_train, &steering };
  const char *pwm_labels[] = { "drive_train", "steering" };
  aJsonObject *json_pwms[2];
  int i;

  root=aJson.createObject();

 aJson.addItemToObject(root,pwm_labels[0], json_pwms[0] = aJson.createObject());
 aJson.addItemToObject(root,pwm_labels[1], json_pwms[1] = aJson.createObject());
 aJson.addItemToObject(root,"sensors", json_sensors = aJson.createObject());
 aJson.addItemToObject(root,"commands", json_commands = aJson.createObject());

 for(i = 0; i < 2 ; i++) {
   aJson.addNumberToObject(json_pwms[i],"current_pwm",pwms[i]->current_pwm);
   aJson.addNumberToObject(json_pwms[i],"target_pwm",pwms[i]->target_pwm);
   aJson.addNumberToObject(json_pwms[i],"current_direction",pwms[i]->current_direction);
   aJson.addNumberToObject(json_pwms[i],"target_direction",pwms[i]->target_direction);
   aJson.addNumberToObject(json_pwms[i],"last_command_time",(double)pwms[i]->last_command_time);
   aJson.addNumberToObject(json_pwms[i],"next_activity_time",(double)pwms[i]->next_activity_time);
   aJson.addNumberToObject(json_pwms[i],"next_pwm_adjustment",(double)pwms[i]->next_pwm_adjustment);
   aJson.addNumberToObject(json_pwms[i],"pwm_max_adjustment",(double)pwms[i]->pwm_max_adjustment);
   aJson.addNumberToObject(json_pwms[i],"inh_pin",pwms[i]->inh_pin);
   aJson.addNumberToObject(json_pwms[i],"in1_pin",pwms[i]->in1_pin);
   aJson.addNumberToObject(json_pwms[i],"in2_pin",pwms[i]->in2_pin);
   aJson.addNumberToObject(json_pwms[i],"pwm_pin",pwms[i]->pwm_pin);
 }
 aJson.addNumberToObject(json_sensors,"left_actual",sensors.left_actual);
 aJson.addNumberToObject(json_sensors,"right_actual",sensors.right_actual);
 aJson.addNumberToObject(json_sensors,"left_target",sensors.left_target);
 aJson.addNumberToObject(json_sensors,"right_target",sensors.right_target);
 aJson.addNumberToObject(json_sensors,"shutoff_time",(float)sensors.shutoff_time);

 for(i = 0 ; i < MAX_COMMANDS ; i++) {
   aJsonObject *current_cmd;
   if(commands[i]) {
     aJson.addItemToObject(json_commands,commands[i]->get_command(), current_cmd = aJson.createObject());
     aJson.addNumberToObject(current_cmd, "num_parameters", commands[i]->get_num_parameters());
     aJson.addItemToObject(current_cmd, "parameter_types", aJson.createStringArray(commands[i]->get_parameter_type_names(),commands[i]->get_num_parameters()));
   }
 }
 aJson.print(root, &serial_stream);
 aJson.deleteItem(root);
 Serial.println("");
}
