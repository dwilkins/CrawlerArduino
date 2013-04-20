extern "C" void __cxa_pure_virtual(void) {
    while(1);
}

#include "Arduino.h"
#include "CrawlerCommand.hpp"

#define IN1 7
#define IN2 5
#define INH 4

#define DRIVE_IN1 7
#define DRIVE_IN2 5
#define DRIVE_INH 4
#define DRIVE_PWM 4
#define DRIVE_TRAIN_MAX_ADJUSTMENT 5

#define STEER_IN1 7
#define STEER_IN2 5
#define STEER_INH 4
#define STEER_PWM 4
#define STEER_MAX_ADJUSTMENT 2

#define RAMPS_PER_MS (255.0/5.0/1000.0)
#define MS_BETWEEN_RAMPS (1.0/RAMPS_PER_MS)


typedef enum {FORWARD,REVERSE,STOP, LEFT, RIGHT, CENTER} direction_t;


int stop(int num_parameters, void *parameters[]);
int dir(int num_parameters, void *parameters[]);
int speed(int num_parameters, void *parameters[]);
int turn(int num_parameters, void *parameters[]);

char command_buffer[128];




typedef struct pwm {
  int current_pwm;
  int target_pwm;
  int target_direction_pwm;
  direction_t current_direction;
  direction_t target_direction;
  long last_command_time;
  long next_activity_time;
  long next_pwm_adjustment;
  long pwm_max_adjustment;
  int inh_pin;
  int in1_pin;
  int in2_pin;
  int pwm_pin;
} pwm_t;


void init_pwm(pwm_t *pwm);
void forward(pwm_t *pwm);
void reverse(pwm_t *pwm);
void brake(pwm_t *pwm);



pwm_t drive_train;
pwm_t steering;

#define PWMMAX 100
#define PWMMIN 30
#define PWMDELAY 50
#define PWMPIN 6

#define MAX_COMMANDS 50

CrawlerCommand *commands[MAX_COMMANDS];

CrawlerCommand stop_cmd("stop",0,NULL);
// dir fwd;
// dir rev;
CrawlerCommand dir_cmd("dir",1,(int[]){CrawlerCommand::PT_STRING});
// speed .5;
CrawlerCommand speed_cmd("speed",1,(int[]){CrawlerCommand::PT_FLOAT});
// turn right .75;
CrawlerCommand turn_cmd("turn",2,(int[]){CrawlerCommand::PT_STRING,CrawlerCommand::PT_FLOAT});


void setup() {

  stop_cmd.set_command_function(&stop);
  dir_cmd.set_command_function(&dir);
  speed_cmd.set_command_function(&speed);
  turn_cmd.set_command_function(&turn);

  commands[0] = &stop_cmd;
  commands[1] = &dir_cmd;
  commands[2] = &speed_cmd;
  commands[3] = &turn_cmd;

  memset(command_buffer,0,sizeof(command_buffer));

  drive_train.current_pwm = 0;
  drive_train.target_pwm = 0;
  drive_train.current_direction = STOP;
  drive_train.target_direction = STOP;
  drive_train.last_command_time = 0;
  drive_train.next_activity_time = 0;
  drive_train.next_pwm_adjustment = 0;
  drive_train.pwm_max_adjustment = DRIVE_TRAIN_MAX_ADJUSTMENT;
  drive_train.inh_pin = DRIVE_INH;
  drive_train.in1_pin = DRIVE_IN1;
  drive_train.in2_pin = DRIVE_IN2;
  drive_train.pwm_pin = DRIVE_PWM;

  steering.current_pwm = 0;
  steering.target_pwm = 0;
  steering.current_direction = STOP;
  steering.target_direction = STOP;
  steering.last_command_time = 0;
  steering.next_activity_time = 0;
  steering.next_pwm_adjustment = 0;
  steering.pwm_max_adjustment = STEER_MAX_ADJUSTMENT;
  steering.inh_pin = STEER_INH;
  steering.in1_pin = STEER_IN1;
  steering.in2_pin = STEER_IN2;
  steering.pwm_pin = STEER_PWM;



  Serial.begin(115200);

  init_pwm(&drive_train);
  init_pwm(&steering);


}

void loop() {

  // read_command
  // if complete command
  // for each command
  //   if command.is_match
  //      command.execute
  //      break
  //
  // process pwm


}

int read_command() {
  while (Serial.available() > 0) {
    // display each character to the LCD
  }

  return 1;
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
  if(current_time >= pwm->next_activity_time) {
    if(pwm->current_pwm != pwm->target_pwm) {
      if(pwm->current_pwm > pwm->target_pwm) {
        pwm->current_pwm -= pwm->next_pwm_adjustment;
        if(pwm->current_pwm < pwm->target_pwm) {pwm->current_pwm = pwm->target_pwm;}
      } else if (pwm->current_pwm < pwm->target_pwm) {
        pwm->current_pwm -= pwm->next_pwm_adjustment;
        if(pwm->current_pwm > pwm->target_pwm) {pwm->current_pwm = pwm->target_pwm;}
      }
      analogWrite(pwm->pwm_pin,pwm->current_pwm);
    }
    if(pwm->current_pwm == pwm->target_pwm &&
       pwm->current_direction != pwm->target_direction) {
      pwm->target_pwm = pwm->target_direction_pwm;
      if(pwm->target_direction == FORWARD || pwm->current_direction == RIGHT) {
        forward(pwm);
        pwm->current_direction = pwm->target_direction;
      }
      if(pwm->target_direction == REVERSE || pwm->current_direction == LEFT) {
        reverse(pwm);
        pwm->current_direction = pwm->target_direction;
      }
      if(pwm->target_direction == STOP || pwm->target_direction == CENTER) {
        brake(pwm);
        pwm->current_direction = pwm->target_direction;
      }
    }
    pwm->next_activity_time = current_time + MS_BETWEEN_RAMPS;
  }
}



int stop(int num_parameters, void *parameters[]) {
  Serial.println("Stopping...");
  drive_train.target_direction = STOP;
  return 0;
}

int dir(int num_parameters, void *parameters[]) {
  char *direction = NULL;
  Serial.println("Direction...");
  if(parameters != NULL && parameters[0] != NULL) {
    direction = (char *)parameters[0];
    if(!strcasecmp(direction,"fwd")) {
      drive_train.target_direction = FORWARD;
    } else if (!strcasecmp(direction,"rev")) {
      drive_train.target_direction = REVERSE;
    } else if (!strcasecmp(direction,"stop")) {
      drive_train.target_direction = STOP;
    }
  }
  return 0;
}

int speed(int num_parameters, void *parameters[]) {
  Serial.println("Change Speed");
  if(parameters != NULL && parameters[0] != NULL) {
    float speed = *(float *)parameters[0];
    if (speed >= 0.0 || speed <= 1.0) {
      drive_train.target_pwm = (int)(255 * speed);
    }
  }
  return 0;
}
int turn(int num_parameters, void *parameters[]) {
  Serial.println("Turning");
  return 0;
}




int main(void)
{
	init();

	setup();

	for (;;)
		loop();

	return 0;
}
