#define STOP      0
#define RUN       1
#define BACK      2
#define LEFT      3
#define RIGHT     4
#define SOFT_LEFT 5   // 左缓弯
#define SOFT_RIGHT 6  // 右缓弯
#define SHARP_LEFT 7  // 左锐角弯（直角）
#define SHARP_RIGHT 8 // 右锐角弯（直角）

// 电机引脚
int a1 = 10;    // 右电机IN1
int a2 = 9;     // 右电机IN2
int b1 = 7;     // 左电机IN3
int b2 = 8;     // 左电机IN4

//电机使能pwm
int enA = 11;   // 右电机PWM使能
int enB = 6;    // 左电机PWM使能

// 传感器引脚（从左到右排列）
int leftOuterIR = 2;  // 左外
int leftInnerIR = 3;  // 左内
int rightInnerIR = 4; // 右内
int rightOuterIR = 5; // 右外

// 控制参数（优化后的值）
int baseSpeed = 90;        // 提高基础速度
int softTurnSpeed = 80;    // 缓转速度（单侧减速）
int sharpTurnSpeed = 90;   // 急转速度（单侧反转）
int recoverSpeed = 80;     // 回归轨道时的速度

// PID控制参数（优化后的值）
float Kp = 12.0;        // 降低比例系数
float Kd = 10.0;         // 提高微分系数
float error = 0;       
float lastError = 0;    
float derivative = 0;  
float pidOutput = 0;   

// 转向状态管理
int currentTurn = 0;    // 0-无转向，1-左转向，2-右转向
unsigned long turnStartTime = 0; // 转向开始时间
const unsigned long sharpTurnDuration = 7000; // 锐角弯持续时间(ms)
const unsigned long maxTurnTime = 8000;      // 最大转向时间(ms)
bool isRecovering = false; // 是否正在恢复寻迹状态

void setup() {
  // 初始化电机引脚
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(enA, OUTPUT);  // 初始化右电机PWM使能引脚
  pinMode(enB, OUTPUT);  // 初始化左电机PWM使能引脚
  
  // 初始化传感器引脚
  pinMode(leftOuterIR, INPUT_PULLUP); // 使用上拉电阻，检测到黑线时为LOW
  pinMode(leftInnerIR, INPUT_PULLUP);
  pinMode(rightInnerIR, INPUT_PULLUP);
  pinMode(rightOuterIR, INPUT_PULLUP);
  
  Serial.begin(9600); // 用于调试
}

void loop() {
  // 检测到黑线时为HIGH
  bool LOuter = digitalRead(leftOuterIR) == HIGH;
  bool LInner = digitalRead(leftInnerIR) == HIGH;
  bool RInner = digitalRead(rightInnerIR) == HIGH;
  bool ROuter = digitalRead(rightOuterIR) == HIGH;

  // 计算PID误差
  calculateError(LOuter, LInner, RInner, ROuter);
  
  // 执行PID计算
  computePID();

  // 检查转向超时
  if (currentTurn != 0 && millis() - turnStartTime > maxTurnTime) {
    currentTurn = 0; // 强制结束转向
    isRecovering = false;
  }

  // 循迹逻辑（优先级从高到低）
  if (LOuter && LInner && RInner && ROuter) {
    // 所有传感器检测到线：十字路口或停车线 - 继续行走
    drive(RUN, baseSpeed, baseSpeed);
    currentTurn = 0;
    isRecovering = false;
  } 
  else if ((LOuter || (LOuter && LInner)) && !(ROuter || RInner)) {
    // 左急弯（直角或锐角）：左外或左外+左内触发
    startTurn(1);
    isRecovering = false;
    drive(SHARP_LEFT, sharpTurnSpeed, sharpTurnSpeed);
    delay(100);
  } 
  else if ((ROuter || (ROuter && RInner)) && !(LOuter || LInner)) {
    // 右急弯（直角或锐角）：右外或右外+右内触发
    startTurn(2);
    isRecovering = false;
    drive(SHARP_RIGHT, sharpTurnSpeed, sharpTurnSpeed);
    delay(100);
  }
  else if (LInner && !RInner) {
    // 左内检测到线：轻微右转（右轮减速）
    drive(SOFT_RIGHT, baseSpeed, softTurnSpeed);
    currentTurn = 0; // 结束之前的转向状态
    isRecovering = false;
  } 
  else if (!LInner && RInner) {
    // 右内检测到线：轻微左转（左轮减速）
    drive(SOFT_LEFT, softTurnSpeed, baseSpeed);
    currentTurn = 0; // 结束之前的转向状态
    isRecovering = false;
  } 
  else if (!LInner && !RInner && !LOuter && !ROuter) {
    // 所有传感器未检测到线
    if (currentTurn != 0) {
      // 继续之前的转向或开始恢复寻迹
      if (!isRecovering) {
        isRecovering = true;
        turnStartTime = millis(); // 重置计时器用于恢复状态
      }
      continueTurn();
    } else {
      // 直行 + PID微调
      drive(RUN, baseSpeed + pidOutput, baseSpeed - pidOutput);
      isRecovering = false;
    }
  }
  else {
    // 其他情况：使用PID控制直行
    drive(RUN, baseSpeed + pidOutput, baseSpeed - pidOutput);
    isRecovering = false;
  }

  // 检查锐角弯是否完成
  if (isRecovering) {
    // 检测是否已回到轨道
    if ((currentTurn == 1 && (LInner || RInner)) || 
        (currentTurn == 2 && (RInner || LInner))) {
      currentTurn = 0;
      isRecovering = false;
    }
  }

  delay(10); // 控制循环频率
}

// 开始新的转向
void startTurn(int direction) {
  if (currentTurn != direction) {
    currentTurn = direction;
    turnStartTime = millis();
    isRecovering = false;
  }
}

// 继续转向
void continueTurn() {
  if (currentTurn == 1) {
    drive(SHARP_LEFT, recoverSpeed, recoverSpeed);
  } else if (currentTurn == 2) {
    drive(SHARP_RIGHT, recoverSpeed, recoverSpeed);
  }
}

// 计算PID误差（优化）
void calculateError(bool LOuter, bool LInner, bool RInner, bool ROuter) {
  // 更精确的误差计算
  if (LOuter && !ROuter) error = -3.0;        // 强左偏
  else if (LOuter && ROuter) error = lastError; // 十字路口保持原误差
  else if (LInner && !RInner) error = -1.5;   // 中左偏
  else if (!LInner && RInner) error = 1.5;    // 中右偏
  else if (ROuter && !LOuter) error = 3.0;    // 强右偏
  else if (LInner && RInner) error = 0;       // 居中
  else error = lastError;                     // 其他情况保持误差
}

// 执行PID计算（优化）
void computePID() {
  // 仅当不需要急转弯时使用PID
  if (currentTurn == 0 && !isRecovering) {  
    derivative = error - lastError;
    pidOutput = Kp * error + Kd * derivative;
    
    // 限制输出范围
    if (pidOutput > baseSpeed) pidOutput = baseSpeed;
    if (pidOutput < -baseSpeed) pidOutput = -baseSpeed;
    
    lastError = error;
  } else {
    // 转向时重置PID
    pidOutput = 0;
    lastError = 0;
  }
}

// 电机驱动函数（优化）
void drive(int mode, int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  switch (mode) {
    case RUN: // 直行
      // 左电机正转
      digitalWrite(b1, LOW);
      digitalWrite(b2, HIGH);
      analogWrite(enB, leftSpeed);
      // 右电机正转
      digitalWrite(a1, HIGH);
      digitalWrite(a2, LOW);
      analogWrite(enA, rightSpeed);
      break;

    case SOFT_LEFT: // 左缓弯
      // 左电机减速正转
      digitalWrite(b1, LOW);
      digitalWrite(b2, HIGH);
      analogWrite(enB, leftSpeed);
      // 右电机全速正转
      digitalWrite(a1, HIGH);
      digitalWrite(a2, LOW);
      analogWrite(enA, rightSpeed);
      break;

    case SOFT_RIGHT: // 右缓弯
      // 左电机全速正转
      digitalWrite(b1, LOW);
      digitalWrite(b2, HIGH);
      analogWrite(enB, leftSpeed);
      // 右电机减速正转
      digitalWrite(a1, HIGH);
      digitalWrite(a2, LOW);
      analogWrite(enA, rightSpeed);
      break;

    case SHARP_LEFT: // 左急弯（直角）
      // 左电机反转
      digitalWrite(b1, HIGH);
      digitalWrite(b2, LOW);
      analogWrite(enB, leftSpeed);
      // 右电机正转
      digitalWrite(a1, HIGH);
      digitalWrite(a2, LOW);
      analogWrite(enA, rightSpeed);
      break;

    case SHARP_RIGHT: // 右急弯（直角）
      // 左电机正转
      digitalWrite(b1, LOW);
      digitalWrite(b2, HIGH);
      analogWrite(enB, leftSpeed);
      // 右电机反转
      digitalWrite(a1, LOW);
      digitalWrite(a2, HIGH);
      analogWrite(enA, rightSpeed);
      break;

    case STOP: // 停车
      digitalWrite(b1, LOW);
      digitalWrite(b2, LOW);
      analogWrite(enB, 0);
      digitalWrite(a1, LOW);
      digitalWrite(a2, LOW);
      analogWrite(enA, 0);
      break;
  }
}
