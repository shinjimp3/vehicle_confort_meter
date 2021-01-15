// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886
#include <M5Stack.h>
//#include "utility/MPU9250.h"
#define SCR_W 320
#define SCR_H 240

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

//ループ一回分の処理時間を取得するためのタイマー
unsigned int process_time = 0; //[ms]
unsigned int start_time = millis();

void setup() {
 
  M5.begin();
  //  serial for debugging
  Serial.begin(115200);

  M5.IMU.Init();

}

void loop() {
  start_time = millis();
  float process_timef = (float)process_time/1000; 
  
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  
  M5.Lcd.fillScreen(BLACK);    

  //Gを表す矢印の描画
  draw_acc_arrow(accX,accY,accZ);

  //快適度(不快度)の計算
  //float confort_degree = calc_confort_degree(accX,accY,accZ);

  //不快度を表す表情の描画
  //draw_confort_face(confort_degree);
  process_time = millis() - start_time; //[ms]
  
  delay(100-process_time); //処理時間を差し引いて，20ms(50Hz)置きにloopを動作させる
  Serial.print(start_time);
  Serial.print(' ');
  Serial.println(process_time);

}

void draw_acc_arrow(float acc_x, float acc_y, float acc_z){
  //加速度の大きさと方向に応じた矢印を立体的に描画
  //(0,0)から(0,1)に伸びる矢印画像を定義
  float ar_aw = 0.3; //矢印軸の幅
  float ar_al = 0.75; //矢印軸の長さ
  float ar_tw = 0.5; //三角部の幅
  float ar_tl = 1.0-ar_al; //三角部の長さ(軸と合わせ全体で1.0になるように)
  //オリジナル矢印の形を決める，矢印の頂点座標
  float arrow_ox[7] = {-ar_aw/2, -ar_aw/2, ar_aw/2, ar_aw/2, -ar_tw/2, ar_tw/2, 0};
  float arrow_oy[7] = {0, ar_al, ar_al, 0, ar_al, ar_al, 1.0};
  float arrow_rx[7], arrow_ry[7], arrow_hx[7], arrow_hy[7];
  int arrow_px[7], arrow_py[7];

  //オリジナル矢印画像をピクセル座標に変換(画面中心から画面下に伸びるよう配置)
  for(int i=0; i<sizeof(arrow_ox)/sizeof(arrow_ox[0]); i++){
      arrow_px[i] = (int)(arrow_ox[i]*SCR_H/2 + SCR_W/2);
      arrow_py[i] = (int)(arrow_oy[i]*SCR_H/2 + SCR_H/2);
  }

  //3角形の塊で矢印を描く
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[1], arrow_py[1], arrow_px[2], arrow_py[2], RED);
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[2], arrow_py[2], arrow_px[3], arrow_py[3], RED);
  M5.Lcd.fillTriangle(arrow_px[4], arrow_py[4], arrow_px[5], arrow_py[5], arrow_px[6], arrow_py[6], RED);
}

void draw_confort_face(float confort_degree){
    // 快適度(不快度)に応じて表情を描画する
}

float calc_confort_degree(float acc_x, float acc_y, float acc_z){
    // 快適度(不快度)を加速度に基づいて算出する
    return 1.0;
}