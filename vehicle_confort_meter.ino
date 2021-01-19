// define must ahead #include <M5Stack.h>
#define M5STACK_MPU6886
#include <M5Stack.h>
//#include "utility/MPU9250.h"
#define SCR_W 320
#define SCR_H 240

float accX, accY, accZ;
float jerkX, jerkY, jerkZ;
float confort_degree = 0.0;
float cutoff_pref = 8.0; //[Hz]
const int buf_length = 300; //100Hzで3秒分
float accX_buf[buf_length], accY_buf[buf_length], accZ_buf[buf_length];
float jerkX_buf[buf_length], jerkY_buf[buf_length], jerkZ_buf[buf_length];
int buf_top = 0;

//メインループ一回分の処理時間を取得するためのタイマー
unsigned long loop_start_time = millis(); //to do 50日以上稼働してもオーバーフローの問題が起こらないようにする
//描画ループ一回分の処理時間を取得するためのタイマー
unsigned long draw_start_time = millis(); //to do 50日以上稼働してもオーバーフローの問題が起こらないようにする

void setup() {
 
  M5.begin();
  M5.Power.begin();
  Serial.begin(115200);
  xTaskCreatePinnedToCore(drawing_task, "Draw", 4096, NULL, 1, NULL, 0);
  M5.IMU.Init();

}

void loop() {
  //加速度取得および快適度(不快度)計算 100Hz(10ms)
  loop_start_time = millis();
  int loop_cycle = 10; //[ms]
  
  //加速度取得
  M5.IMU.getAccelData(&accX,&accY,&accZ); //0 ms
  //カットオフ周波数8Hzの一時フィルタ
  accX = first_orderd_filter(accX, accX_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  accY = first_orderd_filter(accY, accY_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  accZ = first_orderd_filter(accZ, accZ_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  update_acc_jerk_buf(accX, accY, accZ);

  //快適度(不快度)の計算
  confort_degree = calc_disconfort_degree(accX,accY,accZ);


  unsigned long process_time = millis() - loop_start_time; //[ms]  
  if(loop_cycle-process_time >= 0){
    delay(loop_cycle-process_time); //処理時間を差し引いて，10ms(100Hz)置きにloopを動作させる
  }

}

void update_acc_jerk_buf(float accX, float accY, float accZ){
  accX_buf[buf_top] = accX;
  accY_buf[buf_top] = accY;
  accZ_buf[buf_top] = accZ;

  //jerk取得
  jerkX = calc_jerk(accX_buf);
  jerkY = calc_jerk(accY_buf);
  jerkZ = calc_jerk(accZ_buf);

  jerkX_buf[buf_top] = jerkX;
  jerkY_buf[buf_top] = jerkY;
  jerkZ_buf[buf_top] = jerkZ;

//  for debug
  Serial.print(accX_buf[buf_top]);
  Serial.print(",");
  Serial.println(jerkX_buf[buf_top]);

  buf_top++;
  buf_top = buf_top%buf_length;
}

void drawing_task(void* arg){
  while(1){
    //描画 10Hz(100ms)
    draw_start_time = millis();
    int loop_cycle = 100;

    M5.Lcd.fillScreen(BLACK); //33ms
    //Gを表す矢印の描画
    //draw_acc_arrow(accX,accY,accZ); //3ms M5.Lcd.fillTriangleが3つ
    //Test すぐ消す jerkを表示
    draw_acc_arrow(jerkX, jerkY, jerkZ);
    //不快度を表す表情の描画
    draw_confort_face(confort_degree);

    unsigned long process_time = millis() - draw_start_time; //[ms]
    if(loop_cycle-process_time >= 0){
      delay(loop_cycle-process_time); //処理時間を差し引いて，100ms(10Hz)置きにloopを動作させる
    }
  }
}

float first_orderd_filter(float x, float pre_x, float fc, float ts){
  //後退差分で一次遅れフィルタの結果を返す
  float tau = 1/(2*3.1415926535*fc); //時定数 
  return tau/(ts+tau)*pre_x + ts/(ts+tau)*x;
}

void draw_acc_arrow(float acc_x, float acc_y, float acc_z){
  //加速度の大きさと方向に応じた矢印を立体的に描画
  //(0,0)から(0,1)に伸びる矢印画像を定義
  float ar_aw = 0.3; //矢印軸の幅
  float ar_al = 0.75; //矢印軸の長さ
  float ar_tw = 0.5; //三角部の幅
  float ar_tl = 1.0-ar_al; //三角部の長さ(軸と合わせ全体で1.0になるように)
  float g_std_size = 0.1; //[G]
  //オリジナル矢印の形を決める，矢印の頂点座標
  float arrow_ox[7] = {-ar_aw/2, -ar_aw/2, ar_aw/2, ar_aw/2, -ar_tw/2, ar_tw/2, 0};
  float arrow_oy[7] = {0, ar_al, ar_al, 0, ar_al, ar_al, 1.0};
  float arrow_rx[7], arrow_ry[7], arrow_hx[7], arrow_hy[7];
  int arrow_px[7], arrow_py[7];

  //オリジナル矢印画像(→x,↓y, 矢印の根本が(0,0)，先端が(0,1))
  //→加速度に応じて回転＆拡大縮小(→x方向に-accX，↓y方向に-accZ)
  //→ホモグラフィ変換で，奥行きのある台形にする()
  //→ピクセル座標に変換(例：中心座標[0,0]を[160, 120]に持っていく)
  for(int i=0; i<sizeof(arrow_ox)/sizeof(arrow_ox[0]); i++){
    //回転＆拡大縮小
      float theta = atan2(-accZ, -accX) - 3.1415926535/2;
      float scale = sqrt(accZ*accZ + accX*accX) / g_std_size;
      arrow_rx[i] = scale*(cos(theta)*arrow_ox[i] - sin(theta)*arrow_oy[i]);
      arrow_ry[i] = scale*(sin(theta)*arrow_ox[i] + cos(theta)*arrow_oy[i]);
      arrow_px[i] = (int)(arrow_rx[i]*SCR_H/2 + SCR_W/2);
      arrow_py[i] = (int)(arrow_ry[i]*SCR_H/2 + SCR_H/2);
  }

  //3角形の塊で矢印を描く
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[1], arrow_py[1], arrow_px[2], arrow_py[2], RED);
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[2], arrow_py[2], arrow_px[3], arrow_py[3], RED);
  M5.Lcd.fillTriangle(arrow_px[4], arrow_py[4], arrow_px[5], arrow_py[5], arrow_px[6], arrow_py[6], RED);
}

void draw_confort_face(float confort_degree){
    // 快適度(不快度)に応じて表情を描画する
}

float calc_disconfort_degree(float acc_x, float acc_y, float acc_z){
  // 快適度(不快度)を加速度に基づいて算出する
    return 1.0;
}

float calc_jerk(float* acc_buf){
  //加速度の変化を二次式で近似することによる，ノイズに強いjerk計算
  //参考：https://www.jstage.jst.go.jp/article/jje1965/36/4/36_4_191/_pdf/-char/ja
  
  //BLA::Matrix<3,3> A;
  //BLA::Matrix<3,1> b;

  //をやる前にまずは単純な微分値で実装
  //1ステップだとノイズの影響が大きいので，
  //100ms(10サンプル)(10Hz)間の変化量でjerkを計算し，平均化する
  float jerk = (acc_buf[buf_top]-acc_buf[(buf_top+buf_length-10)%buf_length]); //[delta G]
  return jerk;
}