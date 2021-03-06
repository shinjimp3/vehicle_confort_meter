#define M5STACK_MPU6886
#include <M5Stack.h>
//#include "utility/MPU9250.h"
#define SCR_W 320
#define SCR_H 240

float accX, accY, accZ;
float accX_bias, accY_bias, accZ_bias;
bool is_calibrationing;
float jerkX, jerkY, jerkZ;
float disconfort_degree = 0.0;
float cutoff_pref = 4.0; //[Hz]
const int buf_length = 300; //100Hzで3秒分
//x,y,z各方向の加速度とジャークを保存するバッファ。循環バッファとして扱う
//buf_topは最新データの入ったインデックスを表す
float accX_buf[buf_length], accY_buf[buf_length], accZ_buf[buf_length]; //todo: バッファに頼らない処理にする
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
  M5.IMU.Init();
  calibration();
  xTaskCreatePinnedToCore(drawing_task, "Draw", 4096, NULL, 1, NULL, 0);

}

void loop() {
  M5.update();
  //Cボタン押下でキャリブレーション
  if(M5.BtnC.isPressed()){
    calibration();
  }
  //加速度取得および不快度計算を本loop()内において100Hz(10ms)で行う
  loop_start_time = millis();
  int loop_cycle = 10; //[ms]
  
  //加速度取得
  M5.IMU.getAccelData(&accX,&accY,&accZ); //0 ms
  accX -= accX_bias;
  accY -= accY_bias;
  accZ -= accZ_bias;
  //カットオフ周波数cutoff_pref=4.0Hzの一次フィルタ
  accX = first_orderd_filter(accX, accX_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  accY = first_orderd_filter(accY, accY_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  accZ = first_orderd_filter(accZ, accZ_buf[(buf_top+buf_length-1)%buf_length], cutoff_pref, (float)loop_cycle/1000);
  update_acc_jerk_buf(accX, accY, accZ);

  //快適度(不快度)の計算
  disconfort_degree = calc_disconfort_degree();

  unsigned long process_time = millis() - loop_start_time; //[ms]  
  if(loop_cycle-process_time >= 0){
    delay(loop_cycle-process_time); //処理時間を差し引いて，10ms(100Hz)置きにloopを動作させる
  }

}

void update_acc_jerk_buf(float accX, float accY, float accZ){
  //最新の加速度情報を受け，加速度，ジャークのバッファを更新する
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

  //debug
  Serial.print(accX_buf[buf_top]);
  Serial.print(",");
  Serial.print(accZ_buf[buf_top]);
  Serial.print(",");
  Serial.print(jerkX_buf[buf_top]);
  Serial.print(",");
  Serial.print(jerkZ_buf[buf_top]);

  //循環バッファのインデックスを一つ進める
  buf_top++;
  buf_top = buf_top%buf_length;
}

void drawing_task(void* arg){
  while(1){
    if(!is_calibrationing){
      //キャリブレーション時以外の平常時の描画を
      //本drawing_task()において10Hz(100ms)で行う
      draw_start_time = millis();
      int loop_cycle = 100;

      M5.Lcd.fillScreen(BLACK); //33ms
      //Gを表す矢印の描画
      draw_acc_arrow(accX,accY,accZ); //3ms M5.Lcd.fillTriangleが3つ
      //不快度を表す表情の描画
      draw_confort_face(confort_degree);

      unsigned long process_time = millis() - draw_start_time; //[ms]
      if(loop_cycle-process_time >= 0){
        delay(loop_cycle-process_time); //処理時間を差し引いて，100ms(10Hz)置きにwhileを動作させる
      }
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
  float ar_al = 0.6; //矢印軸の長さ
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
  //→奥行きのある形にする(画面中心を水平線，画面下1/4と画面最下端をそれぞれ45度，90度見下ろした時の地面位置とする)
  //→ピクセル座標に変換(例：中心座標[0,0]を[160, 120]に持っていく)
  float pi = 3.2415926535;
  for(int i=0; i<sizeof(arrow_ox)/sizeof(arrow_ox[0]); i++){
    //回転＆拡大縮小
    float theta = atan2(-accZ, -accX) - pi/2;
    float scale = sqrt(accZ*accZ + accX*accX) / g_std_size;
    arrow_rx[i] = scale*(cos(theta)*arrow_ox[i] - sin(theta)*arrow_oy[i]);
    arrow_ry[i] = scale*(sin(theta)*arrow_ox[i] + cos(theta)*arrow_oy[i]);
    //奥行きのある形にする
    arrow_hy[i] = 0.5*(1-atan(1-arrow_ry[i])/(pi/4)) + 0.5; //画面中央が水平線になるように
    arrow_hx[i] = arrow_rx[i]*arrow_hy[i]; //奥に行くほど横幅が小さく見えるように
    //画面サイズに合わせる
    arrow_px[i] = (int)(arrow_hx[i]*SCR_H/2 + SCR_W/2);
    arrow_py[i] = (int)(arrow_hy[i]*SCR_H/2 + SCR_H/2);
  }

  //3角形の塊で矢印を描く
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[1], arrow_py[1], arrow_px[2], arrow_py[2], RED);
  M5.Lcd.fillTriangle(arrow_px[0], arrow_py[0], arrow_px[2], arrow_py[2], arrow_px[3], arrow_py[3], RED);
  M5.Lcd.fillTriangle(arrow_px[4], arrow_py[4], arrow_px[5], arrow_py[5], arrow_px[6], arrow_py[6], RED);
}

void draw_confort_face(float confort_degree){
    // 快適度(不快度)に応じて表情を描画する
    M5.Lcd.setCursor(10,10);
    if(confort_degree<1.0){
      //happy face
      M5.Lcd.fillCircle(160,60,50,YELLOW); //face
      M5.Lcd.drawCircle(160-20,60,10,BLACK); //eye
      M5.Lcd.drawCircle(160+20,60,10,BLACK); //eye
      M5.Lcd.fillCircle(160,60+25,20,RED); //mouse
      M5.Lcd.fillRect(160-25-10,60,70,25,YELLOW); //face
      return;
    }
    if(confort_degree<2.0){
      //smile face
      M5.Lcd.fillCircle(160,60,50,YELLOW); //face
      M5.Lcd.fillRect(160-20-3,60-9,6,18,BLACK); //eye
      M5.Lcd.fillRect(160+20-3,60-9,6,18,BLACK); //eye
      M5.Lcd.drawCircle(160,60+20,10,BLACK); //mouse
      M5.Lcd.fillRect(160-10,60,22,25,YELLOW); //face
      return;
    }
    if(confort_degree<4.0){
      //pien face
      M5.Lcd.fillCircle(160,60,50,CYAN); //face
      M5.Lcd.fillCircle(160-20,60+1,15+1,WHITE); //pien eye
      M5.Lcd.fillCircle(160-20,60,15,BLACK);
      M5.Lcd.fillCircle(160-20-5,60-5,7,WHITE); //hilight
      M5.Lcd.fillCircle(160-20+3,60+3,2,WHITE); //hilight
      M5.Lcd.drawLine(160-40,60-15,160-20,60-25,BLACK); //eybllow
      
      M5.Lcd.fillCircle(160+20,60+1,15+1,WHITE); //pien eye
      M5.Lcd.fillCircle(160+20,60,15,BLACK);
      M5.Lcd.fillCircle(160+20-5,60-5,7,WHITE); //hilight
      M5.Lcd.fillCircle(160+20+3,60+3,2,WHITE); //hilight
      M5.Lcd.drawLine(160+40,60-15,160+20,60-25,BLACK); //eybllow
      
      M5.Lcd.drawLine(160-5,60+30,160,60+30-2,BLACK); //mouse
      M5.Lcd.drawLine(160+5,60+30,160,60+30-2,BLACK); //mouse
      return;
    }
    //gero face
    M5.Lcd.fillCircle(160,60,50,PURPLE); //face
    M5.Lcd.drawLine(160-30,60-6,160-10,60,BLACK); //eye
    M5.Lcd.drawLine(160-30,60+6,160-10,60,BLACK);
    M5.Lcd.drawLine(160+30,60-6,160+10,60,BLACK); //eye
    M5.Lcd.drawLine(160+30,60+6,160+10,60,BLACK);
    M5.Lcd.fillRect(160-20,60+15,40,20,RED); //mouse
    M5.Lcd.fillRect(160-20+3,60+15+10,40-6,30,GREEN); //gero
    return;
}

float calc_disconfort_degree(){
  // 快適度(不快度)を加速度，ジャークの履歴に基づいて算出する
  float jerk_sq_mean = 0.0;
  for(int i; i<buf_length; i++){
    jerk_sq_mean += (jerkX_buf[i]*jerkX_buf[i] + jerkZ_buf[i]*jerkZ_buf[i])/(float)buf_length;
  }
  Serial.print(",");
  Serial.print(jerk_sq_mean);
  //jerkの大きさの2乗平均(実効値)を不快度の指標として用いる
  float disconfort_degree = 0.05*jerk_sq_mean; //係数は要調節
  Serial.print(",");
  Serial.println(disconfort_degree);
  return disconfort_degree;
}

float calc_jerk(float* acc_buf){
  //加速度の変化を二次式で近似することによる，ノイズに強いjerk計算
  //参考：https://www.jstage.jst.go.jp/article/jje1965/36/4/36_4_191/_pdf/-char/ja
  
  //をやる前にまずは単純な微分値で実装
  //1ステップだとノイズの影響が大きいので，
  //100ms(10サンプル)(10Hz)間の変化量でjerkを計算し，平均化する
  float jerk = (acc_buf[buf_top]-acc_buf[(buf_top+buf_length-10)%buf_length])*9.8/0.1; //acc[G]→jerk[m/s^3]
  return jerk;
}

void calibration(){
  //(ユーザーにはキャリブレーション中5秒動かないようにお願いする)
  //5秒程度平均を取って加速度のバイアスを計算する
  is_calibrationing = true;
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);//文字の大きさを設定（1(最小)～7(最大)）
  M5.Lcd.setCursor(20, 20); //文字表示の左上位置を設定
  M5.Lcd.setTextColor(WHITE); //文字色を設定（文字背景色は透明）
  M5.Lcd.println("Calibration now.\n\nPlease don't move me.");
  //キャリブレーション開始までの準備時間
  delay(2000);

  accX_bias = 0;
  accY_bias = 0;
  accZ_bias = 0;

  for(int i=-2; i<3; i++){
    for(int j=0; j<100; j++){
      M5.IMU.getAccelData(&accX,&accY,&accZ); //0 ms
      accX_bias += accX;
      accY_bias += accY;
      accZ_bias += accZ;
      delay(10);
    }
    //キャリブレーションが進行していることを示すため，1秒に1回ドットを表示する
    M5.Lcd.fillCircle((int)(SCR_W/2)+i*40,(int)(SCR_H/2),10,WHITE);
  }
  //5個目のドットを見せるための時間
  delay(500);
  
  accX_bias /= 500;
  accY_bias /= 500;
  accZ_bias /= 500;
  is_calibrationing = false;  
}