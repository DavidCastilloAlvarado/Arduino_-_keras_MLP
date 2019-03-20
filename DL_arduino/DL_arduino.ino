#include <MatrixMath.h>
#include <Math.h>
#include "Wire.h"
 float val;
/*--------------------------MPU 6050-----------------------------*/
// La direcciÃ³n del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estarÃ¡ implicito
//MPU6050 sensor_ang;
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
#define SDA 20
#define SCL 21
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t Tmp;
double ax_d, ay_d, az_d;
double  ang_x, ang_y;
float ang_x_prev, ang_y_prev;
long tiempo_prev,tiempo_prev1,tiempo_prev2;
float dt;
int d_max;
float accel_ang_y,accel_ang_x; 
//time out loop count
const int timeout = 200;
void init_MPU_sensor();
void MPU_sensor();
void MPU_read();
/*------------------------Definiendo pesos para el MLP*/
#define input_N  (1)
#define input_M  (6) 

#define L0_N  (6)
#define L0_M  (10) // same for Bias
#define L1_N  (10)
#define L1_M  (8)  // same for Bias
#define L2_N  (8)
#define L2_M  (8)  // same for Bias
#define L3_N  (8)
#define L3_M  (6)  // same for Bias
#define L4_N  (6)
#define L4_M  (1)  // same for Bias
#define unid_  (1)  // same for Bias
int16_t ax=0;
int16_t ay=0;
int16_t az=0;
int16_t gx=0;
int16_t gy=0;
int16_t gz=0;

mtx_type input[input_N][input_M] = {0.00,0.00,0.00,0.00,0.00,0.00};

mtx_type L0[L0_N][L0_M]= {{ 2.0549965  , 0.73152137 , 0.07078217, -0.7290443 , -2.2930493 , -0.7631958,
						  -0.8233199  , 1.5917686 , -1.7656589,  -0.14494936},
						 {-1.3579916 , -0.9757451 , -1.2048781 ,  0.5304767 , -0.9327282  , 1.6764361,
						   0.172624  ,  0.5577521,  -0.90876377 ,-0.42663667},
						 { 0.869779  , -0.00427562 ,-0.18320262 , 0.6315745  , 0.33462766, -0.0963622,
						  -0.734544 ,  -0.58336145, -0.04939673,  0.78223765},
						 {-0.3513457 ,  0.66198003, 0.09647669, -0.01301495 ,-0.07210436 , 0.31300253,
						  -0.74321854 , 0.08663455 ,-0.20723936 , 0.2497143 },
						 { 0.6262538 , -0.5896861,  -0.16825275, -0.10362005 , 0.3386229 , -0.85132253,
						   0.11431435  ,0.67278266 , 0.20646432 , 0.7237033 },
						 { 0.39709014, -0.23298283 , 0.13151188 , 0.25553346, -0.22546269 ,-0.32140878,
						  -0.26277933 ,0.17771171 ,-0.65898424 , 0.4265563 }};

mtx_type B0B[unid_][L0_M]     = { 0.10846587,  0.22224176 , 0.02659535,  0.05782716, -0.00642552,  0.13359843,
  								0.07542766, -0.05395553, 0.14789149, -0.04249101} ;

mtx_type L1[L1_N][L1_M] = {{ 1.982958  ,  0.49852887  ,0.0108047 ,  0.07161843,  0.04735603  ,1.1888287,
							  -2.919036 ,   0.67658335},
							 { 2.3032928 ,  0.48203713 , 0.2570429  , 0.17908505 , 1.2567558 ,  0.49357593,
							  -1.8713325  , 0.6098087 },
							 { 0.37399566 , 0.55240667 ,-1.109156  , -1.3742007 , -0.10651035 ,-1.1341479,
							  -0.36480933 , 0.9192023 },
							 {-0.34679735 , 0.5442364  , 0.03793924  ,0.6785402 ,  1.96818   ,  2.4196203,
							  -0.0991582 , -0.17028022},
							 {-0.76667196 ,-0.202326  , -0.93059224 ,-1.1529652  , 2.4208324 ,  1.5249335,
							  -0.7466261  , 0.49941906},
							 {-0.41255048 ,-0.54698646 , 1.5698832 ,  1.1906053 ,  0.675963  ,  0.5467384,
							   1.8784422 , -0.97658926},
							 {-0.7600907 , -0.18641368  ,1.0323216 ,  0.8869304 ,  2.520573   , 0.18448111,
							   0.1333522 , -0.78364104},
							 {-0.3237455  , 0.11525488 , 1.1475712  , 1.0254271  ,-2.3948913  ,-1.8994044,
							   0.54964155, -0.04168001},
							 { 0.09244436 , 0.32602373 ,-0.26574048 ,-0.5316153 ,  3.4383354 ,  1.5852231,
							  -0.8917138 , -0.1493364 },
							 { 0.89777595 , 0.43767595, -0.26948428 ,-1.2447916,   0.18189853 , 1.7059469,
							  -1.9572074 ,  0.57123595}}  ;

mtx_type B1B[unid_][L1_M]     = { 0.06483117 , 0.82005876 ,-0.1322395  ,0.12871543 , 0.10346701  ,0.05259631,
								  0.14760274 , 0.66271096} ;

mtx_type L2[L2_N][L2_M] = {{ 3.1194048e+00 ,-1.7216620e+00 , 3.2598519e-01 , 1.2772725e+00,
							   1.0850912e+00 ,-1.8953481e-01 , 2.2084205e+00 ,-8.0055594e-01},
							 {-4.0105242e-02, -5.2174300e-01, -7.7560437e-01 ,-1.1537112e+00,
							  -1.4206325e+00 , 8.9169896e-01 ,-2.9544058e-01 , 9.7361678e-01},
							 {-3.8614254e-02 ,-6.4679003e-01 , 2.7312081e+00 , 8.8321197e-01,
							   2.1270862e+00, -1.9658288e+00,  1.0843894e-01, -1.6189179e+00},
							 { 2.1031250e-01 ,-1.9949751e-01 , 2.8239281e+00 , 2.9935229e-01,
							   1.3435221e+00 ,-2.5104961e+00 , 5.1552451e-01 ,-2.1122296e+00},
							 { 1.6300604e+00, -1.0446337e+00 , 1.6332840e+00  ,1.4484887e+00,
							   1.7282267e+00 ,-1.9154168e+00 , 1.6936052e+00 ,-1.6453424e+00},
							 { 1.6800208e-01, -1.1920677e+00 , 5.1681423e-01 , 1.8418440e+00,
							   1.1689370e+00 ,-1.4984118e+00 , 7.9344720e-01 ,-1.0724494e+00},
							 {-2.4092464e+00 , 2.9763947e+00 ,-1.4353437e+00 ,-2.2452445e+00,
							  -1.1908485e+00 , 8.4151345e-01 ,-1.9663683e+00,  1.0246364e+00},
							 { 1.5908353e-03 ,-1.7836504e-01, -9.3754715e-01, -7.1875101e-01,
							  -1.8678796e+00 , 1.9134377e+00 ,-6.5286875e-01 , 1.5621040e+00}}  ;

mtx_type B2B[unid_][L2_M]  = {-0.10297762 ,0.51434416 ,-0.33080998 ,-1.1566651 , -0.53437984 , 0.38784763,
 							-1.1676469 ,  0.6698718 } ;

mtx_type L3[L3_N][L3_M] = {{-1.0684602  ,-1.0289813,   0.38778698 , 1.3591895,  -1.268505  , -0.61100537},
							 { 3.138089  ,  2.7531319 , -3.2757506  ,-2.9995828  , 3.6078892  , 3.081388  },
							 {-1.0625443 , -0.31779072 , 0.62106335  ,0.4297438 , -0.9112662  ,-1.020865  },
							 {-1.1154268  ,-1.1851511 ,  0.9426039  , 0.6058121 , -1.015511  , -1.4914316 },
							 {-0.13504146 ,-0.81230706 , 0.75212055 , 0.31440598 ,-1.1532211 , -1.1172838 },
							 { 2.251216  ,  3.0667264 , -2.1147158 , -3.2220414 ,  2.4968786 ,  2.50907   },
							 {-0.8644752 , -1.030313  ,  1.4756747 ,  1.4104692 , -0.3922202 , -0.57539076},
							 { 3.0929515 ,  2.3098907  ,-2.6028907 , -2.2840881 ,  2.3587422  , 2.8065836 }} ;

mtx_type B3B[unid_][L3_M]       = { 0.0791825   ,0.23778811, -0.04999492, -0.02931433  ,0.48093852  ,0.4257805 }  ;

mtx_type L4[L4_N][L4_M] = { 2.7341938,
							  2.816239 ,
							 -2.0193458,
							 -1.9935538,
							  3.0644875,
							  3.5271382}  ; 

mtx_type B4B[unid_][L4_M] = {-1.2208393};

mtx_type out0[input_N][L0_M]; //1x10
mtx_type out1[input_N][L1_M]; //1x8
mtx_type out2[input_N][L2_M]; //1x8
mtx_type out3[input_N][L3_M]; //1x6
mtx_type out4[input_N][L4_M]; //1x1

mtx_type out00[input_N][L0_M]; //1x10
mtx_type out11[input_N][L1_M]; //1x8
mtx_type out22[input_N][L2_M]; //1x8
mtx_type out33[input_N][L3_M]; //1x6
mtx_type out44[input_N][L4_M]; //1x1

float sigmoid(float x);
float Neural_Network_MLP();


void setup()
{
	Serial.begin(9600);
	  //------MPU------
	  init_MPU_sensor();
	  //------Timer----
	  //Timer1.initialize(20000);         // Dispara cada 250 ms
	  //Timer1.attachInterrupt(MPU_read); // Activa la interrupcion y la asocia a ISR_Blink
    pinMode(13,OUTPUT);
    
}

void loop(){
  int stt = millis();
	MPU_sensor();
 
  val = Neural_Network_MLP();
  /*
  Matrix.Print((mtx_type*)input, 1, 6, "input");
  Matrix.Print((mtx_type*)out00, 1, 10, "out00");
  double aa = 7.123456789101112/100000;//L3[7][3]*0.115; //6.2288379e-04
  //Matrix.Print((mtx_type*)B4B, 1, 1, "B4");
  Serial.println(aa,20);
  */
  int dt = millis()-stt;
  Serial.print(val);Serial.print(F("  ")); 
  //Serial.print(dt);Serial.print("  ");
  if( val > 0.98){
      Serial.println("ColisiÃ³n");
      digitalWrite(13,1);
  
      }else{
        Serial.println("Normal");
        digitalWrite(13,0);
      }

}

float sigmoid(float x){

	return 1.0/(exp(-x)+1.0);
}

float Neural_Network_MLP(){
  
	// First layer
	Matrix.Multiply((mtx_type*)input, (mtx_type*)L0, input_N, input_M, L0_M, (mtx_type*)out0);
	Matrix.Add((mtx_type*)out0, (mtx_type*)B0B,input_N , L0_M, (mtx_type*) out00);
	// Activation
	for(int i=0; i<L0_M;i++){
		out00[0][i] = tanh(out00[0][i]);
	}
	// Second Layer
	Matrix.Multiply((mtx_type*)out00, (mtx_type*)L1, input_N, L0_M, L1_M, (mtx_type*)out1);
	Matrix.Add((mtx_type*)out1, (mtx_type*)B1B,input_N , L1_M, (mtx_type*) out11);
	// Activation
	for(int i=0; i<L1_M;i++){
		out11[0][i] = tanh(out11[0][i]);
	}
	//Third Layer
	Matrix.Multiply((mtx_type*)out11, (mtx_type*)L2, input_N, L1_M, L2_M, (mtx_type*)out2);
	Matrix.Add((mtx_type*)out2, (mtx_type*)B2B,input_N , L2_M, (mtx_type*) out22);
	// Activation
	for(int i=0; i<L2_M;i++){
		out22[0][i] = sigmoid(out22[0][i]);
	}
	//Fourd Layer
	Matrix.Multiply((mtx_type*)out22, (mtx_type*)L3, input_N, L2_M, L3_M, (mtx_type*)out3);
	Matrix.Add((mtx_type*)out3, (mtx_type*)B3B,input_N , L3_M, (mtx_type*) out33);
	// Activation
	for(int i=0; i<L3_M;i++){
		out33[0][i] = sigmoid(out33[0][i]);
	}
	// Fifth Layer
	Matrix.Multiply((mtx_type*)out33, (mtx_type*)L4, input_N, L3_M, L4_M, (mtx_type*)out4);
	Matrix.Add((mtx_type*)out4, (mtx_type*)B4B,input_N , L4_M, (mtx_type*) out44);
	// Activation
	for(int i=0; i<L4_M;i++){
		out44[0][i] = sigmoid(out44[0][i]);
	}

	return out44[0][0];
	/*--------------------------Confidencia para determinar si hay colición------------------
	if( out44[0][0] > 0.6){
		return 1.0;

		}else{
			return 0.0;
		}*/
}

/*====================================================================*/
void init_MPU_sensor(){
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B); // PWR_MGMT_1 register
	Wire.write(0); // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	// Cambiar a sensibilidad de -+8g
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1C);
	Wire.write(B00011000);   //here is the byte for sensitivity (8g here) check datasheet for the one u want
	Wire.endTransmission(true);
	// Cambiar a sensibilidad de -+2000degr
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1B);
	Wire.write(B00011000);   //here is the byte for sensitivity (2000degr here) check datasheet for the one u want
	Wire.endTransmission(true);
}
/*====================================================================*/
void MPU_sensor(){
  /* Leer las aceleraciones y velocidades angulares
  sensor_ang.getAcceleration(&ax, &ay, &az);
  sensor_ang.getRotation(&gx, &gy, &gz);
  // Correxion de Offsets*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  ax=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ax=ax+53;
  ay=ay+48;
  az=az+219;
  gx=gx+77;
  gy=gy-39;
  gz=gz-24;
  
  input[0][0] = ax/32767.0;
  input[0][1] = ay/32767.0;
  input[0][2] = az/32767.0;
  input[0][3] = gx/32767.0;
  input[0][4] = gy/32767.0;
  input[0][5] = gz/32767.0;

}
