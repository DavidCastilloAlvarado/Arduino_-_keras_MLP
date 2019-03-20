#include <MatrixMath.h>
#include <Math.h>

#define N  (1)
#define M  (4)
#define p  (3)
float ID=0;
float *pID = &ID;


mtx_type A[N][M]= {6.2288379e-04,2,3,4};
mtx_type B[M][p] = {{1,2,8},{5,8,2},{7,8,9},{1,5,7}};
mtx_type C[N][p];
mtx_type v[N][p]= {1,2,3};      // This is a row vector
mtx_type w[M][N]= {7,2,8,4};

mtx_type maxVal = 10;  // maxValimum random matrix entry range

void setup()
{
	Serial.begin(9600);
	// Initialize matrices
  
  
  /**
  // Initialize matrices
  for (int p = 0; p < N; p++)
  {
    for (int q = 0; q < M; q++)
    {
      B[p][q] = random(maxVal) - maxVal / 2.0f; // A is random
      
    }
  }

**/
}
void loop()
{
  int start = millis();
  ID = 777;
  int data = A[0][0]*100000;
  Serial.println(data);
  Serial.println(sigmoid(1)*100);
  for(int i=0; i<p;i++){
    v[0][i] = tanh(v[0][i]);
  }

  
	Matrix.Multiply((mtx_type*)A, (mtx_type*)B, N, M, p, (mtx_type*)C);
  
  int end_ = millis();
  int dt = end_ - start;
	Serial.print("\nAfter multiplying C = A*B: Tiempo de ejecuciOn: "); Serial.println(dt);
	Matrix.Print((mtx_type*)A, N, M, "A");
ID = 1111;
Serial.println(16**pID);
	Matrix.Print((mtx_type*)B, M, p, "B");
	Matrix.Print((mtx_type*)C, N, p, "C");
	Matrix.Print((mtx_type*)v, N, p, "v");
  Matrix.Print((mtx_type*)w, M, N, "w");

	Matrix.Add((mtx_type*) B, (mtx_type*) C, N, M, (mtx_type*) C);
	Serial.println("\nC = B+C (addition in-place)");
	Matrix.Print((mtx_type*)C, N, M, "C");
	Matrix.Print((mtx_type*)B, N, M, "B");

	Matrix.Copy((mtx_type*)A, N, N, (mtx_type*)B);
	Serial.println("\nCopied A to B:");
	Matrix.Print((mtx_type*)B, N, M, "B");

	Matrix.Invert((mtx_type*)A, N);
	Serial.println("\nInverted A:");
	Matrix.Print((mtx_type*)A, N, N, "A");

	Matrix.Multiply((mtx_type*)A, (mtx_type*)B, N, N, M, (mtx_type*)C);
	Serial.println("\nC = A*B");
	Matrix.Print((mtx_type*)C, N, M, "C");

	// Because the library uses pointers and DIY indexing,
	// a 1D vector can be smoothly handled as either a row or col vector
	// depending on the dimensions we specify when calling a function
	Matrix.Multiply((mtx_type*)C, (mtx_type*)v, N, M, 1, (mtx_type*)w);
	Serial.println("\n C*v = w:");
	Matrix.Print((mtx_type*)v, N, 1, "v");
	Matrix.Print((mtx_type*)w, N, 1, "w");

}

float sigmoid(float x){
  return 1/(exp(-x)+1);
}
