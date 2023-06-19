// function [xr,yr,tr,xrd,yrd,trd] = fcn(t)

int flag = 0;
if (flag == 0){
    unsigned long prevTime = 0;
    flag = 1;
} 

unsigned long t = millis() - prevTime;

int T = 10;
int r = 1;
float f = 1/T;

float tr = 2*pi*f*t;
float trd = 2*pi*f;

float xr = r*sin(tr);
float yr = -r*cos(tr);

float xrd = r*trd*cos(tr);
float yrd = r*trd*sin(tr);

// function [v,w]  = fcn(xr,yr,thetar,xrd,yrd,thetard, x, y, theta)

int k1 = 40;    
int k2 = 20;
float d = 0.01; //Distancia entre ruedas

float V1aux = xrd + k1*(xr - x);
float V2aux = yrd + k2*(yr - y);

float v = V1aux*cos(thetar) + V2aux*sin(thetar);
float w = -V1aux*sin(thetar)/d + V2aux*cos(thetar)/d;

// function [wd, wi] = fcn(v, w)

// 1/r(1 | L
// 1 | -L)

int r = 1;
float L = 1.5;
// float T[2][2] = {{r/2, r/2}, {r/(2*L), r/(2*L)}} //r/2*[1 1; 1/L -1/L];
float Tinv[2][2] = {{1/r, 1/(r*L)}, {1/r, -1/(r*L)}};

// qpp = T\[v;w];
// wd = qpp(1);
// wi = qpp(2);

wd = Tinv[0][0]*v + Tinv[0][1]*w;
wi = Tinv[1][0]*v + Tinv[1][1]*w;

// function salida = fcn(wd, wi,theta)

float xp = (r/2)*(wd+wi)*cos(theta);
float yp = (r/2)*(wd+wi)*sin(theta);
float thetad = (r/2*L)*(wd-wi);
//salida = [xp, yp, thetad];

float xSum = xp 