 clear all
 clc
THUMB   =      1
INDEX   =      2
MIDDLE  =      3
RING    =      4
LITTLE  =      5

WRIST_MOTORS  =    4
WRIST_ANGLES   =   2

FINGER_ANGLES   = 3
FINGER_MOTORS    =4
FINGERS_NUMBER   =5


PROXIMAL_LEFT     =  1
PROXIMAL_RIGHT     = 2
MEDIAL              =3
ANTAGONIST         = 4

WRIST_BACK          =1
WRIST_FORWARD       =2
WRIST_LEFT          =3
WRIST_RIGHT         =4

LATERAL_ANGLE       =1
PROXIMAL_ANGLE    =2
MEDIAL_ANGLE		  =3
ANTAGONIST_ANGLE	  =4
 
         
 a_c(1)=150;
 alpha_c(1)=-pi/2;
 d_c(1)=450; 
 teta_c(1)=0;
 
 a_c(2)=590; 
 alpha_c(2)=pi;
 d_c(2)=0;
 teta_c(2)=-pi/2;
 
 a_c(3)=130; 
 alpha_c(3)=pi/2;
 d_c(3)=0;
 teta_c(3)=pi/2;
 
 a_c(4)=0;
 alpha_c(4)=-pi/2;
 d_c(4)=647.07;
 dc4_1=660;
 teta_c(4)=0;
 
 a_c(5)=0;
 alpha_c(5)=pi/2;
 d_c(5)=0;
 teta_c(5)=0;
 
 a_c(6)=0;
 alpha_c(6)=0;
 d_c(6)=0;
 teta_c(6)=0;
 teta_d=atan(647.07/130); 
 T6HAND=[1 0 0 0;0 1 0 0; 0 0 1 357.48+95; 0 0 0 1]

 a_c
 alpha_c
 d_c
 teta_c
 
 a_w(1)=22;          % Denavit parameters Wrist
 a_w(2)=0;
 d_w(1)=0;
 d_w(2)=0;
 alpha_w(1)=pi/2;
 alpha_w(2)=0;
 
 a_w
 d_w
 alpha_w
 d_f=zeros(4,5);
 
 teta_f=zeros(4,5);
 a_f=zeros(4,5);
 alpha_f=zeros(4,5);
 d_f(THUMB,1) = 0;
 d_f(THUMB,2) = -2.18;
 d_f(THUMB,3) = 5.25;
 d_f(THUMB,4) = 0;
 
 teta_f(THUMB,1)= 0;
 teta_f(THUMB,2)= (11.01)/180*pi;
 teta_f(THUMB,3)= -4.63/180*pi;
 teta_f(THUMB,4)= 0/180*pi;
 
 a_f(THUMB,1)=18;   %Denavit parameters Fingers
 a_f(THUMB,2)=24.54;
 a_f(THUMB,3)=30;
 a_f(THUMB,4)=35; 

alpha_f(THUMB,1)=-pi/2;
alpha_f(THUMB,2)=65.41/180*pi;
alpha_f(THUMB,3)=0;
alpha_f(THUMB,4)=0;

 a_f(INDEX,1)=18;   %Denavit parameters Fingers
 a_f(INDEX,2)=37;
 a_f(INDEX,3)=25;
 a_f(INDEX,4)=27; 

alpha_f(INDEX,1)=pi/2;
alpha_f(INDEX,2)=0;
alpha_f(INDEX,3)=0;
alpha_f(INDEX,4)=0;

 a_f(MIDDLE,1)=18;   %Denavit parameters Fingers
 a_f(MIDDLE,2)=44;
 a_f(MIDDLE,3)=25;
 a_f(MIDDLE,4)=27; 

alpha_f(MIDDLE,1)=pi/2;
alpha_f(MIDDLE,2)=0;
alpha_f(MIDDLE,3)=0;
alpha_f(MIDDLE,4)=0;

 a_f(RING,1)=18;   %Denavit parameters Fingers
 a_f(RING,2)=39;
 a_f(RING,3)=25;
 a_f(RING,4)=27; 

alpha_f(RING,1)=pi/2;
alpha_f(RING,2)=0;
alpha_f(RING,3)=0;
alpha_f(RING,4)=0;


 a_f(LITTLE,1)=18;   %Denavit parameters Fingers
 a_f(LITTLE,2)=32;
 a_f(LITTLE,3)=25;
 a_f(LITTLE,4)=27; 

alpha_f(LITTLE,1)=pi/2;
alpha_f(LITTLE,2)=0;
alpha_f(LITTLE,3)=0;
alpha_f(LITTLE,4)=0;

a_f
alpha_f


Dt_fingers(THUMB).origin=[29.55 14.74 27.9];
Dt_fingers(THUMB).rotz=150;
Dt_fingers(THUMB).roty=110;
Dt_fingers(THUMB).rotz_1=190;

    

Dt_fingers(INDEX).origin=[77 -5.9 43];
Dt_fingers(INDEX).rotx=-90;
Dt_fingers(INDEX).roty=0;
Dt_fingers(INDEX).rotz=-25;


Dt_fingers(MIDDLE).origin=[84 -6.9 15];
Dt_fingers(MIDDLE).rotx=-86;
Dt_fingers(MIDDLE).roty=0;
Dt_fingers(MIDDLE).rotz=-5;

Dt_fingers(RING).origin=[80 -1.9 -11.5];
Dt_fingers(RING).rotx=-80;
Dt_fingers(RING).roty=0;
Dt_fingers(RING).rotz=5;

Dt_fingers(LITTLE).origin=[66 4.1 -35];
Dt_fingers(LITTLE).rotx=-75;
Dt_fingers(LITTLE).roty=0;
Dt_fingers(LITTLE).rotz=15;

 Dt_fingers(THUMB).T_w2_finger_base=trasx(Dt_fingers(THUMB).origin(1))*trasy(Dt_fingers(THUMB).origin(2))*trasz(Dt_fingers(THUMB).origin(3));
 Dt_fingers(THUMB).T_w2_finger_base=Dt_fingers(THUMB).T_w2_finger_base*rotz(Dt_fingers(THUMB).rotz)*roty(Dt_fingers(THUMB).roty)*rotz(Dt_fingers(THUMB).rotz_1);
 
 for(i=2:5)
    Dt_fingers(i).T_w2_finger_base=trasx(Dt_fingers(i).origin(1))*trasy(Dt_fingers(i).origin(2))*trasz(Dt_fingers(i).origin(3));
    Dt_fingers(i).T_w2_finger_base=Dt_fingers(i).T_w2_finger_base*rotx(Dt_fingers(i).rotx)*roty(Dt_fingers(i).roty)*rotz(Dt_fingers(i).rotz);
 end


