/*
 * Hexapod Robot
Author: Jon Dyson, based upon:
Original Project Page: http://blog.oscarliang.net/arduino-hexapod-robot/ , by Oscar Liang.
Last Modified: 23 Feb 2014

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details: http://www.gnu.org/licenses/gpl.html

*/


// This is the main program


// 23/02/2014

#include "fmath.h"
#include "coor.h"
#include <Servo.h>

// Define Leg Numbers
#define RF 0
#define RM 1
#define RR 2
#define LR 3
#define LM 4
#define LF 5

// Bluetooth Command
byte packet[4];
int pIndex;


Servo legServo[18];
// servo adjust offset in micro seconds
int servoOffset[18] = {-40,-80,-70,
						-30,50,50,
						-85,-25,0,
						-30,-10,80,
						0,-20,60,
						-20,-5,-5};
// servo constrains
int servoMin[18] = {800, 780,1200};
int servoMax[18] = {2200,1800,2000};

//[ANGLES]
float Angle_Old[18] = {0.0};


// ---------- BODY DIMENSIONS ----------

const int CoxaLength  = 29;          //Length of the Coxa [mm]
const int FemurLength = 57;          //Length of the Femur [mm]
const int TibiaLength = 141;          //Lenght of the Tibia [mm]
const int HexSideLength = 137;        // distance from one coxa to the next

const int FLxFL = FemurLength*FemurLength;
const int TLxTL = TibiaLength*TibiaLength;
const int FLxTL = FemurLength*TibiaLength;

const int AngleOffset[6] = {30, 90, 150, 210, 270, 330};

coor2 BodyCoxaOffset;
coor2 BCOffset[6];		//Distance from centre of the body to the coxa's
coor CurPos[6];		//Start positions of the legs

// -------------- Gait --------------

int GaitStep = 1;
int LegLiftHeight = 50;

int TLDivFactor=0; //Number of steps that a leg is on the floor while walking
int NrLiftedPos=0; //Number of positions that a single leg is lifted (1-3)
int HalfLiftHeigth = 0;  //If TRUE the outer positions of the ligted legs will be half height

int StepsInGait=0;  //Number of steps in gait
int GaitLegNr[6]; //Init position of the leg

coor GaitPos[6];

// -------------- Balance -------------

coor TotalTrans;
coor TotalBal;

//--------------------------------------------------------------------
//[VARIABLES]

// inputs
coor BodyPos; //Global Input for the position of the body
coor BodyRot; //Global Input pitch of the body

coor WalkLength; // Global Input for the walking

int speed = 2; // get from input
int mode = 0;

//=================================================================
//=================================================================

/*float constrain(float x, float a, float b)
{
	    if(x < a) {
	        return a;
	    }
	    else if(b < x) {
	        return b;
	    }
	    else
	        return x;
}*/
void GaitSelect (int GaitType){
//Gait selector

	//Ripple Gait 18 steps
	GaitLegNr[LR] = 1;
	GaitLegNr[RF] = 4;
	GaitLegNr[LM] = 7;
	GaitLegNr[RR] = 10;
	GaitLegNr[LF] = 13;
	GaitLegNr[RM] = 16;

	NrLiftedPos = 3;
	TLDivFactor = 12;
	StepsInGait = 18;
}

coor GaitCalculate (coor WalkLength){
	//Calculate Gait positions

	for (int LegIndex = 0; LegIndex < 6; LegIndex++){

		bool walking = ((abs(WalkLength.X)>10) || (abs(WalkLength.Z)>10) || (abs(WalkLength.RotY)>10));
		bool homePosition = (abs(GaitPos[LegIndex].X)<10) && (abs(GaitPos[LegIndex].Z)<10) && (abs(GaitPos[LegIndex].RotY)<10);


		if (walking && GaitStep == GaitLegNr[LegIndex]) {

			GaitPos[LegIndex].X = -WalkLength.X/2 * FSIN_TABLE[80];
			GaitPos[LegIndex].Z = -WalkLength.Z/2 * FSIN_TABLE[80];
			GaitPos[LegIndex].Y = -LegLiftHeight*FSIN_TABLE[20];
			GaitPos[LegIndex].RotY = -WalkLength.RotY/2 * FSIN_TABLE[80];

		}
		else if (walking && GaitStep == GaitLegNr[LegIndex]+1) {

			GaitPos[LegIndex].X = -WalkLength.X/2 * FSIN_TABLE[30];
			GaitPos[LegIndex].Z = -WalkLength.Z/2 * FSIN_TABLE[30];
			GaitPos[LegIndex].Y = -LegLiftHeight*FSIN_TABLE[70];
			GaitPos[LegIndex].RotY = -WalkLength.RotY/2 * FSIN_TABLE[30];

		}
		else if ((walking && GaitStep == GaitLegNr[LegIndex]+2) || (walking==false && homePosition==false && GaitStep == GaitLegNr[LegIndex])) {

			GaitPos[LegIndex].X = -WalkLength.X/2;
			GaitPos[LegIndex].Z = -WalkLength.Z/2;
			GaitPos[LegIndex].Y = -LegLiftHeight;
			GaitPos[LegIndex].RotY = -WalkLength.RotY/2;

		}
		else if (walking && ((GaitStep == GaitLegNr[LegIndex]+3) || (GaitStep == GaitLegNr[LegIndex]+3-18))) {

			GaitPos[LegIndex].X = WalkLength.X/2 * FSIN_TABLE[30];
			GaitPos[LegIndex].Z = WalkLength.Z/2 * FSIN_TABLE[30];
			GaitPos[LegIndex].Y = -LegLiftHeight*FSIN_TABLE[70];
			GaitPos[LegIndex].RotY = WalkLength.RotY/2 * FSIN_TABLE[30];

		}
		else if (walking && ((GaitStep == GaitLegNr[LegIndex]+4) || (GaitStep == GaitLegNr[LegIndex]+4-18))) {

			GaitPos[LegIndex].X = WalkLength.X/2 * FSIN_TABLE[80];
			GaitPos[LegIndex].Z = WalkLength.Z/2 * FSIN_TABLE[80];
			GaitPos[LegIndex].Y = -LegLiftHeight*FSIN_TABLE[20];
			GaitPos[LegIndex].RotY = WalkLength.RotY/2 * FSIN_TABLE[80];

		}
		else if (walking && ((GaitStep == GaitLegNr[LegIndex]+5) || (GaitStep == GaitLegNr[LegIndex]+5-18))) {

			GaitPos[LegIndex].X = WalkLength.X/2;
			GaitPos[LegIndex].Z = WalkLength.Z/2;
			GaitPos[LegIndex].Y = 0;
			GaitPos[LegIndex].RotY = WalkLength.RotY/2;

		}
		else {

			GaitPos[LegIndex].X = GaitPos[LegIndex].X - (WalkLength.X/TLDivFactor);
			GaitPos[LegIndex].Z = GaitPos[LegIndex].Z - (WalkLength.Z/TLDivFactor);
			GaitPos[LegIndex].Y = 0;
			GaitPos[LegIndex].RotY = GaitPos[LegIndex].RotY - (WalkLength.RotY/TLDivFactor);

		}



	}

	//Advance to the next step
	if (++GaitStep > StepsInGait)
		GaitStep = 1;

}


void InverseKinematics (coor Rotation, coor RB_FeetPos, coor2 coxaOffset, int angleOffset, float *OUT_IKCoxaAngle, float *OUT_IKFemurAngle, float *OUT_IKTibiaAngle){
	// RB - related to body
	// RC - related to coxa

	// -------------------- Body IK ---------------------------

	coor RB_Total;

	RB_Total.X = RB_FeetPos.X + coxaOffset.X;
	RB_Total.Z = RB_FeetPos.Z + coxaOffset.Z;
	RB_Total.Y = RB_FeetPos.Y;

	// Calculate cosine and sine of rotation angles
	// T = theta, A = alpha, B = beta
	float sinT = fsin(Rotation.X);  // 0
	float cosT = fcos(Rotation.X);  // 1
	float sinA = fsin(Rotation.Z);  // 0
	float cosA = fcos(Rotation.Z);  // 1
	float sinB = fsin(Rotation.Y);  // 0
	float cosB = fcos(Rotation.Y);  // 1


	//Calculate position corrections of feet using Rotation Matrix
	coor RB_BodyIKPos;

	RB_BodyIKPos.X = (RB_Total.X*cosA*cosB - RB_Total.Z*cosA*sinB + RB_Total.Y*sinA) - RB_Total.X;
	RB_BodyIKPos.Z = (RB_Total.X*(sinT*sinA*cosB+cosT*sinB) + RB_Total.Z*(sinT*sinA*sinB+cosT*cosB) - RB_Total.Y*sinT*cosA) - RB_Total.Z;
	RB_BodyIKPos.Y = (RB_Total.X*(sinT*sinB-cosT*sinA*cosB) + RB_Total.Z*(cosT*sinA*sinB+sinT*cosB) + RB_Total.Y*cosT*cosA) - RB_Total.Y;


	// -------------------- Leg IK --------------------------

	// the updated feet position
	RB_FeetPos.X = RB_FeetPos.X + RB_BodyIKPos.X;
	RB_FeetPos.Z = RB_FeetPos.Z + RB_BodyIKPos.Z;
	RB_FeetPos.Y = RB_FeetPos.Y + RB_BodyIKPos.Y;

	// moving from global frame to local frame
	coor RC_FeetPos;

	RC_FeetPos.X = RB_FeetPos.X*fcos(angleOffset) - RB_FeetPos.Z*fsin(angleOffset);
    RC_FeetPos.Z = RB_FeetPos.X*fsin(angleOffset) + RB_FeetPos.Z*fcos(angleOffset);
	RC_FeetPos.Y = RB_FeetPos.Y;

	//Length between the Coxa and Feet
	float CoxaFeetDist = sqrt(RC_FeetPos.X*RC_FeetPos.X + RC_FeetPos.Z*RC_FeetPos.Z);

	//IKSW - Length between shoulder and wrist
	float IKSW = sqrt((CoxaFeetDist-CoxaLength)*(CoxaFeetDist-CoxaLength) + RC_FeetPos.Y*RC_FeetPos.Y);

	//IKA1 - Angle between SW line and the ground in radians
	float IKA1 = fatan2(CoxaFeetDist-CoxaLength, RC_FeetPos.Y);

	//IKA2 - Angle between SW line and femur
	float IKA2 = facos((FLxFL + IKSW*IKSW - TLxTL) / (2.0*FemurLength*IKSW));

	//IKFemurAngle
	*OUT_IKFemurAngle = (IKA1+IKA2)-PIby2;

	//IKTibiaAngle
	float TAngle = facos((TLxTL + FLxFL -IKSW*IKSW) / (2.0*FLxTL));

	// convert Tibia angle with relation to the normal of Femur axis
	*OUT_IKTibiaAngle = PIby2 - TAngle ;


	//IKCoxaAngle
	*OUT_IKCoxaAngle = fatan2(RC_FeetPos.X,RC_FeetPos.Z);

}


void BodyBalance(){

	// ============= Leg Balance Calculation ===========

	coor Total;

	for (int LegIndex = 0; LegIndex < 6; LegIndex++){

		//Calculating totals from centre of the body to the feet
		Total.Z = BCOffset[LegIndex].Z + CurPos[LegIndex].Z + BodyPos.Z + GaitPos[LegIndex].Z;
		Total.X = BCOffset[LegIndex].X + CurPos[LegIndex].X + BodyPos.X + GaitPos[LegIndex].X;
		Total.Y = GaitPos[LegIndex].Y; // using the value 150 to lower the centre point of rotation 'BodyPosY +

		TotalTrans.Y = TotalTrans.Y + Total.Y;
		TotalTrans.Z = TotalTrans.Z + Total.Z;
		TotalTrans.X = TotalTrans.X + Total.X;

		TotalBal.Y = TotalBal.Y + RadToDeg(atan(Total.X/Total.Z));
		TotalBal.Z = TotalBal.Z + RadToDeg(atan(Total.X/Total.Y));
		TotalBal.X = TotalBal.X + RadToDeg(atan(Total.Z/Total.Y));

	}

  // ============= Body Balance Calculation ===========

  	TotalTrans.Z = TotalTrans.Z/6;
	TotalTrans.X = TotalTrans.X/6;
	TotalTrans.Y = TotalTrans.Y/6;

	if (TotalBal.Y < -180) 	//Tangents fix caused by +/- 180 deg
		TotalBal.Y = TotalBal.Y + 360;
	if (TotalBal.Z < -180 )	//Tangents fix caused by +/- 180 deg
		TotalBal.Z = TotalBal.Z + 360;
	if (TotalBal.X < -180 )	//Tangents fix caused by +/- 180 deg
		TotalBal.X = TotalBal.X + 360;

	//Balance rotation
	TotalBal.Y = TotalBal.Y/6;
	TotalBal.X = TotalBal.X/6;
	TotalBal.Z = -TotalBal.Z/6;

	//Balance translation

	for (int LegIndex = 0; LegIndex < 6; LegIndex++){
		GaitPos[LegIndex].Z = GaitPos[LegIndex].Z - TotalTrans.Z;
		GaitPos[LegIndex].X = GaitPos[LegIndex].X - TotalTrans.X;
		GaitPos[LegIndex].Y = GaitPos[LegIndex].Y - TotalTrans.Y;
	}

}

void GetCommandValues(){

// Command Packet Format (4 byte): [type] [X] [Y] [extra]
// for detail check my website: http://blog.oscarliang.net/arduino-hexapod-robot/

	byte jX = 0;
	byte jZ = 0;
	byte jY = 0;

	mode = packet[0];

	switch(packet[0]){
		case 121:	// Body Rotation (Accelerometer X & Z)

			jX = packet[1];
			jZ = packet[2];
			jY = packet[3];
			BodyRot.X = map(jX, 0, 255, -13, 13);
			BodyRot.Z = map(jZ, 0, 255, -13, 13);
			BodyRot.Y = map(jY, 0, 255, -13, 13);
			break;

		case 122: // Walking (JOYSTICK)

			jX = packet[1];
			jZ = packet[2];
			WalkLength.X = map(jX, 0, 255, -70, 70); // normally 80
			WalkLength.Z = map(jZ, 0, 255, -70, 70);
			speed = packet[3];

			break;

		case 123: // Body Translation (Joystick)

			jX = packet[1];
			jZ = packet[2];
			BodyPos.X = map(jX, 0, 255, -30, 30);
			BodyPos.Z = map(jZ, 0, 255, -30, 30);
			break;

		case 124: // Adjust Body Height (Button)

			if (packet[1] > 178){
				WalkLength.RotY = map(packet[1], 0, 255, -15, 15);
			}
			else if(packet[1]< 78){
				WalkLength.RotY = map(packet[1], 0, 255, -15, 15);
			}

			if (packet[2] < 78){
				BodyPos.Y =map(packet[2], 0, 255, -20, 20);
			}
			else if (packet[2] > 178){
				BodyPos.Y = map(packet[2], 0, 255, -20, 20);
			}

			speed = packet[3];
			break;

		default:
			;
	}


}



void UpdateLeg(float Angle[18]){

	float ServoChange[18]={0.0};

	int delaytime = 2;
	int Resolution = 8;;

	if (mode == 122){ // if walking, delay according to speed
		delaytime = constrain((int)6/(int)speed, 2, 6);
		Resolution = speed;
	}


	for (int i=0; i<18; i++){
		ServoChange[i] = (Angle[i] - Angle_Old[i])/Resolution;
	}

	for (int j=0; j<Resolution; j++){

		for (int i=17; i>=0; i--){

			Angle_Old[i] = Angle_Old[i]+ServoChange[i];
			int mSecond = constrain(RadToMicroV2(Angle_Old[i]+PIby2) + servoOffset[i], servoMin[i%3], servoMax[i%3]);
			legServo[i].writeMicroseconds(mSecond);
			delay(delaytime);
		}

	}

}


// ================================================================
// ================================================================


void setup ()
{

	// ==================== Init Values ======================
	// [DIMENSION PARAMETERS]

	BodyCoxaOffset.X = HexSideLength / 2.0;
	BodyCoxaOffset.Z = sqrt(HexSideLength*HexSideLength - BodyCoxaOffset.X*BodyCoxaOffset.X);

	BCOffset[RF].X = BodyCoxaOffset.X;     //Distance X from centre of the body to the Right Front coxa
	BCOffset[RF].Z = BodyCoxaOffset.Z;          //Distance Z from centre of the body to the Right Front coxa
	BCOffset[RM].X = HexSideLength;          //Distance X from centre of the body to the Right Middle coxa
	BCOffset[RM].Z = 0.0;          //Distance Z from centre of the body to the Right Middle coxa
	BCOffset[RR].X = BodyCoxaOffset.X;          //Distance X from centre of the body to the Right Rear coxa
	BCOffset[RR].Z = -BodyCoxaOffset.Z;          //Distance Z from centre of the body to the Right Rear coxa

	BCOffset[LF].X = -BodyCoxaOffset.X;          //Distance X from centre of the body to the Left Front coxa
	BCOffset[LF].Z = BodyCoxaOffset.Z;          //Distance Z from centre of the body to the Left Front coxa
	BCOffset[LM].X = -HexSideLength;               //Distance Z from centre of the body to the Left Middle coxa
	BCOffset[LM].Z = 0.0;               //Distance Z from centre of the body to the Left Middle coxa
	BCOffset[LR].X = -BodyCoxaOffset.X;          //Distance X from centre of the body to the Left Rear coxa
	BCOffset[LR].Z = -BodyCoxaOffset.Z;          //Distance Z from centre of the body to the Left Rear coxa

	//--------------------------------------------------------------------
	//[INITIAL LEG POSITIONS]

	CurPos[RF].X = cos(DegToRad(60)) * (FemurLength+CoxaLength);          //Start positions of the Right Front leg
	CurPos[RF].Y = TibiaLength;
	CurPos[RF].Z = sin(DegToRad(60)) * (FemurLength+CoxaLength);

	CurPos[RM].X = FemurLength + CoxaLength;     //Start positions of the Right Middle leg
	CurPos[RM].Y = TibiaLength;
	CurPos[RM].Z = 0.0;

	CurPos[RR].X = cos(DegToRad(60)) * (FemurLength+CoxaLength);          //Start positions of the Right Rear leg
	CurPos[RR].Y = TibiaLength;
	CurPos[RR].Z = -sin(DegToRad(60)) * (FemurLength+CoxaLength);

	CurPos[LF].X = -cos(DegToRad(60)) * (FemurLength+CoxaLength);          //Start positions of the Left Front leg
	CurPos[LF].Y = TibiaLength;
	CurPos[LF].Z = sin(DegToRad(60)) * (FemurLength+CoxaLength);

	CurPos[LM].X = -(FemurLength + CoxaLength);     //Start positions of the Left Middle leg
	CurPos[LM].Y = TibiaLength;
	CurPos[LM].Z = 0.0;

	CurPos[LR].X = -cos(DegToRad(60)) * (FemurLength+CoxaLength);          //Start positions of the Left Rear leg
	CurPos[LR].Y = TibiaLength;
	CurPos[LR].Z = -sin(DegToRad(60)) * (FemurLength+CoxaLength);



	Serial.begin (9600);
	// ================  Servos  ==================
	for (int i=0; i<18; i++){
		legServo[i].attach(i+23);
		delay(10);
		legServo[i].writeMicroseconds(1500+servoOffset[i]);
		delay(100);
	}

	// ================  Gait  ==================
	GaitSelect (1);

	//ResetLegPosition();
	delay(2000);

}


void loop (){

	int i = 0;
	float Angle[18] = {0.0};

	coor posIO;
	coor rotIO;

	BodyPos.Reset();
	BodyRot.Reset();
	WalkLength.Reset();

	// see if there's incoming serial data:
	if (Serial.available() > 0)  {
		packet[pIndex++] = Serial.read();
	}

     // If we received the 3 bytes, then go execute them
	if(pIndex >= 4){

        // --------- Controller Input --------------
        GetCommandValues();

        // -------- Gaits Calculation --------------
		GaitCalculate(WalkLength);

		pIndex = 0;
		Serial.write(1); // send the request for next bytes

		// ------- Balancing Calculation ------------
		//BodyBalance();

		// ----- IK -------

		for (int LegIndex = 0; LegIndex < 6; LegIndex++){

			posIO.X = CurPos[LegIndex].X + BodyPos.X + GaitPos[LegIndex].X;
			posIO.Z = CurPos[LegIndex].Z + BodyPos.Z + GaitPos[LegIndex].Z;
			posIO.Y = CurPos[LegIndex].Y + BodyPos.Y + GaitPos[LegIndex].Y;

			rotIO.X = BodyRot.X;
			rotIO.Z = BodyRot.Z;
			rotIO.Y = BodyRot.Y + GaitPos[LegIndex].RotY;

			InverseKinematics(rotIO, posIO, BCOffset[LegIndex], AngleOffset[LegIndex], &Angle[i++], &Angle[i++], &Angle[i++]);
			//Angle[i++] = IKCoxaAngle;
			//Angle[i++] = IKFemurAngle;
			//Angle[i++] = IKTibiaAngle;

		}

		//PrintServoAngles();
		UpdateLeg(Angle);

		// Save last loop angle values
		for (int i=0;i<18;i++){
			Angle_Old[i] = Angle[i];
		}

	}



}

// ================================================================
