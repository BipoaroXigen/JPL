#include "RAPID.H"
#include <math.h>
#include <stdio.h>
#include <iostream>

extern "C" {

RAPID_model *ENV = new RAPID_model;
RAPID_model *OBJ = new RAPID_model;
RAPID_model *PRB = new RAPID_model;

int init(double ** matrix) {  // not used

	ENV->BeginModel();
	ENV->AddTri(matrix[0], matrix[1], matrix[2], 10);
	ENV->EndModel();

	OBJ->BeginModel();
	OBJ->AddTri(matrix[0], matrix[1], matrix[2], 20);
	OBJ->EndModel();

	return 0;
}

struct collision_pair* tell() {
	return RAPID_contact;
}

int init_env(double *** env, int len, int id_offset) {
	ENV->BeginModel();
	for (int i=0;i<len;i++) {
//		ENV->AddTri(env[i][0], env[i][1], env[i][2], i+len+10);
		ENV->AddTri(env[i][0], env[i][1], env[i][2], i+id_offset);
	}
	//ENV->EndModel();

	return 0;
}

int init_rob(double *** rob, int len) {
	OBJ->BeginModel();
	for (int i=0;i<len;i++) {
		OBJ->AddTri(rob[i][0], rob[i][1], rob[i][2], i);
		//std::cout << "hi\n";
	}
	OBJ->EndModel();
	return 0;
}

int init_gol(double *** gol, int len, int id_offset) {
	//GOL->BeginModel();
	// instead added to ENV model
	for (int i=0;i<len;i++) {
		ENV->AddTri(gol[i][0], gol[i][1], gol[i][2], i+id_offset);
	}
	ENV->EndModel();

	return 0;
}

int init_prb(double *** prb, int len) {
	PRB->BeginModel();
	for (int i=0;i<len;i++) {
		PRB->AddTri(prb[i][0], prb[i][1], prb[i][2], i);
	}
	PRB->EndModel();

	return 0;
}

int detect_collision(double ** robot_rotation, double * robot_translation) {
	double R1[3][3], R2[3][3];
	double T1[3], T2[3];

   	R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
   	R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
   	R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

   	T1[0] = 0.0;  T1[1] = 0.0; T1[2] = 0.0;
   	
	R2[0][0] = robot_rotation[0][0];
	R2[1][1] = robot_rotation[1][1];
	R2[2][2] = robot_rotation[2][2];
   	R2[0][1] = robot_rotation[0][1];
	R2[1][0] = robot_rotation[1][0];
	R2[2][0] = robot_rotation[2][0];
   	R2[0][2] = robot_rotation[0][2];
	R2[1][2] = robot_rotation[1][2];
	R2[2][1] = robot_rotation[2][1];

   	T2[0] = robot_translation[0];  
	T2[1] = robot_translation[1]; 
	T2[2] = robot_translation[2];


	int res = RAPID_Collide(R1, T1, ENV, R2, T2, OBJ, RAPID_ALL_CONTACTS);
	//int res = RAPID_Collide(R1, T1, ENV, R2, T2, OBJ, RAPID_FIRST_CONTACT);
	// the other constant 
	return RAPID_num_contacts;
}

int probe_collision(double ** robot_rotation, double * robot_translation) {
	double R1[3][3], R2[3][3];
	double T1[3], T2[3];

   	R1[0][0] = R1[1][1] = R1[2][2] = 1.0;
   	R1[0][1] = R1[1][0] = R1[2][0] = 0.0;
   	R1[0][2] = R1[1][2] = R1[2][1] = 0.0;

   	T1[0] = 0.0;  T1[1] = 0.0; T1[2] = 0.0;
   	
	R2[0][0] = robot_rotation[0][0];
	R2[1][1] = robot_rotation[1][1];
	R2[2][2] = robot_rotation[2][2];
   	R2[0][1] = robot_rotation[0][1];
	R2[1][0] = robot_rotation[1][0];
	R2[2][0] = robot_rotation[2][0];
   	R2[0][2] = robot_rotation[0][2];
	R2[1][2] = robot_rotation[1][2];
	R2[2][1] = robot_rotation[2][1];

   	T2[0] = robot_translation[0];  
	T2[1] = robot_translation[1]; 
	T2[2] = robot_translation[2];


	int res = RAPID_Collide(R1, T1, ENV, R2, T2, PRB, RAPID_ALL_CONTACTS);
	//int res = RAPID_Collide(R1, T1, ENV, R2, T2, PRB, RAPID_FIRST_CONTACT);
	// the other constant 
	return RAPID_num_contacts;
}


int collides(double ** env_rotation, double * env_translation, double ** robot_rotation, double * robot_translation) {
	double R1[3][3], R2[3][3];
	double T1[3], T2[3];

	R1[0][0] = env_rotation[0][0];
	R1[1][1] = env_rotation[1][1];
	R1[2][2] = env_rotation[2][2];
   	R1[0][1] = env_rotation[0][1];
	R1[1][0] = env_rotation[1][0];
	R1[2][0] = env_rotation[2][0];
   	R1[0][2] = env_rotation[0][2];
	R1[1][2] = env_rotation[1][2];
	R1[2][1] = env_rotation[2][1];

   	T1[0] = env_translation[0];  
	T1[1] = env_translation[1]; 
	T1[2] = env_translation[2];




	R2[0][0] = robot_rotation[0][0];
	R2[1][1] = robot_rotation[1][1];
	R2[2][2] = robot_rotation[2][2];
   	R2[0][1] = robot_rotation[0][1];
	R2[1][0] = robot_rotation[1][0];
	R2[2][0] = robot_rotation[2][0];
   	R2[0][2] = robot_rotation[0][2];
	R2[1][2] = robot_rotation[1][2];
	R2[2][1] = robot_rotation[2][1];

   	T2[0] = robot_translation[0];  
	T2[1] = robot_translation[1]; 
	T2[2] = robot_translation[2];


	int res = RAPID_Collide(R1, T1, PRB, R2, T2, OBJ, RAPID_ALL_CONTACTS);
	//int res = RAPID_Collide(R1, T1, ENV, R2, T2, PRB, RAPID_FIRST_CONTACT);
	// the other constant 
	return RAPID_num_contacts;
}





}
