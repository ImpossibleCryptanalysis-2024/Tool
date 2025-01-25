#include "gurobi_c++.h"
#include <iostream>

using namespace std;

#define ROUND 23
#define midRound 12
// #define minDistLength 14
#define label 5
#define r1 4
#define r2 4
#define state 16

// 3-XOR
// x[0,1] + y[0,1] + z[0,1] = k[0,1]
// d - distinguisher or key-recovery (quadratic constraints)
void _3XOR_deterministic(GRBModel& model, GRBVar x0, GRBVar x1, GRBVar y0, GRBVar y1, GRBVar z0, GRBVar z1, GRBVar k0, GRBVar k1, GRBVar d) {
	model.addQConstr(d * (x1 + y1 + z1 + 2 * k0 - k1 - 2) >= 0);
	model.addQConstr(d * (x0 + y0 + z0 + 2 * k0 + 2 * k1 - 2) >= 0);
	model.addQConstr(d * (x0 + y1 + z1 + 2 * k0 - 2) >= 0);
	model.addQConstr(d * (2 - x1 - y1 - z1 + k1) >= 0);
	model.addQConstr(d * (2 - x0 - y0 - z1 + k0 + k1) >= 0);
	model.addQConstr(d * (2 - x0 - y0 - z0 + k0) >= 0);
	model.addQConstr(d * (4 + x0 - y0 - y1 - z0 - z1 - k0) >= 0);
	model.addQConstr(d * (4 - x0 - x1 + y0 - z0 - z1 - k0) >= 0);
	model.addQConstr(d * (4 - x0 - x1 - y0 - y1 + z0 - k0) >= 0);
	model.addQConstr(d * (y1 - k1) >= 0);
	model.addQConstr(d * (x1 - k1) >= 0);
	model.addQConstr(d * (z1 - k1) >= 0);
	model.addQConstr(d * (z1 - z0 + k0) >= 0);
	model.addQConstr(d * (y0 - y1 + k0 + k1) >= 0);
}

void _3XOR_probabilistic(GRBModel& model, GRBVar x0, GRBVar x1, GRBVar y0, GRBVar y1, GRBVar z0, GRBVar z1, GRBVar k0, GRBVar k1, GRBVar d) {
	model.addQConstr(d * (x0 + y0 + z0 + 2 * k0 + 2 * k1 - 2) >= 0);
	model.addQConstr(d * (x1 + y1 + z1 + 2 * k0 + 2 * k1 - 2) >= 0);
	model.addQConstr(d * (6 - 2 * x0 - x1 - 2 * y0 - y1 - 2 * z0 - z1 + 2 * k0 + k1) >= 0);
	model.addQConstr(d * (2 - x0 - y1 - z1 + k0 + k1) >= 0);
	model.addQConstr(d * (2 - x1 - y0 - z0 + k0 + k1) >= 0);
	model.addQConstr(d * (2 - x1 - y1 - z1 + k1) >= 0);
	model.addQConstr(d * (4 - x0 - x1 + y0 - z0 - z1 - k0) >= 0);
	model.addQConstr(d * (4 + x1 - y0 - y1 - z0 - z1 - k1) >= 0);
	model.addQConstr(d * (4 - x0 - x1 - y0 - y1 + z0 - k0) >= 0);
	model.addQConstr(d * (4 - x0 - x1 + y1 - z0 - z1 - k1) >= 0);
	model.addQConstr(d * (4 + x0 - y0 - y1 - z0 - z1 - k0) >= 0);
	model.addQConstr(d * (4 - x0 - x1 - y0 - y1 + z1 - k1) >= 0);
	model.addQConstr(d * (y1 + z0 + k0 + k1 - 1) >= 0);
	model.addQConstr(d * (y0 + z1 + k0 + k1 - 1) >= 0);
}

// 2-XOR
// x[0,1] + y[0,1] = k[0,1]
// d - distinguisher or key-recovery (quadratic constraints)
void _2XOR_deterministic(GRBModel& model, GRBVar x0, GRBVar x1, GRBVar y0, GRBVar y1, GRBVar k0, GRBVar k1, GRBVar d) {
	model.addQConstr(d * (1 - x1 - y1 + k1) >= 0);
	model.addQConstr(d * (x1 - k1) >= 0);
	model.addQConstr(d * (y1 - k1) >= 0);
	model.addQConstr(d * (1 - x0 - y0 + k0) >= 0);
	model.addQConstr(d * (x1 + y0 + k0 - 1) >= 0);
	model.addQConstr(d * (x0 + y1 + k0 - 1) >= 0);
	model.addQConstr(d * (2 - x0 - x1 + y0 - k0) >= 0);
	model.addQConstr(d * (x0 - y0 - y1 - k0 + 2) >= 0);
}

void _2XOR_probabilistic(GRBModel& model, GRBVar x0, GRBVar x1, GRBVar y0, GRBVar y1, GRBVar k0, GRBVar k1, GRBVar d) {
	model.addQConstr(d * (3 - 2 * x0 - x1 - 2 * y0 - y1 + 2 * k0 + k1) >= 0);
	model.addQConstr(d * (1 - x1 - y1 + k1) >= 0);
	model.addQConstr(d * (x0 + y1 + k0 + k1 - 1) >= 0);
	model.addQConstr(d * (x1 + y0 + k0 + k1 - 1) >= 0);
	model.addQConstr(d * (2 - x0 - x1 + y0 - k0) >= 0);
	model.addQConstr(d * (2 - x0 - x1 + y1 - k1) >= 0);
	model.addQConstr(d * (2 + x0 - y0 - y1 - k0) >= 0);
	model.addQConstr(d * (2 + x1 - y0 - y1 - k1) >= 0);
}

// Initialize roundkey variables
// &
// SKINNY-2n Key Schedule
void keySchedule(GRBModel& model, vector<vector<vector<GRBVar>>>& roundkey) {
	int p[16] = { 9, 15, 8, 13, 10, 14, 12, 11, 0, 1, 2, 3, 4, 5, 6, 7 };
	int pp[16] = { 8,9,10,11,12,13,14,15,2,0,4,7,6,3,5,1 };
	// roundkey[round][state][s0 / s1 / involved / guess / known]
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// Initialize
			roundkey[r][i][0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			roundkey[r][i][1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			roundkey[r][i][2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			roundkey[r][i][3] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			roundkey[r][i][4] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

			// Linear Key Schedule => All roundkey differences are fixed.
			model.addConstr(roundkey[r][i][1] == 1);

			// Constraints for relationships within the attributes
			// i = 0 = > g = 0
			// g >= 1 => k = 1
			model.addGenConstrIndicator(roundkey[r][i][2], 0, roundkey[r][i][3] == 0);
			model.addGenConstrIndicator(roundkey[r][i][4], 0, roundkey[r][i][3] == 0);
		}
	}

	// Subtweakey Difference Cancellation 
	// & 
	// Key Bridge from linear key schedule
	// 
	// counter for s0|s1=11 and k=1 in ROUND rounds：0,1,ROUND
	for (int i = 0; i < state; i++) {
		GRBQuadExpr KS1 = 0;
		int j = i;
		for (int r = 0; r < ROUND; r++) {
			KS1 += roundkey[r][j][0] * roundkey[r][j][1];
			j = pp[j];
		}
		GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addQConstr(KS1 <= ROUND - (ROUND - 1) * e1);
		model.addQConstr(KS1 >= ROUND - ROUND * e1);
	}

	// Related-key Setting
	GRBQuadExpr ks = 0;
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			ks += roundkey[r][i][0] * roundkey[r][i][1];
		}
	}
	model.addQConstr(ks <= 16 * ROUND - 1);

	//// Single-Key Setting
	//for (int r = 0; r < ROUND; r++) {
	//	for (int i = 0; i < state; i++) {
	//		model.addConstr(roundkey[r][i][0] + roundkey[r][i][1] == 2);
	//	}
	//}
}

// Initialize state variables
void internalState_U(GRBModel& model, vector<vector<GRBVar>>& plaintext, vector<vector<GRBVar>>& ciphertext, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	// state[round][state][s0 / s1 / d / c / rb]
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			for (int j = 0; j < 4; j++) {
				StateX[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateY[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateZ[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateW[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			}
			StateX[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateY[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateZ[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateW[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		}
		GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e4 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBLinExpr Dist1 = 0;
		GRBLinExpr Dist2 = 0;
		GRBLinExpr Dist3 = 0;
		GRBLinExpr Dist4 = 0;
		for (int i = 0; i < state; i++) {
			Dist1 += StateX[r][i][2];
			Dist2 += StateY[r][i][2];
			Dist3 += StateZ[r][i][2];
			Dist4 += StateW[r][i][2];
		}
		model.addConstr(Dist1 == 16 * e1);
		model.addConstr(Dist2 == 16 * e2);
		model.addConstr(Dist3 == 16 * e3);
		model.addConstr(Dist4 == 16 * e4);
	}

	// Plaintext (△P in attacks) 
	// & 
	// Ciphertext (△C, useless in attacks)
	for (int i = 0; i < state; i++) {
		for (int j = 0; j < 4; j++) {
			plaintext[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			ciphertext[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(plaintext[i][j] == StateX[0][i][j]);
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		model.addConstr(plaintext[i][4] == StateX[0][i][4]);

		model.addConstr(plaintext[i][2] == 0);
		model.addConstr(StateX[midRound - 2][i][2] == 1);
	}

	//// At least one cell with k=1 in Plaintext
	//GRBLinExpr up_p = 0;
	//for (int i = 0; i < state; i++) {
	//	up_p += plaintext[i][1];
	//}
	//model.addConstr(up_p >= 1);

	//model.addConstr(StateX[0][0][2] == 0);
}

void internalState_L(GRBModel& model, vector<vector<GRBVar>>& plaintext, vector<vector<GRBVar>>& ciphertext, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	// state[round][state][s0 / s1 / d / c / rb]
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			for (int j = 0; j < 4; j++) {
				StateX[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateY[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateZ[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				StateW[r][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			}
			StateX[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateY[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateZ[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
			StateW[r][i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		}
		GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar e4 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBLinExpr Dist1 = 0;
		GRBLinExpr Dist2 = 0;
		GRBLinExpr Dist3 = 0;
		GRBLinExpr Dist4 = 0;
		for (int i = 0; i < state; i++) {
			Dist1 += StateX[r][i][2];
			Dist2 += StateY[r][i][2];
			Dist3 += StateZ[r][i][2];
			Dist4 += StateW[r][i][2];
		}
		model.addConstr(Dist1 == 16 * e1);
		model.addConstr(Dist2 == 16 * e2);
		model.addConstr(Dist3 == 16 * e3);
		model.addConstr(Dist4 == 16 * e4);
	}

	// Plaintext (▽P, useless in attacks) 
	// & 
	// Ciphertext (▽C in attacks)
	for (int i = 0; i < state; i++) {
		for (int j = 0; j < 4; j++) {
			plaintext[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			ciphertext[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(ciphertext[i][j] == StateW[ROUND - 1][i][j]);
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		model.addConstr(ciphertext[i][4] == StateW[ROUND - 1][i][4]);

		model.addConstr(StateY[midRound + 2][i][2] == 1);
		model.addConstr(ciphertext[i][2] == 0);
	}

	//// At least one cell with k=1 in Ciphertext
	//GRBLinExpr lo_c = 0;
	//for (int i = 0; i < state; i++) {
	//	lo_c += ciphertext[i][1];
	//}
	//model.addConstr(lo_c >= 1);

	//model.addConstr(StateX[ROUND - 1][0][2] == 0);
}

// X_r SB Y_r
void SubBytes_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY) {
	// Difference Propagatioon
	//						Xd		Yd
	//						1		1
	// Xs0 Xs1 Ys0 Ys1						Inequalities
	// 0	0	0	0						Xs0 - Ys0 == 0
	// 0	1	0	0						Xs0 - Ys1 >= 0
	// 1	0	1	0						Xs1 - Ys1 >= 0
	// 1	1	1	1						Ys1 - Xs0 - Xs1 >= -1
	// 
	//						Xd		Yd
	//						0		0/1
	// Xs0 Xs1 Ys0 Ys1						Inequalities
	// 1	1	1	1						Xs0 - Ys0 == 0
	// 1	0	1	0						Ys1 - Xs1 >= 0
	// 0	1	0	1						Xs1 - Ys0 - Ys1 >= -1 
	// 0	0	0	1						
	// 0	0	0	0						
	//
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// Xd <= Yd 
			model.addConstr(StateX[r][i][2] - StateY[r][i][2] <= 0);
			// Xs0 = Ys0
			model.addConstr(StateX[r][i][0] == StateY[r][i][0]);
			// Others
			model.addQConstr(StateX[r][i][2] * (StateX[r][i][1] - StateY[r][i][1]) + (1 - StateX[r][i][2]) * (StateY[r][i][1] - StateX[r][i][1]) >= 0);
			model.addQConstr(StateX[r][i][2] * (StateY[r][i][1] - StateX[r][i][0] - StateX[r][i][1]) + (1 - StateX[r][i][2]) * (StateX[r][i][1] - StateY[r][i][0] - StateY[r][i][1]) >= -1);
			model.addQConstr(StateX[r][i][2] * (StateX[r][i][0] - StateY[r][i][1]) >= 0);
		}
	}

	// Cell-Condition Determination
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			//	Xd=0 && Ys0=0 && Ys1=1 <=> Yc=1
			GRBVar Xd_NOT = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys0_NOT = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xd_NOT == 1 - StateX[r][i][2]);
			model.addConstr(Ys0_NOT == 1 - StateY[r][i][0]);
			GRBVar Arr_AND[3] = { Xd_NOT,Ys0_NOT,StateY[r][i][1] };
			model.addGenConstrAnd(StateY[r][i][3], Arr_AND, 3);
		}
	}

	// Cell-Condition Satisfied to Generate Quartets
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// Yc=0 => Yc'=0
			// Yc'=1 => Yc=1
			model.addGenConstrIndicator(StateY[r][i][3], 0, StateY[r][i][4] == 0);
			model.addGenConstrIndicator(StateY[r][i][4], 1, StateY[r][i][3] == 1);
		}
	}
}

void SubBytes_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY) {
	//						Xd		Yd
	//						1		1
	// Xs0 Xs1 Ys0 Ys1						Inequalities
	// 0	0	0	0						Xs0 - Ys0 == 0
	// 0	0	0	1						Ys0 - Xs1 >= 0
	// 1	0	1	0						Ys1 - Xs1 >= 0
	// 1	1	1	1						Xs1 - Ys0 - Ys1 >= -1
	// 
	//						Xd		Yd
	//						0/1		0
	// Xs0 Xs1 Ys0 Ys1						Inequalities
	// 1	1	1	1						Xs0 - Ys0 == 0
	// 1	0	1	0						Xs1 - Ys1 >= 0
	// 0	1	0	1						Ys1 - Xs1 - Ys0 >= -1 
	// 0	1	0	0						
	// 0	0	0	0						
	//
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// Xd >= Yd
			model.addConstr(StateX[r][i][2] - StateY[r][i][2] >= 0);
			// Xs0 = Ys0
			model.addConstr(StateX[r][i][0] == StateY[r][i][0]);
			// Others
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][1] - StateX[r][i][1]) + (1 - StateY[r][i][2]) * (StateX[r][i][1] - StateY[r][i][1]) >= 0);
			model.addQConstr(StateY[r][i][2] * (StateX[r][i][1] - StateY[r][i][0] - StateY[r][i][1]) + (1 - StateY[r][i][2]) * (StateY[r][i][1] - StateY[r][i][0] - StateX[r][i][1]) >= -1);
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][0] - StateX[r][i][1]) >= 0);
		}
	}

	// Cell-Condition Determination
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			//	Yd=0 && Xs0=0 && Xs1=1 <=> Xc=1
			GRBVar Yd_NOT = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs0_NOT = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Yd_NOT == 1 - StateY[r][i][2]);
			model.addConstr(Xs0_NOT == 1 - StateX[r][i][0]);
			GRBVar Arr_AND[3] = { Yd_NOT,Xs0_NOT,StateX[r][i][1] };
			model.addGenConstrAnd(StateX[r][i][3], Arr_AND, 3);
		}
	}

	// Cell-Condition Satisfied to Generate Quartets
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// Xc=0 => Xc'=0
			// Xc'=1 => Xc=1
			model.addGenConstrIndicator(StateX[r][i][3], 0, StateX[r][i][4] == 0);
			model.addGenConstrIndicator(StateX[r][i][4], 1, StateX[r][i][3] == 1);
		}
	}
}

// Y_r ART Z_r
void AddRoundKey_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	// internal round 
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < 8; i++) {
			model.addQConstr(StateY[r][i][2] * (1 - StateY[r][i][1] - key[r][i][1] + StateZ[r][i][1]) + (1 - StateY[r][i][2]) * (1 - StateZ[r][i][1] - key[r][i][1] + StateY[r][i][1]) >= 0);
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][1] - StateZ[r][i][1]) + (1 - StateY[r][i][2]) * (StateZ[r][i][1] - StateY[r][i][1]) >= 0);
			model.addQConstr(StateY[r][i][2] * (key[r][i][1] - StateZ[r][i][1]) + (1 - StateY[r][i][2]) * (key[r][i][1] - StateY[r][i][1]) >= 0);
			model.addQConstr(StateY[r][i][2] * (1 - StateY[r][i][0] - key[r][i][0] + StateZ[r][i][0]) + (1 - StateY[r][i][2]) * (1 - StateZ[r][i][0] - key[r][i][0] + StateY[r][i][0]) >= 0);
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][1] + key[r][i][0] + StateZ[r][i][0] - 1) + (1 - StateY[r][i][2]) * (StateZ[r][i][1] + key[r][i][0] + StateY[r][i][0] - 1) >= 0);
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][0] + key[r][i][1] + StateZ[r][i][0] - 1) + (1 - StateY[r][i][2]) * (StateZ[r][i][0] + key[r][i][1] + StateY[r][i][0] - 1) >= 0);
			model.addQConstr(StateY[r][i][2] * (2 - StateY[r][i][0] - StateY[r][i][1] + key[r][i][0] - StateZ[r][i][0]) + (1 - StateY[r][i][2]) * (2 - StateZ[r][i][0] - StateZ[r][i][1] + key[r][i][0] - StateY[r][i][0]) >= 0);
			model.addQConstr(StateY[r][i][2] * (StateY[r][i][0] - key[r][i][0] - key[r][i][1] - StateZ[r][i][0] + 2) + (1 - StateY[r][i][2]) * (StateZ[r][i][0] - key[r][i][0] - key[r][i][1] - StateY[r][i][0] + 2) >= 0);
			model.addConstr(StateY[r][i][2] <= StateZ[r][i][2]);
			model.addConstr(StateZ[r][i][3] == 0);
			model.addConstr(StateZ[r][i][4] == 0);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(StateY[r][i][0] == StateZ[r][i][0]);
			model.addConstr(StateY[r][i][1] == StateZ[r][i][1]);
			model.addConstr(StateY[r][i][2] <= StateZ[r][i][2]);
			model.addConstr(StateZ[r][i][3] == 0);
			model.addConstr(StateZ[r][i][4] == 0);
		}
	}
}

void AddRoundKey_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	// internal round 
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < 8; i++) {
			model.addQConstr(StateZ[r][i][2] * (1 - StateZ[r][i][1] - key[r][i][1] + StateY[r][i][1]) + (1 - StateZ[r][i][2]) * (1 - StateY[r][i][1] - key[r][i][1] + StateZ[r][i][1]) >= 0);
			model.addQConstr(StateZ[r][i][2] * (StateZ[r][i][1] - StateY[r][i][1]) + (1 - StateZ[r][i][2]) * (StateY[r][i][1] - StateZ[r][i][1]) >= 0);
			model.addQConstr(StateZ[r][i][2] * (key[r][i][1] - StateY[r][i][1]) + (1 - StateZ[r][i][2]) * (key[r][i][1] - StateZ[r][i][1]) >= 0);
			model.addQConstr(StateZ[r][i][2] * (1 - StateZ[r][i][0] - key[r][i][0] + StateY[r][i][0]) + (1 - StateZ[r][i][2]) * (1 - StateY[r][i][0] - key[r][i][0] + StateZ[r][i][0]) >= 0);
			model.addQConstr(StateZ[r][i][2] * (StateZ[r][i][1] + key[r][i][0] + StateY[r][i][0] - 1) + (1 - StateZ[r][i][2]) * (StateY[r][i][1] + key[r][i][0] + StateZ[r][i][0] - 1) >= 0);
			model.addQConstr(StateZ[r][i][2] * (StateZ[r][i][0] + key[r][i][1] + StateY[r][i][0] - 1) + (1 - StateZ[r][i][2]) * (StateY[r][i][0] + key[r][i][1] + StateZ[r][i][0] - 1) >= 0);
			model.addQConstr(StateZ[r][i][2] * (2 - StateZ[r][i][0] - StateZ[r][i][1] + key[r][i][0] - StateY[r][i][0]) + (1 - StateZ[r][i][2]) * (2 - StateY[r][i][0] - StateY[r][i][1] + key[r][i][0] - StateZ[r][i][0]) >= 0);
			model.addQConstr(StateZ[r][i][2] * (StateZ[r][i][0] - key[r][i][0] - key[r][i][1] - StateY[r][i][0] + 2) + (1 - StateZ[r][i][2]) * (StateY[r][i][0] - key[r][i][0] - key[r][i][1] - StateZ[r][i][0] + 2) >= 0);
			model.addConstr(StateZ[r][i][2] <= StateY[r][i][2]);
			model.addConstr(StateY[r][i][3] == 0);
			model.addConstr(StateY[r][i][4] == 0);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(StateY[r][i][0] == StateZ[r][i][0]);
			model.addConstr(StateY[r][i][1] == StateZ[r][i][1]);
			model.addConstr(StateZ[r][i][2] <= StateY[r][i][2]);
			model.addConstr(StateY[r][i][3] == 0);
			model.addConstr(StateY[r][i][4] == 0);
		}
	}
}

// Z_r SR W_r
void ShiftRow_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	int shift[16] = { 0,1,2,3,7,4,5,6,10,11,8,9,13,14,15,12 };
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// s0 s1 d - remaining
			model.addConstr(StateW[r][i][0] == StateZ[r][shift[i]][0]);
			model.addConstr(StateW[r][i][1] == StateZ[r][shift[i]][1]);
			model.addConstr(StateW[r][i][2] == StateZ[r][shift[i]][2]);
			// c rb - 0
			model.addConstr(StateW[r][i][3] == 0);
			model.addConstr(StateW[r][i][4] == 0);
		}
	}
}

void ShiftRow_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	int shift[16] = { 0,1,2,3,7,4,5,6,10,11,8,9,13,14,15,12 };
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			// s0 s1 d - remaining
			model.addConstr(StateW[r][i][0] == StateZ[r][shift[i]][0]);
			model.addConstr(StateW[r][i][1] == StateZ[r][shift[i]][1]);
			model.addConstr(StateW[r][i][2] == StateZ[r][shift[i]][2]);
			// c rb - 0
			model.addConstr(StateZ[r][i][3] == 0);
			model.addConstr(StateZ[r][i][4] == 0);
		}
	}
}

void MixColumns_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX) {
	for (int r = 0; r < ROUND - 1; r++) {
		// Wd <= Xd
		model.addConstr(StateW[r][0][2] <= StateX[r + 1][0][2]);

		for (int col = 0; col < 4; col++) {
			// In Distinguisher (Wd = 1): Deterministic Propagation
			{
				// Row 0
				_2XOR_deterministic(model, StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 12][0], StateW[r][col + 12][1], StateX[r + 1][col][0], StateX[r + 1][col][1], StateW[r][col][2]);
				// Row 1
				model.addConstr(StateW[r][col][0] == StateX[r + 1][col + 4][0]);
				model.addConstr(StateW[r][col][1] == StateX[r + 1][col + 4][1]);
				// Row 2
				_2XOR_deterministic(model, StateW[r][col + 4][0], StateW[r][col + 4][1], StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 8][0], StateX[r + 1][col + 8][1], StateW[r][col][2]);
				// Row 3
				_2XOR_deterministic(model, StateW[r][col][0], StateW[r][col][1], StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col][2]);

			}

			// In Key-Recovery (Wd = 0): Probabilistic Extension <-
			{
				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][col][2]);
				// Row 0
				model.addConstr(StateX[r + 1][col + 4][0] == StateW[r][col][0]);
				model.addConstr(StateX[r + 1][col + 4][1] == StateW[r][col][1]);
				// Row 1
				_2XOR_probabilistic(model, StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 8][0], StateX[r + 1][col + 8][1], StateW[r][col + 4][0], StateW[r][col + 4][1], Wd_Not);
				// Row 2
				_2XOR_probabilistic(model, StateX[r + 1][col + 4][0], StateX[r + 1][col + 4][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 8][0], StateW[r][col + 8][1], Wd_Not);
				// Row 3
				_2XOR_probabilistic(model, StateX[r + 1][col][0], StateX[r + 1][col][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 12][0], StateW[r][col + 12][1], Wd_Not);
			}

			// Determine the Cell-Condition
			{
				// Wd=0 & ∑ Ws1<=3 & Xs1=1 => Xc=1 
				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][col][2]);

				// Row 0
				{
					// s1: 0 0 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateW[r][col + 0][1] + StateW[r][col + 8][1] + StateW[r][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateW[r][col + 0][1] + StateW[r][col + 8][1] + StateW[r][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[3] = { Wd_Not,MC_Ind0,StateX[r + 1][col + 0][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 3);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] == 1);
				}
				{
					// s1: 1 0 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateW[r][col + 8][1] + StateW[r][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateW[r][col + 8][1] + StateW[r][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Wd_Not,MC_Ind0,StateX[r + 1][col + 0][1],StateW[r][col + 0][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] == 1);
				}
				{
					// s1: 0 1 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateW[r][col + 0][1] + StateW[r][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateW[r][col + 0][1] + StateW[r][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Wd_Not,MC_Ind0,StateX[r + 1][col + 0][1],StateW[r][col + 8][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] == 1);
				}
				{
					// s1: 0 0 1 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateW[r][col + 0][1] + StateW[r][col + 8][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateW[r][col + 0][1] + StateW[r][col + 8][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Wd_Not,MC_Ind0,StateX[r + 1][col + 0][1],StateW[r][col + 12][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] + StateX[r + 1][col + 12][3] == 1);
				}
				{
					// s1: 0 1 1 / 101 / 110  -> 1
					/*GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateW[r][col + 0][1] + StateW[r][col + 8][1] + StateW[r][col + 12][1] <= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateW[r][col + 0][1] + StateW[r][col + 8][1] + StateW[r][col + 12][1] >= 2);
					GRBVar MC_Temp_AND0[2] = { Wd_Not,MC_Ind0 };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 2);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] == 0);*/

					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					GRBLinExpr Ws1_Sum = StateW[r][col + 0][1] + StateW[r][col + 8][1] + StateW[r][col + 12][1];
					GRBVar MC_Temp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
					model.addConstr(MC_Temp0 == Ws1_Sum - 2);
					GRBVar MC_Temp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
					model.addGenConstrAbs(MC_Temp1, MC_Temp0);
					model.addGenConstrIndicator(MC_Ind0, 0, MC_Temp1 >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, Ws1_Sum == 2);
					GRBVar MC_Temp_AND0[3] = { Wd_Not,MC_Ind0, StateX[r + 1][col + 0][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 3);
					model.addGenConstrIndicator(MC_Ind1, 1, StateX[r + 1][col + 0][3] == 1);

				}

				// Row 1
				model.addConstr(StateX[r + 1][col + 4][3] == 0);

				// Row 2
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, StateW[r][col + 4][1] + StateW[r][col + 8][1] == 2);
				model.addGenConstrIndicator(MC_Ind2, 1, StateW[r][col + 4][1] + StateW[r][col + 8][1] <= 1);
				GRBVar MC_Temp_AND2[3] = { Wd_Not,MC_Ind2,StateX[r + 1][col + 8][1] };
				model.addGenConstrAnd(StateX[r + 1][col + 8][3], MC_Temp_AND2, 3);

				// Row 3
				GRBVar MC_Ind3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind3, 0, StateW[r][col + 0][1] + StateW[r][col + 8][1] == 2);
				model.addGenConstrIndicator(MC_Ind3, 1, StateW[r][col + 0][1] + StateW[r][col + 8][1] <= 1);
				GRBVar MC_Temp_AND3[3] = { Wd_Not,MC_Ind3,StateX[r + 1][col + 12][1] };
				model.addGenConstrAnd(StateX[r + 1][col + 12][3], MC_Temp_AND3, 3);
			}

			model.addGenConstrIndicator(StateX[r + 1][col + 0][3], 0, StateX[r + 1][col + 0][4] == 0);
			model.addGenConstrIndicator(StateX[r + 1][col + 0][4], 1, StateX[r + 1][col + 0][3] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 4][3], 0, StateX[r + 1][col + 4][4] == 0);
			model.addGenConstrIndicator(StateX[r + 1][col + 4][4], 1, StateX[r + 1][col + 4][3] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 0, StateX[r + 1][col + 8][4] == 0);
			model.addGenConstrIndicator(StateX[r + 1][col + 8][4], 1, StateX[r + 1][col + 8][3] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 0, StateX[r + 1][col + 12][4] == 0);
			model.addGenConstrIndicator(StateX[r + 1][col + 12][4], 1, StateX[r + 1][col + 12][3] == 1);
		}
	}
}

// W_r MC X_r+1
void MixColumns_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX) {
	for (int r = 0; r < ROUND - 1; r++) {
		// Xd <= Wd
		model.addConstr(StateX[r + 1][0][2] <= StateW[r][0][2]);

		for (int col = 0; col < 4; col++) {
			// In Distinguisher (Xd = 1): Deterministic Propagation
			{
				// Row 0
				model.addConstr(StateW[r][col][0] == StateX[r + 1][col + 4][0]);
				model.addConstr(StateW[r][col][1] == StateX[r + 1][col + 4][1]);
				// Row 1
				//_3XOR_deterministic(model, StateX[r + 1][col + 4][0], StateX[r + 1][col + 4][1], StateX[r + 1][col + 8][0], StateX[r + 1][col + 8][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 4][0], StateW[r][col + 4][1], StateX[r + 1][col][2]);
				_2XOR_deterministic(model, StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 8][0], StateX[r + 1][col + 8][1], StateW[r][col + 4][0], StateW[r][col + 4][1], StateX[r + 1][col][2]);
				// Row 2
				_2XOR_deterministic(model, StateX[r + 1][col + 4][0], StateX[r + 1][col + 4][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col][2]);
				// Row 3
				_2XOR_deterministic(model, StateX[r + 1][col][0], StateX[r + 1][col][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 12][0], StateW[r][col + 12][1], StateX[r + 1][col][2]);
			}

			// In Key-Recovery (Xd = 0): Probabilistic Extension ->
			{
				GRBVar Xd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Xd_Not == 1 - StateX[r + 1][col][2]);
				// Row 0
				_2XOR_probabilistic(model, StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], StateW[r][col + 12][0], StateW[r][col + 12][1], StateX[r + 1][col][0], StateX[r + 1][col][1], Xd_Not);
				// Row 1
				model.addConstr(StateW[r][col][0] == StateX[r + 1][col + 4][0]);
				model.addConstr(StateW[r][col][1] == StateX[r + 1][col + 4][1]);
				// Row 2
				_2XOR_probabilistic(model, StateW[r][col + 4][0], StateW[r][col + 4][1], StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 8][0], StateX[r + 1][col + 8][1], Xd_Not);
				// Row 3
				_2XOR_probabilistic(model, StateW[r][col][0], StateW[r][col][1], StateW[r][col + 8][0], StateW[r][col + 8][1], StateX[r + 1][col + 12][0], StateX[r + 1][col + 12][1], Xd_Not);
			}

			// Determine the Cell-Condition
			{
				// Xd=0 & ∑ Xs1<=3 & Ws1=1 => Wc=1 
				GRBVar Xd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Xd_Not == 1 - StateX[r + 1][col][2]);

				// Row 0
				model.addConstr(StateW[r][col][3] == 0);

				// Row 1
				{
					// s1: 0 0 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 8][1] + StateX[r + 1][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 8][1] + StateX[r + 1][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[3] = { Xd_Not,MC_Ind0, StateW[r][col + 4][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 3);
					model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col + 4][3] == 1);
				}
				{
					// s1: 1 0 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateX[r + 1][col + 8][1] + StateX[r + 1][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateX[r + 1][col + 8][1] + StateX[r + 1][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Xd_Not,MC_Ind0,StateX[r + 1][col + 4][1],StateW[r][col + 4][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col + 4][3] == 1);
				}
				{
					// s1: 0 0 1 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 8][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 8][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Xd_Not,MC_Ind0,StateX[r + 1][col + 12][1],StateW[r][col + 4][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col + 4][3] == 1);
				}
				{
					// s1: 0 1 0 -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind0, 0, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 12][1] >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 12][1] == 0);
					GRBVar MC_Temp_AND0[4] = { Xd_Not,MC_Ind0,StateX[r + 1][col + 8][1],StateW[r][col + 4][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 4);
					GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col + 4][3] + StateW[r][col + 8][3] == 1);
				}
				{
					// s1: 0 1 1 / 101 / 110 / -> 1
					GRBVar MC_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					GRBLinExpr Xs1_Sum = StateX[r + 1][col + 4][1] + StateX[r + 1][col + 8][1] + StateX[r + 1][col + 12][1];
					GRBVar MC_Temp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
					model.addConstr(MC_Temp0 == Xs1_Sum - 2);
					GRBVar MC_Temp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
					model.addGenConstrAbs(MC_Temp1, MC_Temp0);
					model.addGenConstrIndicator(MC_Ind0, 0, MC_Temp1 >= 1);
					model.addGenConstrIndicator(MC_Ind0, 1, Xs1_Sum == 2);
					GRBVar MC_Temp_AND0[3] = { Xd_Not,MC_Ind0,StateW[r][col + 4][1] };
					GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					model.addGenConstrAnd(MC_Ind1, MC_Temp_AND0, 3);
					model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col + 4][3] == 1);
				}

				// Row 2
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 12][1] == 2);
				model.addGenConstrIndicator(MC_Ind2, 1, StateX[r + 1][col + 4][1] + StateX[r + 1][col + 12][1] <= 1);
				GRBVar MC_Temp_AND2[3] = { Xd_Not,MC_Ind2,StateW[r][col + 8][1] };
				model.addGenConstrAnd(StateW[r][col + 8][3], MC_Temp_AND2, 3);

				// Row 3
				GRBVar MC_Ind3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind3, 0, StateX[r + 1][col + 0][1] + StateX[r + 1][col + 12][1] == 2);
				model.addGenConstrIndicator(MC_Ind3, 1, StateX[r + 1][col + 0][1] + StateX[r + 1][col + 12][1] <= 1);
				GRBVar MC_Temp_AND3[3] = { Xd_Not,MC_Ind3,StateW[r][col + 12][1] };
				model.addGenConstrAnd(StateW[r][col + 12][3], MC_Temp_AND3, 3);
			}

			model.addGenConstrIndicator(StateW[r][col + 0][3], 0, StateW[r][col + 0][4] == 0);
			model.addGenConstrIndicator(StateW[r][col + 0][4], 1, StateW[r][col + 0][3] == 1);
			model.addGenConstrIndicator(StateW[r][col + 4][3], 0, StateW[r][col + 4][4] == 0);
			model.addGenConstrIndicator(StateW[r][col + 4][4], 1, StateW[r][col + 4][3] == 1);
			model.addGenConstrIndicator(StateW[r][col + 8][3], 0, StateW[r][col + 8][4] == 0);
			model.addGenConstrIndicator(StateW[r][col + 8][4], 1, StateW[r][col + 8][3] == 1);
			model.addGenConstrIndicator(StateW[r][col + 12][3], 0, StateW[r][col + 12][4] == 0);
			model.addGenConstrIndicator(StateW[r][col + 12][4], 1, StateW[r][col + 12][3] == 1);

		}
	}
}


// Data Complexity D = 2 ^ {n + c_B + c_f - △B - ▽F + LG(g) / 2 + 1};
// c_B + c_F - △B - △F < 2c (c: cell size)
void BeyondFullCodebook(GRBModel& model, vector<vector<GRBVar>>& Up_plaintext, vector<vector<GRBVar>>& Lo_ciphertext, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Up_StateY, vector<vector<vector<GRBVar>>>& Lo_StateX, vector<vector<vector<GRBVar>>>& Lo_StateW) {
	GRBLinExpr rB = 0;
	GRBLinExpr rF = 0;
	GRBLinExpr cB = 0;
	GRBLinExpr cF = 0;
	for (int i = 0; i < state; i++) {
		rB += (1 - Up_StateX[1][i][1]);
		rF += (1 - Lo_StateW[ROUND - 1][i][1]);
	}
	for (int r = 1; r < midRound; r++) {
		for (int i = 0; i < state; i++) {
			cB += Up_StateY[r][i][3];
		}
	}
	for (int r = 2; r < midRound; r++) {
		for (int i = 0; i < state; i++) {
			cB += Up_StateX[r][i][3];
		}
	}

	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			cF += Lo_StateX[r][i][3];
		}
	}
	for (int r = midRound; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			cF += Lo_StateW[r][i][3];
		}
	}

	model.addConstr(cB + cF <= rB + rF - 1);// +2);
	//model.addConstr(cF <= rF-1);
	//model.addConstr(cB == rB );
}


void KeyBridge(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_Roundkey, vector<vector<vector<GRBVar>>>& Lo_Roundkey, GRBQuadExpr& key_involved_independent, vector<GRBVar>& independent) {
	// k=1 in ROUND rounds for upKey and loKey：0,1,2,ROUND
	int pp[16] = { 8,9,10,11,12,13,14,15,2,0,4,7,6,3,5,1 };
	/*for (int i = 0; i < state; i++) {
		GRBQuadExpr KeyBridge = 0;
		int j = i;
		for (int r = 0; r < ROUND; r++) {
			KeyBridge += Up_Roundkey[r][j][4] + Lo_Roundkey[r][j][4];
			j = pp[j];
		}
		GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(KeyBridge <= 2 * ROUND * e1 + (1 - e1));
		model.addConstr(KeyBridge >= 2 * ROUND * e1);
	}*/
	for (int i = 0; i < state; i++) {
		GRBLinExpr cnt = 0;
		int j = i;
		for (int r = 0; r < ROUND; r++) {
			if (r <= midRound) {
				cnt += Up_Roundkey[r][j][2];

			}
			else {
				cnt += Lo_Roundkey[r][j][2];

			}
			j = pp[j];
		}

		GRBVar Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Ind, 0, cnt <= 1);
		model.addGenConstrIndicator(Ind, 1, cnt >= 2);

		independent[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(independent[i] == cnt);

		key_involved_independent += (Ind * 2 + (1 - Ind) * cnt);
	}
}

void InvolvedRoundkeys_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY) {

	GRBVar key_temp[midRound + 1][16];
	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 16; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}
	for (int r = 1; r < midRound + 1; r++) {
		// Use state_temp[] as a temp variable connecting X[r+1][3] and key[r-1][2].
		GRBVar state_temp[16];
		for (int i = 0; i < 16; i++) {
			state_temp[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}

		for (int col = 0; col < 4; col++) {
			// conditions for Y
			/*GRBVar Row0[3] = { key[r - 1][col][2],key[r - 1][8 + (col + 2) % 4][2],key[r - 1][12 + (col + 1) % 4][2] };
			GRBVar Row1[1] = { key[r - 1][col][2] };
			GRBVar Row2[2] = { key[r - 1][4 + (col + 3) % 4][2], key[r - 1][8 + (col + 2) % 4][2] };
			GRBVar Row3[2] = { key[r - 1][col][2], key[r - 1][8 + (col + 2) % 4][2] };*/

			GRBVar Row0[3] = { key_temp[r - 1][col],key_temp[r - 1][8 + (col + 2) % 4],key_temp[r - 1][12 + (col + 1) % 4] };
			GRBVar Row1[1] = { key_temp[r - 1][col] };
			GRBVar Row2[2] = { key_temp[r - 1][4 + (col + 3) % 4], key_temp[r - 1][8 + (col + 2) % 4] };
			GRBVar Row3[2] = { key_temp[r - 1][col],key_temp[r - 1][8 + (col + 2) % 4] };


			GRBVar Key_Ind[4];
			Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrAnd(Key_Ind[0], Row0, 3);
			model.addGenConstrAnd(Key_Ind[1], Row1, 1);
			model.addGenConstrAnd(Key_Ind[2], Row2, 2);
			model.addGenConstrAnd(Key_Ind[3], Row3, 2);

			model.addGenConstrIndicator(StateY[r][col][3], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(StateY[r][col + 4][3], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(StateY[r][col + 8][3], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(StateY[r][col + 12][3], 1, Key_Ind[3] == 1);


			// conditions for X
			// Use state_temp as a temp variable connecting X[r+1][3] and key[r-1][2].
			// Row 0
			GRBVar Row0_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row0_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row0_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Ys1_Not0 == 1 - StateY[r][col][1]);
			model.addConstr(Ys1_Not1 == 1 - StateY[r][8 + (col + 2) % 4][1]);
			model.addConstr(Ys1_Not2 == 1 - StateY[r][12 + (col + 1) % 4][1]);
			GRBVar Row0_And0[2] = { StateX[r + 1][col][3],Ys1_Not0 };
			GRBVar Row0_And1[2] = { StateX[r + 1][col][3],Ys1_Not1 };
			GRBVar Row0_And2[2] = { StateX[r + 1][col][3],Ys1_Not2 };
			model.addGenConstrAnd(Row0_Ind0, Row0_And0, 2);
			model.addGenConstrAnd(Row0_Ind1, Row0_And1, 2);
			model.addGenConstrAnd(Row0_Ind2, Row0_And2, 2);
			model.addGenConstrIndicator(Row0_Ind0, 1, state_temp[col] == 1);
			model.addGenConstrIndicator(Row0_Ind1, 1, state_temp[8 + (col + 2) % 4] == 1);
			model.addGenConstrIndicator(Row0_Ind2, 1, state_temp[12 + (col + 1) % 4] == 1);
			// Row 2
			model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[4 + (col + 3) % 4] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[8 + (col + 2) % 4] == 1);
			// Row 3
			model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[col] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[8 + (col + 2) % 4] == 1);


			// from state_temp to key[r-1][2]
			model.addGenConstrIndicator(state_temp[col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(state_temp[col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(state_temp[col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(state_temp[col + 12], 1, Key_Ind[3] == 1);

			// from key[r][2] to key[r-1][2]
			/*model.addGenConstrIndicator(key[r][col][2], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key[r][col + 4][2], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key[r][col + 8][2], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key[r][col + 12][2], 1, Key_Ind[3] == 1);*/
			model.addGenConstrIndicator(key_temp[r][col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 12], 1, Key_Ind[3] == 1);

		}
	}

	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 8; i++) {
			model.addConstr(key_temp[r][i] == key[r][i][2]);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(0 == key[r][i][2]);
		}
	}



	//// Revised
	//for (int r = 1; r < midRound + 1; r++) {
	//	// Use state_temp[] as a temp variable connecting X[r+1][3] and key[r-1][2].
	//	GRBVar state_temp[16];
	//	for (int i = 0; i < 16; i++) {
	//		state_temp[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//	}

	//	for (int col = 0; col < 4; col++) {
	//		// conditions for Y
	//		GRBVar Row0[3] = { key[r - 1][col][2],key[r - 1][8 + (col + 2) % 4][2],key[r - 1][12 + (col + 1) % 4][2] };
	//		GRBVar Row1[1] = { key[r - 1][col][2] };
	//		GRBVar Row2[2] = { key[r - 1][4 + (col + 3) % 4][2], key[r - 1][8 + (col + 2) % 4][2] };
	//		GRBVar Row3[2] = { key[r - 1][col][2], key[r - 1][8 + (col + 2) % 4][2] };


	//		GRBVar Key_Ind[4];
	//		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		model.addGenConstrAnd(Key_Ind[0], Row0, 3);
	//		model.addGenConstrAnd(Key_Ind[1], Row1, 1);
	//		model.addGenConstrAnd(Key_Ind[2], Row2, 2);
	//		model.addGenConstrAnd(Key_Ind[3], Row3, 2);

	//		model.addGenConstrIndicator(StateY[r][col][3], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 4][3], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 8][3], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 12][3], 1, Key_Ind[3] == 1);


	//		// conditions for X
	//		// Use state_temp as a temp variable connecting X[r+1][3] and key[r-1][2].
	//		// Row 0
	//		GRBVar Row0_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Row0_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Row0_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		model.addConstr(Ys1_Not0 == 1 - StateY[r][col][1]);
	//		model.addConstr(Ys1_Not1 == 1 - StateY[r][8 + (col + 2) % 4][1]);
	//		model.addConstr(Ys1_Not2 == 1 - StateY[r][12 + (col + 1) % 4][1]);
	//		GRBVar Row0_And0[2] = { StateX[r + 1][col][3],Ys1_Not0 };
	//		GRBVar Row0_And1[2] = { StateX[r + 1][col][3],Ys1_Not1 };
	//		GRBVar Row0_And2[2] = { StateX[r + 1][col][3],Ys1_Not2 };
	//		model.addGenConstrAnd(Row0_Ind0, Row0_And0, 2);
	//		model.addGenConstrAnd(Row0_Ind1, Row0_And1, 2);
	//		model.addGenConstrAnd(Row0_Ind2, Row0_And2, 2);
	//		model.addGenConstrIndicator(Row0_Ind0, 1, state_temp[col] == 1);
	//		model.addGenConstrIndicator(Row0_Ind1, 1, state_temp[8 + (col + 2) % 4] == 1);
	//		model.addGenConstrIndicator(Row0_Ind2, 1, state_temp[12 + (col + 1) % 4] == 1);
	//		// Row 2
	//		model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[4 + (col + 3) % 4] == 1);
	//		model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[8 + (col + 2) % 4] == 1);
	//		// Row 3
	//		model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[col] == 1);
	//		model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[8 + (col + 2) % 4] == 1);
	//		
	//		
	//		// from state_temp to key[r-1][2]
	//		model.addGenConstrIndicator(state_temp[col], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 4], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 8], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 12], 1, Key_Ind[3] == 1);

	//		// from key[r][2] to key[r-1][2]
	//		model.addGenConstrIndicator(key[r][col][2], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(key[r][col + 4][2], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(key[r][col + 8][2], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(key[r][col + 12][2], 1, Key_Ind[3] == 1);

	//	}
	//}
}

void InvolvedRoundkeys_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateW) {

	GRBVar key_temp[ROUND][16];
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < 16; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}

	for (int r = midRound + 3; r < ROUND; r++) {

		for (int i = 0; i < 16; i++) {
			// conditions for X
			/*model.addGenConstrIndicator(StateX[r][i][3], 1, key[r][i][2] == 1);*/
			model.addGenConstrIndicator(StateX[r][i][3], 1, key_temp[r][i] == 1);
		}

		for (int col = 0; col < 4; col++) {
			/*GRBVar Row0[1] = { key[r][col + 4][2] };
			GRBVar Row1[3] = { key[r][4 + (col + 1) % 4][2], key[r][8 + (col + 1) % 4][2], key[r][12 + (col + 1) % 4][2] };
			GRBVar Row2[2] = { key[r][4 + (col + 2) % 4][2], key[r][12 + (col + 2) % 4][2] };
			GRBVar Row3[2] = { key[r][(col + 3) % 4][2], key[r][12 + (col + 3) % 4][2] };*/
			GRBVar Row0[1] = { key_temp[r][col + 4] };
			GRBVar Row1[3] = { key_temp[r][4 + (col + 1) % 4], key_temp[r][8 + (col + 1) % 4], key_temp[r][12 + (col + 1) % 4] };
			GRBVar Row2[2] = { key_temp[r][4 + (col + 2) % 4], key_temp[r][12 + (col + 2) % 4] };
			GRBVar Row3[2] = { key_temp[r][(col + 3) % 4], key_temp[r][12 + (col + 3) % 4] };

			GRBVar Key_Ind[4];
			Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrAnd(Key_Ind[0], Row0, 1);
			model.addGenConstrAnd(Key_Ind[1], Row1, 3);
			model.addGenConstrAnd(Key_Ind[2], Row2, 2);
			model.addGenConstrAnd(Key_Ind[3], Row3, 2);


			// conditions for W
			// Row 1
			GRBVar Row1_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row1_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row1_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xs1_Not0 == 1 - StateX[r][4 + col][1]);
			model.addConstr(Xs1_Not1 == 1 - StateX[r][8 + col][1]);
			model.addConstr(Xs1_Not2 == 1 - StateX[r][12 + col][1]);
			GRBVar Row1_And0[2] = { StateW[r - 1][col + 4][3],Xs1_Not0 };
			GRBVar Row1_And1[2] = { StateW[r - 1][col + 4][3],Xs1_Not1 };
			GRBVar Row1_And2[2] = { StateW[r - 1][col + 4][3],Xs1_Not2 };
			model.addGenConstrAnd(Row1_Ind0, Row1_And0, 2);
			model.addGenConstrAnd(Row1_Ind1, Row1_And1, 2);
			model.addGenConstrAnd(Row1_Ind2, Row1_And2, 2);
			/*model.addGenConstrIndicator(Row1_Ind0, 1, key[r][col + 4][2] == 1);
			model.addGenConstrIndicator(Row1_Ind1, 1, key[r][col + 8][2] == 1);
			model.addGenConstrIndicator(Row1_Ind2, 1, key[r][col + 12][2] == 1);*/
			model.addGenConstrIndicator(Row1_Ind0, 1, key_temp[r][col + 4] == 1);
			model.addGenConstrIndicator(Row1_Ind1, 1, key_temp[r][col + 8] == 1);
			model.addGenConstrIndicator(Row1_Ind2, 1, key_temp[r][col + 12] == 1);

			// Row 2
			/*model.addGenConstrIndicator(StateW[r - 1][col + 8][3], 1, key[r][col + 4][2] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 8][3], 1, key[r][col + 12][2] == 1);*/
			model.addGenConstrIndicator(StateW[r - 1][col + 8][3], 1, key_temp[r][col + 4] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 8][3], 1, key_temp[r][col + 12] == 1);
			// Row 3
			/*model.addGenConstrIndicator(StateW[r - 1][col + 12][3], 1, key[r][col][2] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 12][3], 1, key[r][col][2] == 1);*/
			model.addGenConstrIndicator(StateW[r - 1][col + 12][3], 1, key_temp[r][col] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 12][3], 1, key_temp[r][col] == 1);


			// from key[r][2] to key[r-1][2]
			/*model.addGenConstrIndicator(key[r-1][col][2], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key[r-1][col + 4][2], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key[r-1][col + 8][2], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key[r-1][col + 12][2], 1, Key_Ind[3] == 1);*/
			model.addGenConstrIndicator(key_temp[r - 1][col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 12], 1, Key_Ind[3] == 1);

		}
	}

	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < 8; i++) {
			model.addConstr(key_temp[r][i] == key[r][i][2]);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(0 == key[r][i][2]);
		}
	}
}


void KeyRecovery_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY) {

	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}
	GRBVar key_temp[midRound + 1][16];
	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 16; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}
	for (int r = 1; r < midRound + 1; r++) {
		// Use state_temp[] as a temp variable connecting X[r+1][4] and key[r-1][3].
		GRBVar state_temp[16];
		for (int i = 0; i < 16; i++) {
			state_temp[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}

		for (int col = 0; col < 4; col++) {
			// conditions for Y

			GRBVar Row0[3] = { key_temp[r - 1][col],key_temp[r - 1][8 + (col + 2) % 4],key_temp[r - 1][12 + (col + 1) % 4] };
			GRBVar Row1[1] = { key_temp[r - 1][col] };
			GRBVar Row2[2] = { key_temp[r - 1][4 + (col + 3) % 4], key_temp[r - 1][8 + (col + 2) % 4] };
			GRBVar Row3[2] = { key_temp[r - 1][col],key_temp[r - 1][8 + (col + 2) % 4] };


			GRBVar Key_Ind[4];
			Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrAnd(Key_Ind[0], Row0, 3);
			model.addGenConstrAnd(Key_Ind[1], Row1, 1);
			model.addGenConstrAnd(Key_Ind[2], Row2, 2);
			model.addGenConstrAnd(Key_Ind[3], Row3, 2);

			model.addGenConstrIndicator(StateY[r][col][4], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(StateY[r][col + 4][4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(StateY[r][col + 8][4], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(StateY[r][col + 12][4], 1, Key_Ind[3] == 1);


			// conditions for X
			// Use state_temp as a temp variable connecting X[r+1][3] and key[r-1][2].
			// Row 0
			GRBVar Row0_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row0_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row0_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Ys1_Not0 == 1 - StateY[r][col][1]);
			model.addConstr(Ys1_Not1 == 1 - StateY[r][8 + (col + 2) % 4][1]);
			model.addConstr(Ys1_Not2 == 1 - StateY[r][12 + (col + 1) % 4][1]);
			GRBVar Row0_And0[2] = { StateX[r + 1][col][4],Ys1_Not0 };
			GRBVar Row0_And1[2] = { StateX[r + 1][col][4],Ys1_Not1 };
			GRBVar Row0_And2[2] = { StateX[r + 1][col][4],Ys1_Not2 };
			model.addGenConstrAnd(Row0_Ind0, Row0_And0, 2);
			model.addGenConstrAnd(Row0_Ind1, Row0_And1, 2);
			model.addGenConstrAnd(Row0_Ind2, Row0_And2, 2);
			model.addGenConstrIndicator(Row0_Ind0, 1, state_temp[col] == 1);
			model.addGenConstrIndicator(Row0_Ind1, 1, state_temp[8 + (col + 2) % 4] == 1);
			model.addGenConstrIndicator(Row0_Ind2, 1, state_temp[12 + (col + 1) % 4] == 1);
			// Row 2
			model.addGenConstrIndicator(StateX[r + 1][col + 8][4], 1, state_temp[4 + (col + 3) % 4] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 8][4], 1, state_temp[8 + (col + 2) % 4] == 1);
			// Row 3
			model.addGenConstrIndicator(StateX[r + 1][col + 12][4], 1, state_temp[col] == 1);
			model.addGenConstrIndicator(StateX[r + 1][col + 12][4], 1, state_temp[8 + (col + 2) % 4] == 1);


			// from state_temp to key[r-1][2]
			model.addGenConstrIndicator(state_temp[col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(state_temp[col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(state_temp[col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(state_temp[col + 12], 1, Key_Ind[3] == 1);

			// from key[r][2] to key[r-1][2]
			/*model.addGenConstrIndicator(key[r][col][3], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key[r][col + 4][3], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key[r][col + 8][3], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key[r][col + 12][3], 1, Key_Ind[3] == 1);*/
			model.addGenConstrIndicator(key_temp[r][col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key_temp[r][col + 12], 1, Key_Ind[3] == 1);

		}
	}

	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 8; i++) {
			model.addConstr(key_temp[r][i] == key[r][i][3]);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(0 == key[r][i][3]);
		}
	}



	//// Revised
	//for (int r = 1; r < midRound + 1; r++) {
	//	// Use state_temp[] as a temp variable connecting X[r+1][3] and key[r-1][2].
	//	GRBVar state_temp[16];
	//	for (int i = 0; i < 16; i++) {
	//		state_temp[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//	}

	//	for (int col = 0; col < 4; col++) {
	//		// conditions for Y
	//		GRBVar Row0[3] = { key[r - 1][col][2],key[r - 1][8 + (col + 2) % 4][2],key[r - 1][12 + (col + 1) % 4][2] };
	//		GRBVar Row1[1] = { key[r - 1][col][2] };
	//		GRBVar Row2[2] = { key[r - 1][4 + (col + 3) % 4][2], key[r - 1][8 + (col + 2) % 4][2] };
	//		GRBVar Row3[2] = { key[r - 1][col][2], key[r - 1][8 + (col + 2) % 4][2] };


	//		GRBVar Key_Ind[4];
	//		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		model.addGenConstrAnd(Key_Ind[0], Row0, 3);
	//		model.addGenConstrAnd(Key_Ind[1], Row1, 1);
	//		model.addGenConstrAnd(Key_Ind[2], Row2, 2);
	//		model.addGenConstrAnd(Key_Ind[3], Row3, 2);

	//		model.addGenConstrIndicator(StateY[r][col][3], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 4][3], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 8][3], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(StateY[r][col + 12][3], 1, Key_Ind[3] == 1);


	//		// conditions for X
	//		// Use state_temp as a temp variable connecting X[r+1][3] and key[r-1][2].
	//		// Row 0
	//		GRBVar Row0_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Row0_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Row0_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		GRBVar Ys1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//		model.addConstr(Ys1_Not0 == 1 - StateY[r][col][1]);
	//		model.addConstr(Ys1_Not1 == 1 - StateY[r][8 + (col + 2) % 4][1]);
	//		model.addConstr(Ys1_Not2 == 1 - StateY[r][12 + (col + 1) % 4][1]);
	//		GRBVar Row0_And0[2] = { StateX[r + 1][col][3],Ys1_Not0 };
	//		GRBVar Row0_And1[2] = { StateX[r + 1][col][3],Ys1_Not1 };
	//		GRBVar Row0_And2[2] = { StateX[r + 1][col][3],Ys1_Not2 };
	//		model.addGenConstrAnd(Row0_Ind0, Row0_And0, 2);
	//		model.addGenConstrAnd(Row0_Ind1, Row0_And1, 2);
	//		model.addGenConstrAnd(Row0_Ind2, Row0_And2, 2);
	//		model.addGenConstrIndicator(Row0_Ind0, 1, state_temp[col] == 1);
	//		model.addGenConstrIndicator(Row0_Ind1, 1, state_temp[8 + (col + 2) % 4] == 1);
	//		model.addGenConstrIndicator(Row0_Ind2, 1, state_temp[12 + (col + 1) % 4] == 1);
	//		// Row 2
	//		model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[4 + (col + 3) % 4] == 1);
	//		model.addGenConstrIndicator(StateX[r + 1][col + 8][3], 1, state_temp[8 + (col + 2) % 4] == 1);
	//		// Row 3
	//		model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[col] == 1);
	//		model.addGenConstrIndicator(StateX[r + 1][col + 12][3], 1, state_temp[8 + (col + 2) % 4] == 1);
	//		
	//		
	//		// from state_temp to key[r-1][2]
	//		model.addGenConstrIndicator(state_temp[col], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 4], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 8], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(state_temp[col + 12], 1, Key_Ind[3] == 1);

	//		// from key[r][2] to key[r-1][2]
	//		model.addGenConstrIndicator(key[r][col][2], 1, Key_Ind[0] == 1);
	//		model.addGenConstrIndicator(key[r][col + 4][2], 1, Key_Ind[1] == 1);
	//		model.addGenConstrIndicator(key[r][col + 8][2], 1, Key_Ind[2] == 1);
	//		model.addGenConstrIndicator(key[r][col + 12][2], 1, Key_Ind[3] == 1);

	//	}
	//}
}

void KeyRecovery_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateW) {

	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}

	GRBVar key_temp[ROUND][16];
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < 16; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}

	for (int r = midRound + 3; r < ROUND; r++) {

		for (int i = 0; i < 16; i++) {
			// conditions for X
			/*model.addGenConstrIndicator(StateX[r][i][4], 1, key[r][i][3] == 1);*/
			model.addGenConstrIndicator(StateX[r][i][4], 1, key_temp[r][i] == 1);
		}

		for (int col = 0; col < 4; col++) {
			GRBVar Row0[1] = { key_temp[r][col + 4] };
			GRBVar Row1[3] = { key_temp[r][4 + (col + 1) % 4], key_temp[r][8 + (col + 1) % 4], key_temp[r][12 + (col + 1) % 4] };
			GRBVar Row2[2] = { key_temp[r][4 + (col + 2) % 4], key_temp[r][12 + (col + 2) % 4] };
			GRBVar Row3[2] = { key_temp[r][(col + 3) % 4], key_temp[r][12 + (col + 3) % 4] };

			GRBVar Key_Ind[4];
			Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrAnd(Key_Ind[0], Row0, 1);
			model.addGenConstrAnd(Key_Ind[1], Row1, 3);
			model.addGenConstrAnd(Key_Ind[2], Row2, 2);
			model.addGenConstrAnd(Key_Ind[3], Row3, 2);


			// conditions for W
			// Row 1
			GRBVar Row1_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row1_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Row1_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xs1_Not0 == 1 - StateX[r][4 + col][1]);
			model.addConstr(Xs1_Not1 == 1 - StateX[r][8 + col][1]);
			model.addConstr(Xs1_Not2 == 1 - StateX[r][12 + col][1]);
			GRBVar Row1_And0[2] = { StateW[r - 1][col + 4][4],Xs1_Not0 };
			GRBVar Row1_And1[2] = { StateW[r - 1][col + 4][4],Xs1_Not1 };
			GRBVar Row1_And2[2] = { StateW[r - 1][col + 4][4],Xs1_Not2 };
			model.addGenConstrAnd(Row1_Ind0, Row1_And0, 2);
			model.addGenConstrAnd(Row1_Ind1, Row1_And1, 2);
			model.addGenConstrAnd(Row1_Ind2, Row1_And2, 2);

			model.addGenConstrIndicator(Row1_Ind0, 1, key_temp[r][col + 4] == 1);
			model.addGenConstrIndicator(Row1_Ind1, 1, key_temp[r][col + 8] == 1);
			model.addGenConstrIndicator(Row1_Ind2, 1, key_temp[r][col + 12] == 1);

			// Row 2
			/*model.addGenConstrIndicator(StateW[r - 1][col + 8][4], 1, key[r][col + 4][3] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 8][4], 1, key[r][col + 12][3] == 1);*/
			model.addGenConstrIndicator(StateW[r - 1][col + 8][4], 1, key_temp[r][col + 4] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 8][4], 1, key_temp[r][col + 12] == 1);
			// Row 3
			/*model.addGenConstrIndicator(StateW[r - 1][col + 12][4], 1, key[r][col][3] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 12][4], 1, key[r][col][3] == 1);*/
			model.addGenConstrIndicator(StateW[r - 1][col + 12][4], 1, key_temp[r][col] == 1);
			model.addGenConstrIndicator(StateW[r - 1][col + 12][4], 1, key_temp[r][col] == 1);


			// from key[r][2] to key[r-1][2]
			model.addGenConstrIndicator(key_temp[r - 1][col], 1, Key_Ind[0] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 4], 1, Key_Ind[1] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 8], 1, Key_Ind[2] == 1);
			model.addGenConstrIndicator(key_temp[r - 1][col + 12], 1, Key_Ind[3] == 1);

		}
	}

	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < 8; i++) {
			model.addConstr(key_temp[r][i] == key[r][i][3]);
		}
		for (int i = 8; i < 16; i++) {
			model.addConstr(0 == key[r][i][3]);
		}
	}
}

void KeyRecovery(GRBModel& model, vector<vector<vector<GRBVar>>>& upkey, vector<vector<vector<GRBVar>>>& lokey) {
	GRBLinExpr KR_count = 0;
	for (int r = 0; r < midRound; r++) {
		for (int i = 0; i < 8; i++) {
			KR_count += upkey[r][i][2];
		}
	}
	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < 8; i++) {
			KR_count += lokey[r][i][2];
		}
	}
	model.addConstr(KR_count <= 45);
}

void Em_Construct_DDT(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY) {
	GRBLinExpr Em_1r = 0;
	GRBVar midround_value[16];
	for (int i = 0; i < state; i++) {
		midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
		model.addConstr(Xs1_Not == 1 - Up_StateX[midRound][i][1]);
		GRBVar MidTemp_AND[4] = { Xs0_Not,Xs1_Not,Lo_StateY[midRound][i][0],Lo_StateY[midRound][i][1] };
		model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
		Em_1r += midround_value[i];
	}
	model.addConstr(Em_1r >= 1);
}

void Em_Construct_All(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY, vector<vector<GRBVar>>& cond) {

	// 1-round Em 
	// with DDT({01},{01})=0
	{
		GRBLinExpr Em_1r = 0;
		GRBVar midround_value[16];
		for (int i = 0; i < state; i++) {
			midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
			model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound][i][0]);
			GRBVar MidTemp_AND[4] = { Xs0_Not,Up_StateX[midRound][i][1],Ys0_Not,Lo_StateY[midRound][i][1] };
			model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
			Em_1r += midround_value[i];
		}
		cond[0][0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(cond[0][0], 0, Em_1r == 0);
		model.addGenConstrIndicator(cond[0][0], 1, Em_1r >= 1);
	}

	// with DDT({00},{11})=0
	{
		GRBLinExpr Em_1r = 0;
		GRBVar midround_value[16];
		for (int i = 0; i < state; i++) {
			midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
			model.addConstr(Xs1_Not == 1 - Up_StateX[midRound][i][1]);
			GRBVar MidTemp_AND[4] = { Xs0_Not,Xs1_Not,Lo_StateY[midRound][i][0],Lo_StateY[midRound][i][1] };
			model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
			Em_1r += midround_value[i];
		}
		cond[0][1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(cond[0][1], 0, Em_1r == 0);
		model.addGenConstrIndicator(cond[0][1], 1, Em_1r >= 1);
	}

	// with DDT({11},{00})=0
	{
		GRBLinExpr Em_1r = 0;
		GRBVar midround_value[16];
		for (int i = 0; i < state; i++) {
			midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound][i][0]);
			model.addConstr(Ys1_Not == 1 - Lo_StateY[midRound][i][1]);
			GRBVar MidTemp_AND[4] = { Up_StateX[midRound][i][0],Up_StateX[midRound][i][1],Ys0_Not,Ys1_Not };
			model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
			Em_1r += midround_value[i];
		}
		cond[0][2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(cond[0][2], 0, Em_1r == 0);
		model.addGenConstrIndicator(cond[0][2], 1, Em_1r >= 1);
	}

	// with DDT({01},{11})=0
	{
		GRBLinExpr Em_1r = 0;
		GRBVar midround_value[16];
		for (int i = 0; i < state; i++) {
			midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
			GRBVar MidTemp_AND[4] = { Xs0_Not,Up_StateX[midRound][i][1],Lo_StateY[midRound][i][0],Lo_StateY[midRound][i][1] };
			model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
			Em_1r += midround_value[i];
		}
		cond[1][0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(cond[1][0], 0, Em_1r == 0);
		model.addGenConstrIndicator(cond[1][0], 1, Em_1r >= 1);
	}

	// with DDT({11},{01})=0
	{
		GRBLinExpr Em_1r = 0;
		GRBVar midround_value[16];
		for (int i = 0; i < state; i++) {
			midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound][i][0]);
			GRBVar MidTemp_AND[4] = { Ys0_Not,Lo_StateY[midRound][i][1],Up_StateX[midRound][i][0],Up_StateX[midRound][i][1] };
			model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
			Em_1r += midround_value[i];
		}
		cond[1][1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(cond[1][1], 0, Em_1r == 0);
		model.addGenConstrIndicator(cond[1][1], 1, Em_1r >= 1);
	}

	// 2-round Em 

	// with [DDT+DDT]
	// x —SB—> ? —ART—> ? —SR—> ? —MC—> 
	// ? —SB—> y    
	// Row 1 of MixColumns: X_r[i,8+(i+2)%4,12+(i+1)%4]\in{11,11,01}, Y_r+1[i]=01, i\in{0,1,2,3}
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][i][0] + Up_StateX[midRound][8 + (i + 2) % 4][0] + Up_StateX[midRound][12 + (i + 1) % 4][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][i][1] + Up_StateX[midRound][8 + (i + 2) % 4][1] + Up_StateX[midRound][12 + (i + 1) % 4][1];
		GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound + 1][i][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Xs0_Sum - 2);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Xs0_Sum == 2);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Xs1_Sum == 3);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Xs1_Sum <= 2);

		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Ys0_Not,Lo_StateY[midRound + 1][i][1] };
		cond[2][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[2][i], Em2r_AND, 4);
	}
	// Row 2 of M & Row 1 of M^{-1}: X_r[i]=01, Y_r+1[i+4]=01, i\in{0,1,2,3}
	for (int i = 0; i < 4; i++) {
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
		model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound + 1][i + 4][0]);
		GRBVar Em2r_AND[4] = { Xs0_Not,Up_StateX[midRound][i][1],Ys0_Not, Lo_StateY[midRound + 1][i + 4][1] };
		cond[3][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[3][i], Em2r_AND, 4);
	}
	// Row 3 of M: 
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][i + 4][0] + Up_StateX[midRound][8 + (i + 3) % 4][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][i + 4][1] + Up_StateX[midRound][8 + (i + 3) % 4][1];
		GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound + 1][8 + (i + 1) % 4][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Xs0_Sum - 1);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Xs0_Sum == 1);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Xs1_Sum == 2);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Xs1_Sum <= 1);
		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Ys0_Not,Lo_StateY[midRound + 1][8 + (i + 1) % 4][1] };
		cond[4][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[4][i], Em2r_AND, 4);
	}
	// Row 4 of M: 
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][i][0] + Up_StateX[midRound][8 + (i + 2) % 4][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][i][1] + Up_StateX[midRound][8 + (i + 2) % 4][1];
		GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound + 1][i + 12][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Xs0_Sum - 1);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Xs0_Sum == 1);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Xs1_Sum == 2);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Xs1_Sum <= 1);
		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Ys0_Not,Lo_StateY[midRound + 1][i + 12][1] };
		cond[5][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[5][i], Em2r_AND, 4);
	}

	// with [DDT+DDT]
	// Row 2 of M^{-1}
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][i + 4][0] + Lo_StateY[midRound + 1][i + 8][0] + Lo_StateY[midRound + 1][i + 12][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][i + 4][1] + Lo_StateY[midRound + 1][i + 8][1] + Lo_StateY[midRound + 1][i + 12][1];
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][4 + (i + 3) % 4][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Ys0_Sum - 2);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Ys0_Sum == 2);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Ys1_Sum == 3);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Ys1_Sum <= 2);
		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Xs0_Not,Up_StateX[midRound][4 + (i + 3) % 4][1] };
		cond[6][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[6][i], Em2r_AND, 4);
	}
	// Row 3 of M^{-1}
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][i + 4][0] + Lo_StateY[midRound + 1][i + 12][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][i + 4][1] + Lo_StateY[midRound + 1][i + 12][1];
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][8 + (i + 2) % 4][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Ys0_Sum - 1);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Ys0_Sum == 1);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Ys1_Sum == 2);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Ys1_Sum <= 1);
		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Xs0_Not,Up_StateX[midRound][8 + (i + 2) % 4][1] };
		cond[7][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[7][i], Em2r_AND, 4);
	}
	// Row 4 of M^{-1}
	for (int i = 0; i < 4; i++) {
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][i][0] + Lo_StateY[midRound + 1][i + 12][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][i][1] + Lo_StateY[midRound + 1][i + 12][1];
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][12 + (i + 1) % 4][0]);
		GRBVar Em2r_Tmp0 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(Em2r_Tmp0 == Ys0_Sum - 1);
		GRBVar Em2r_Tmp1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addGenConstrAbs(Em2r_Tmp1, Em2r_Tmp0);
		GRBVar Em2r_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind0, 0, Em2r_Tmp1 >= 1);
		model.addGenConstrIndicator(Em2r_Ind0, 1, Ys0_Sum == 1);
		GRBVar Em2r_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Em2r_Ind1, 1, Ys1_Sum == 2);
		model.addGenConstrIndicator(Em2r_Ind1, 0, Ys1_Sum <= 1);
		GRBVar Em2r_AND[4] = { Em2r_Ind0,Em2r_Ind1,Xs0_Not,Up_StateX[midRound][12 + (i + 1) % 4][1] };
		cond[8][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(cond[8][i], Em2r_AND, 4);
	}


	GRBVar cc[33] = {
		cond[0][0],	cond[0][1],cond[0][2],
		cond[1][0], cond[1][1],
		cond[2][0], cond[2][1], cond[2][2], cond[2][3],
		cond[3][0], cond[3][1], cond[3][2], cond[3][3],
		cond[4][0], cond[4][1], cond[4][2], cond[4][3],
		cond[5][0], cond[5][1], cond[5][2], cond[5][3],
		cond[6][0], cond[6][1], cond[6][2], cond[6][3],
		cond[7][0], cond[7][1], cond[7][2], cond[7][3],
		cond[8][0], cond[8][1], cond[8][2], cond[8][3]
	};


	GRBVar c = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	model.addGenConstrOr(c, cc, 33);
	model.addConstr(c == 1);
}


void Output_Print(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_Roundkey, vector<vector<vector<GRBVar>>>& Lo_Roundkey, vector<vector<GRBVar>>& Up_plaintext, vector<vector<GRBVar>>& Up_ciphertext, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Up_StateY, vector<vector<vector<GRBVar>>>& Up_StateZ, vector<vector<vector<GRBVar>>>& Up_StateW, vector<vector<GRBVar>>& Lo_plaintext, vector<vector<GRBVar>>& Lo_ciphertext, vector<vector<vector<GRBVar>>>& Lo_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY, vector<vector<vector<GRBVar>>>& Lo_StateZ, vector<vector<vector<GRBVar>>>& Lo_StateW, vector<vector<GRBVar>>& cond, vector<GRBVar>& independent, GRBLinExpr obj1, GRBLinExpr obj2, GRBLinExpr obj3, GRBQuadExpr obj4) {
	cout << "P/C D_Pattern: " << obj1.getValue() << endl;
	cout << "Condition: " << obj2.getValue() << endl;
	cout << "keys: " << obj3.getValue() << endl;
	cout << "Independent: " << obj4.getValue() << endl << endl;

	cout << "Contradiction:" << endl;
	cout << cond[0][0].get(GRB_DoubleAttr_X) << ' ' << cond[0][1].get(GRB_DoubleAttr_X) << ' ' << cond[0][2].get(GRB_DoubleAttr_X) << endl;
	cout << cond[1][0].get(GRB_DoubleAttr_X) << ' ' << cond[1][1].get(GRB_DoubleAttr_X) << endl;
	for (int i = 2; i < 9; i++) {
		cout << cond[i][0].get(GRB_DoubleAttr_X) << ' ' << cond[i][1].get(GRB_DoubleAttr_X) << ' ' << cond[i][2].get(GRB_DoubleAttr_X) << ' ' << cond[i][3].get(GRB_DoubleAttr_X) << endl;
	}
	cout << endl;

	cout << "Keys Independent:" << endl;
	for (int i = 0; i < state; i++) {
		cout << independent[i].get(GRB_DoubleAttr_X) << ' ';
		if ((i + 1) % 4 == 0) cout << endl;
	}
	cout << endl;

	cout << "Upper Trail:" << endl;
	cout << "        P" << endl;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << Up_plaintext[4 * i + j][0].get(GRB_DoubleAttr_X) << Up_plaintext[4 * i + j][1].get(GRB_DoubleAttr_X) << Up_plaintext[4 * i + j][2].get(GRB_DoubleAttr_X) << Up_plaintext[4 * i + j][3].get(GRB_DoubleAttr_X) << Up_plaintext[4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
		}
	}
	cout << endl;

	for (int r = 0; r < ROUND; r++) {
		cout << r << " K" << endl;

		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < 4; j++) {
				cout << Up_Roundkey[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Up_StateX[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_StateX[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_StateX[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_StateX[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_StateX[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl << "SB" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Up_StateY[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_StateY[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_StateY[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_StateY[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_StateY[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl << "ART" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Up_StateZ[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_StateZ[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_StateZ[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_StateZ[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_StateZ[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl << "SR" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Up_StateW[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_StateW[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_StateW[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_StateW[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_StateW[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "MC" << endl << endl;
	}


	cout << "Lower Trail :" << endl;

	for (int r = 0; r < ROUND; r++) {
		cout << r << " K" << endl;
		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < 4; j++) {
				cout << Up_Roundkey[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Up_Roundkey[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Lo_StateX[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Lo_StateX[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Lo_StateX[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Lo_StateX[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Lo_StateX[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "SB" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Lo_StateY[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Lo_StateY[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Lo_StateY[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Lo_StateY[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Lo_StateY[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "ART" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				cout << Lo_StateZ[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Lo_StateZ[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Lo_StateZ[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Lo_StateZ[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Lo_StateZ[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "SR" << endl;

		if (r != ROUND - 1) {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					cout << Lo_StateW[r][4 * i + j][0].get(GRB_DoubleAttr_X) << Lo_StateW[r][4 * i + j][1].get(GRB_DoubleAttr_X) << Lo_StateW[r][4 * i + j][2].get(GRB_DoubleAttr_X) << Lo_StateW[r][4 * i + j][3].get(GRB_DoubleAttr_X) << Lo_StateW[r][4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "MC" << endl << endl;
		}
	}

	cout << " C" << endl;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << Lo_ciphertext[4 * i + j][0].get(GRB_DoubleAttr_X) << Lo_ciphertext[4 * i + j][1].get(GRB_DoubleAttr_X) << Lo_ciphertext[4 * i + j][2].get(GRB_DoubleAttr_X) << Lo_ciphertext[4 * i + j][3].get(GRB_DoubleAttr_X) << Lo_ciphertext[4 * i + j][4].get(GRB_DoubleAttr_X) << ' ';
		}
	}
	cout << endl;
}

int main() {
	try {
		GRBEnv env = GRBEnv(true);
		env.set("LogFile", "mip1.log");
		env.start();
		GRBModel model = GRBModel(env);

		// ImPossible Boomerang - 4 Differential Trails with Probability 1: Upper0, Upper1, Lower0, Lower1
		// Round Function (r-th round): w_r-1 ARK(r) x_r SB y_r SR z_r MC w_r ARK(r+1) x_r+1 
		// Every Round include 4 states: x y z w, each of which with 5 attributes: [state0 / state1 / distinguisher / cell-condition / rb] 
		vector<vector<GRBVar>> Up_plaintext(state, vector<GRBVar>(5));
		vector<vector<GRBVar>> Up_ciphertext(state, vector<GRBVar>(5));
		vector<vector<GRBVar>> Lo_plaintext(state, vector<GRBVar>(5));
		vector<vector<GRBVar>> Lo_ciphertext(state, vector<GRBVar>(5));

		// State Variable Declaration
		vector<vector<vector<GRBVar>>> Up_StateX(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Up_StateY(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Up_StateZ(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Up_StateW(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		internalState_U(model, Up_plaintext, Up_ciphertext, Up_StateX, Up_StateY, Up_StateZ, Up_StateW);

		vector<vector<vector<GRBVar>>> Lo_StateX(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Lo_StateY(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Lo_StateZ(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		vector<vector<vector<GRBVar>>> Lo_StateW(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		internalState_L(model, Lo_plaintext, Lo_ciphertext, Lo_StateX, Lo_StateY, Lo_StateZ, Lo_StateW);

		//// SubKey (256bit) state: represented by [round][state * 2][z/k]
		//// RoundKey (128bit) derived from Subkey:  represented by [round][state][z/k]
		//// Two Related-Key: upKey, loKey
		vector<vector<vector<GRBVar>>> Up_Roundkey(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		keySchedule(model, Up_Roundkey);
		vector<vector<vector<GRBVar>>> Lo_Roundkey(ROUND, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		keySchedule(model, Lo_Roundkey);

		//// Upper Trail Up0&Up1: Rk Encryption
		SubBytes_U(model, Up_StateX, Up_StateY);
		AddRoundKey_U(model, Up_Roundkey, Up_StateY, Up_StateZ);
		ShiftRow_U(model, Up_StateZ, Up_StateW);
		MixColumns_U(model, Up_StateW, Up_StateX);

		//Lower Trail Lo0&Lo1: Rk Decryption
		SubBytes_L(model, Lo_StateX, Lo_StateY);
		AddRoundKey_L(model, Up_Roundkey, Lo_StateY, Lo_StateZ);
		ShiftRow_L(model, Lo_StateZ, Lo_StateW);
		MixColumns_L(model, Lo_StateW, Lo_StateX);

		// Data Complexity
		BeyondFullCodebook(model, Up_plaintext, Lo_ciphertext, Up_StateX, Up_StateY, Lo_StateX, Lo_StateW);

		// Key Bridge 
		GRBQuadExpr key_involved_independent;
		vector<GRBVar> independent(16);
		KeyBridge(model, Up_Roundkey, Up_Roundkey, key_involved_independent, independent);

		// Involved Keys
		InvolvedRoundkeys_U(model, Up_Roundkey, Up_StateX, Up_StateY);
		InvolvedRoundkeys_L(model, Up_Roundkey, Lo_StateX, Lo_StateW);
		KeyRecovery_U(model, Up_Roundkey, Up_StateX, Up_StateY);
		KeyRecovery_L(model, Up_Roundkey, Lo_StateX, Lo_StateW);

		// Key Recovery
		//KeyRecovery(model, Up_Roundkey, Up_Roundkey);

		// Em - Contradiction Construction
		//Em_Construct_BCT(model, Up_StateX, Lo_StateY);
		vector<vector<GRBVar>> cond(9, vector<GRBVar>(4));
		Em_Construct_All(model, Up_StateX, Lo_StateY, cond);

		// T_guess
		GRBLinExpr T_guess = 0;
		for (int r = 0; r < ROUND; r++) {
			for (int i = 8; i < state; i++) {
				if (r <= midRound) {
					T_guess += Up_Roundkey[r][i][3];
				}
				else {
					T_guess += Lo_Roundkey[r][i][3];
				}
			}
		}
		model.addConstr(T_guess <= 15);

		//// Test - specified values
		//model.addConstr(Up_StateY[5][0][2] == 1);
		//model.addConstr(Lo_StateX[17][0][2] == 1);
		//GRBLinExpr Dist_Length = 0;
		//for (int r = 0; r < midRound; r++) {
		//	Dist_Length += Up_StateY[r][0][2];
		//}
		//for (int r = midRound; r < ROUND; r++) {
		//	Dist_Length += Lo_StateX[r][0][2];
		//}
		//model.addConstr(Dist_Length >= minDistLength);

		GRBLinExpr test1 = 0;
		GRBLinExpr test2 = 0;
		for (int i = 0; i < state; i++) {
			test1 += Up_StateX[1][i][1];
			test2 += Lo_ciphertext[i][1];
		}
		model.addConstr(test1 <= 15);
		model.addConstr(test2 <= 15);

		// Obj Function and Optimize
		GRBQuadExpr obj = 0;
		GRBLinExpr obj1 = 0;
		GRBLinExpr obj2 = 0;
		GRBLinExpr obj3 = 0;
		GRBQuadExpr obj4 = 0;
		for (int i = 0; i < state; i++) {
			obj1 += Up_StateX[1][i][1] + Lo_ciphertext[i][1];
		}

		for (int r = 1; r < midRound; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Up_StateY[r][i][3];
			}
		}
		for (int r = 2; r < midRound; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Up_StateX[r][i][3];
			}
		}

		for (int r = midRound; r < ROUND; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Lo_StateX[r][i][3];
			}
		}
		for (int r = midRound; r < ROUND - 1; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Lo_StateW[r][i][3];
			}
		}

		for (int r = 0; r < ROUND; r++) {
			for (int i = 0; i < 8; i++) {
				obj3 -= Up_Roundkey[r][i][2];
			}
		}
		obj4 -= key_involved_independent;
		obj = (obj2 + obj3 + obj4);
		//model.addQConstr(obj >= -90);
		//obj = (obj1 + obj2 + obj3 + obj4);
		//model.set("NonConvex", "2.0");
		model.addQConstr(obj2 >= -31);
		model.addQConstr(obj4 >= -31);
		model.setObjective(obj, GRB_MAXIMIZE);
		model.optimize();

		// Output Recording
		Output_Print(model, Up_Roundkey, Up_Roundkey, Up_plaintext, Up_ciphertext, Up_StateX, Up_StateY, Up_StateZ, Up_StateW, Lo_plaintext, Lo_ciphertext, Lo_StateX, Lo_StateY, Lo_StateZ, Lo_StateW, cond, independent, obj1, obj2, obj3, obj4);
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}