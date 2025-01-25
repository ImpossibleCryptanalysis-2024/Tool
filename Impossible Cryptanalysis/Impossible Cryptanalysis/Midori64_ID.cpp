#include "gurobi_c++.h"
#include <iostream>

using namespace std;

#define ROUND 11
#define midRound 5
// #define minDistLength 6
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
// Midori Key Schedule
void keySchedule(GRBModel& model, vector<vector<vector<GRBVar>>>& roundkey) {
	// roundkey[round][state][s0 / s1 / involved / guess / known]
	for (int r = 0; r < ROUND + 1; r++) {
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

	//// Subtweakey Difference Cancellation 
	//// & 
	//// Key Bridge from linear key schedule
	//// 
	//// counter for s0|s1=11 and k=1 in ROUND rounds：0,ROUND
	//for (int i = 0; i < state; i++) {
	//	GRBQuadExpr KS1 = 0;
	//	int j = i;
	//	for (int r = 0; r < ROUND; r++) {
	//		KS1 += roundkey[r][j][0] * roundkey[r][j][1];
	//		j = pp[j];
	//	}
	//	GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	//	//model.addQConstr(KS1 <= ROUND - (ROUND - 1) * e1);
	//	model.addQConstr(KS1 == ROUND - ROUND * e1);
	//}

	//// Related-key Setting
	//GRBQuadExpr ks = 0;
	//for (int r = 0; r < ROUND; r++) {
	//	for (int i = 0; i < state; i++) {
	//		ks += roundkey[r][i][0] * roundkey[r][i][1];
	//	}
	//}
	//model.addQConstr(ks <= 16 * ROUND - 1);

	// Single-Key Setting
	for (int r = 0; r < ROUND + 1; r++) {
		for (int i = 0; i < state; i++) {
			model.addConstr(roundkey[r][i][0] + roundkey[r][i][1] == 2);
		}
	}
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
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);

		model.addConstr(plaintext[i][2] == 0);
		model.addConstr(StateX[midRound - 2][i][2] == 1);
	}

	//// At least one cell with k=1 in Plaintext
	//GRBLinExpr up_p = 0;
	//for (int i = 0; i < state; i++) {
	//	up_p += plaintext[i][1];
	//}
	//model.addConstr(up_p >= 1);

	model.addConstr(StateX[0][0][2] == 0);
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
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);

		model.addConstr(StateY[midRound + 2][i][2] == 1);
		model.addConstr(ciphertext[i][2] == 0);
	}

	//// At least one cell with k=1 in Ciphertext
	//GRBLinExpr lo_c = 0;
	//for (int i = 0; i < state; i++) {
	//	lo_c += ciphertext[i][1];
	//}
	//model.addConstr(lo_c >= 1);

	model.addConstr(StateY[ROUND - 1][0][2] == 0);
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

// Y_r Sf Z_r
void Shuffle_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			// s0 s1 d - remaining
			model.addConstr(StateZ[r][i][0] == StateY[r][shift[i]][0]);
			model.addConstr(StateZ[r][i][1] == StateY[r][shift[i]][1]);
			model.addConstr(StateZ[r][i][2] == StateY[r][shift[i]][2]);
			// c rb - 0
			model.addConstr(StateZ[r][i][3] == 0);
			model.addConstr(StateZ[r][i][4] == 0);
		}
	}
}

void Shuffle_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			// s0 s1 d - remaining
			model.addConstr(StateZ[r][i][0] == StateY[r][shift[i]][0]);
			model.addConstr(StateZ[r][i][1] == StateY[r][shift[i]][1]);
			model.addConstr(StateZ[r][i][2] == StateY[r][shift[i]][2]);
			// c rb - 0
			model.addConstr(StateY[r][i][3] == 0);
			model.addConstr(StateY[r][i][4] == 0);
		}
	}
}

// Z_r MC W_r
void MixColumns_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < ROUND - 1; r++) {
		// Zd <= Wd
		model.addConstr(StateZ[r][0][2] <= StateW[r][0][2]);

		for (int col = 0; col < 4; col++) {
			// In Distinguisher (Zd = 1): Deterministic Propagation
			{
				GRBVar MC_Tmp01[2];
				MC_Tmp01[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp01[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], MC_Tmp01[0], MC_Tmp01[1], StateZ[r][col][2]);
				GRBVar MC_Tmp02[2];
				MC_Tmp02[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp02[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], MC_Tmp02[0], MC_Tmp02[1], StateZ[r][col][2]);
				GRBVar MC_Tmp03[2];
				MC_Tmp03[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp03[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp03[0], MC_Tmp03[1], StateZ[r][col][2]);
				GRBVar MC_Tmp12[2];
				MC_Tmp12[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp12[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], MC_Tmp12[0], MC_Tmp12[1], StateZ[r][col][2]);
				GRBVar MC_Tmp13[2];
				MC_Tmp13[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp13[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp13[0], MC_Tmp13[1], StateZ[r][col][2]);
				GRBVar MC_Tmp23[2];
				MC_Tmp23[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp23[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp23[0], MC_Tmp23[1], StateZ[r][col][2]);


				// Row 0
				_2XOR_deterministic(model, MC_Tmp12[0], MC_Tmp12[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col][0], StateW[r][4 * col][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp13[0], MC_Tmp13[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col][0], StateW[r][4 * col][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp23[0], MC_Tmp23[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col][0], StateW[r][4 * col][1], StateZ[r][col][2]);

				// Row 1
				_2XOR_deterministic(model, MC_Tmp02[0], MC_Tmp02[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp03[0], MC_Tmp03[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp23[0], MC_Tmp23[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][col][2]);

				// Row 2
				_2XOR_deterministic(model, MC_Tmp01[0], MC_Tmp01[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp03[0], MC_Tmp03[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp13[0], MC_Tmp13[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][col][2]);

				// Row 3
				_2XOR_deterministic(model, MC_Tmp01[0], MC_Tmp01[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp02[0], MC_Tmp02[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp12[0], MC_Tmp12[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][col][2]);

				// Exclude 01 01 01 01 => 11 11 11 11 
				GRBLinExpr Zs0_Sum = StateZ[r][4 * col + 0][0] + StateZ[r][4 * col + 1][0] + StateZ[r][4 * col + 2][0] + StateZ[r][4 * col + 3][0];
				GRBLinExpr Zs1_Sum = StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1];
				GRBVar Zs0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Zs1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs0_Ind, 1, Zs0_Sum == 0);
				model.addGenConstrIndicator(Zs0_Ind, 0, Zs0_Sum >= 1);
				model.addGenConstrIndicator(Zs1_Ind, 1, Zs1_Sum == 4);
				model.addGenConstrIndicator(Zs1_Ind, 0, Zs1_Sum <= 3);
				GRBVar Zs_And[2] = { Zs0_Ind,Zs1_Ind };
				GRBVar Zs_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(Zs_Ind, Zs_And, 2);
				model.addGenConstrIndicator(Zs_Ind, 1, StateW[r][4 * col + 0][0] + StateW[r][4 * col + 1][0] + StateW[r][4 * col + 2][0] + StateW[r][4 * col + 3][0] + StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1] <= 7);
			}

			// In Key-Recovery (Zd = 0): Probabilistic Extension <-
			{
				GRBVar Zd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Zd_Not == 1 - StateZ[r][col][2]);

				GRBVar MC_Tmp01[2];
				MC_Tmp01[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp01[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], MC_Tmp01[0], MC_Tmp01[1], Zd_Not);
				GRBVar MC_Tmp02[2];
				MC_Tmp02[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp02[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], MC_Tmp02[0], MC_Tmp02[1], Zd_Not);
				GRBVar MC_Tmp03[2];
				MC_Tmp03[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp03[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp03[0], MC_Tmp03[1], Zd_Not);
				GRBVar MC_Tmp12[2];
				MC_Tmp12[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp12[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], MC_Tmp12[0], MC_Tmp12[1], Zd_Not);
				GRBVar MC_Tmp13[2];
				MC_Tmp13[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp13[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp13[0], MC_Tmp13[1], Zd_Not);
				GRBVar MC_Tmp23[2];
				MC_Tmp23[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp23[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp23[0], MC_Tmp23[1], Zd_Not);


				// Row 0
				_2XOR_probabilistic(model, MC_Tmp12[0], MC_Tmp12[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp13[0], MC_Tmp13[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp23[0], MC_Tmp23[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], Zd_Not);

				// Row 1
				_2XOR_probabilistic(model, MC_Tmp02[0], MC_Tmp02[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp03[0], MC_Tmp03[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp23[0], MC_Tmp23[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], Zd_Not);

				// Row 2
				_2XOR_probabilistic(model, MC_Tmp01[0], MC_Tmp01[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp03[0], MC_Tmp03[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp13[0], MC_Tmp13[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], Zd_Not);

				// Row 3
				_2XOR_probabilistic(model, MC_Tmp01[0], MC_Tmp01[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp02[0], MC_Tmp02[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], Zd_Not);
				_2XOR_probabilistic(model, MC_Tmp12[0], MC_Tmp12[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], Zd_Not);
			}

			// Determine the Cell-Condition
			{
				GRBVar Zd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Zd_Not == 1 - StateZ[r][col][2]);

				// Ws1=0 => Wc=0
				model.addGenConstrIndicator(StateW[r][4 * col + 0][1], 0, StateW[r][4 * col + 0][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 1][1], 0, StateW[r][4 * col + 1][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 2][1], 0, StateW[r][4 * col + 2][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 3][1], 0, StateW[r][4 * col + 3][3] == 0);

				// Zd=1 => Wc=0
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][2], 1, StateW[r][4 * col + 0][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][2], 1, StateW[r][4 * col + 1][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][2], 1, StateW[r][4 * col + 2][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][2], 1, StateW[r][4 * col + 3][3] == 0);

				// ∑Zs1=3 => Wc=0
				model.addGenConstrIndicator(StateW[r][4 * col + 0][3], 1, StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateW[r][4 * col + 1][3], 1, StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateW[r][4 * col + 2][3], 1, StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateW[r][4 * col + 3][3], 1, StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] <= 2);

				// ∑Zs1=1 & ∑Ws1=3  => ∑Wc=2
				// ∑Zs1=2 & ∑Ws1=2  => ∑Wc=1
				// ∑Zs1=0 & ∑Ws1=2  => ∑Wc=2
				//          ∑Ws1=1  => ∑Wc=1
				GRBLinExpr Zs1_Sum = StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1];
				GRBLinExpr Ws1_Sum = StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1];
				GRBLinExpr Wc_Sum = StateW[r][4 * col + 0][3] + StateW[r][4 * col + 1][3] + StateW[r][4 * col + 2][3] + StateW[r][4 * col + 3][3];


				GRBVar Zs1_Sum0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum0_Ind, 1, Zs1_Sum == 0);
				model.addGenConstrIndicator(Zs1_Sum0_Ind, 0, Zs1_Sum >= 1);

				GRBVar Zs1_Sum1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Zs1_Sum1 == Zs1_Sum - 1);
				GRBVar Zs1_Sum1_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Zs1_Sum1_Abs, Zs1_Sum1);
				GRBVar Zs1_Sum1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum1_Ind, 1, Zs1_Sum == 1);
				model.addGenConstrIndicator(Zs1_Sum1_Ind, 0, Zs1_Sum1_Abs >= 1);

				GRBVar Zs1_Sum2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Zs1_Sum2 == Zs1_Sum - 2);
				GRBVar Zs1_Sum2_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Zs1_Sum2_Abs, Zs1_Sum2);
				GRBVar Zs1_Sum2_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum2_Ind, 1, Zs1_Sum == 2);
				model.addGenConstrIndicator(Zs1_Sum2_Ind, 0, Zs1_Sum2_Abs >= 1);

				GRBVar Ws1_Sum1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Ws1_Sum1 == Ws1_Sum - 1);
				GRBVar Ws1_Sum1_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Ws1_Sum1_Abs, Ws1_Sum1);
				GRBVar Ws1_Sum1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum1_Ind, 1, Ws1_Sum == 1);
				model.addGenConstrIndicator(Ws1_Sum1_Ind, 0, Ws1_Sum1_Abs >= 1);

				GRBVar Ws1_Sum2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Ws1_Sum2 == Ws1_Sum - 2);
				GRBVar Ws1_Sum2_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Ws1_Sum2_Abs, Ws1_Sum2);
				GRBVar Ws1_Sum2_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum2_Ind, 1, Ws1_Sum == 2);
				model.addGenConstrIndicator(Ws1_Sum2_Ind, 0, Ws1_Sum2_Abs >= 1);

				GRBVar Ws1_Sum3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Ws1_Sum3 == Ws1_Sum - 3);
				GRBVar Ws1_Sum3_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Ws1_Sum3_Abs, Ws1_Sum3);
				GRBVar Ws1_Sum3_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum3_Ind, 1, Ws1_Sum == 3);
				model.addGenConstrIndicator(Ws1_Sum3_Ind, 0, Ws1_Sum3_Abs >= 1);



				GRBVar Cond_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And0[3] = { Zd_Not,Zs1_Sum1_Ind, Ws1_Sum3_Ind };
				model.addGenConstrAnd(Cond_Ind0, Cond_And0, 3);
				model.addGenConstrIndicator(Cond_Ind0, 1, Wc_Sum == 2);

				GRBVar Cond_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And1[3] = { Zd_Not,Zs1_Sum2_Ind, Ws1_Sum2_Ind };
				model.addGenConstrAnd(Cond_Ind1, Cond_And1, 3);
				model.addGenConstrIndicator(Cond_Ind1, 1, Wc_Sum == 1);

				GRBVar Cond_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And2[3] = { Zd_Not,Zs1_Sum0_Ind, Ws1_Sum2_Ind };
				model.addGenConstrAnd(Cond_Ind2, Cond_And2, 3);
				model.addGenConstrIndicator(Cond_Ind2, 1, Wc_Sum == 2);

				GRBVar Cond_Ind3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And3[3] = { Zd_Not,Ws1_Sum1_Ind };
				model.addGenConstrAnd(Cond_Ind3, Cond_And3, 2);
				model.addGenConstrIndicator(Cond_Ind3, 1, Wc_Sum == 1);

			}

			// Conditions Satisfied to Generate Quartets
			// Wc=0 => Wc'=0
			// Wc'=1 => Wc=1
			model.addGenConstrIndicator(StateW[r][4 * col + 0][3], 0, StateW[r][4 * col + 0][4] == 0);
			model.addGenConstrIndicator(StateW[r][4 * col + 0][4], 1, StateW[r][4 * col + 0][3] == 1);
			model.addGenConstrIndicator(StateW[r][4 * col + 1][3], 0, StateW[r][4 * col + 1][4] == 0);
			model.addGenConstrIndicator(StateW[r][4 * col + 1][4], 1, StateW[r][4 * col + 1][3] == 1);
			model.addGenConstrIndicator(StateW[r][4 * col + 2][3], 0, StateW[r][4 * col + 2][4] == 0);
			model.addGenConstrIndicator(StateW[r][4 * col + 2][4], 1, StateW[r][4 * col + 2][3] == 1);
			model.addGenConstrIndicator(StateW[r][4 * col + 3][3], 0, StateW[r][4 * col + 3][4] == 0);
			model.addGenConstrIndicator(StateW[r][4 * col + 3][4], 1, StateW[r][4 * col + 3][3] == 1);
		}
	}
}

void MixColumns_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < ROUND - 1; r++) {
		// Wd <= Zd
		model.addConstr(StateW[r][0][2] <= StateZ[r][0][2]);

		for (int col = 0; col < 4; col++) {
			// In Distinguisher (Wd = 1): Deterministic Propagation
			{
				GRBVar MC_Tmp01[2];
				MC_Tmp01[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp01[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], MC_Tmp01[0], MC_Tmp01[1], StateW[r][col][2]);
				GRBVar MC_Tmp02[2];
				MC_Tmp02[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp02[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], MC_Tmp02[0], MC_Tmp02[1], StateW[r][col][2]);
				GRBVar MC_Tmp03[2];
				MC_Tmp03[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp03[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp03[0], MC_Tmp03[1], StateW[r][col][2]);
				GRBVar MC_Tmp12[2];
				MC_Tmp12[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp12[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], MC_Tmp12[0], MC_Tmp12[1], StateW[r][col][2]);
				GRBVar MC_Tmp13[2];
				MC_Tmp13[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp13[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp13[0], MC_Tmp13[1], StateW[r][col][2]);
				GRBVar MC_Tmp23[2];
				MC_Tmp23[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp23[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_deterministic(model, StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], MC_Tmp23[0], MC_Tmp23[1], StateW[r][col][2]);


				// Row 0
				_2XOR_deterministic(model, MC_Tmp12[0], MC_Tmp12[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp13[0], MC_Tmp13[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp23[0], MC_Tmp23[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col][0], StateZ[r][4 * col][1], StateW[r][col][2]);

				// Row 1
				_2XOR_deterministic(model, MC_Tmp02[0], MC_Tmp02[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp03[0], MC_Tmp03[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp23[0], MC_Tmp23[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][col][2]);

				// Row 2
				_2XOR_deterministic(model, MC_Tmp01[0], MC_Tmp01[1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp03[0], MC_Tmp03[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp13[0], MC_Tmp13[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][col][2]);

				// Row 3
				_2XOR_deterministic(model, MC_Tmp01[0], MC_Tmp01[1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp02[0], MC_Tmp02[1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][col][2]);
				_2XOR_deterministic(model, MC_Tmp12[0], MC_Tmp12[1], StateW[r][4 * col + 0][0], StateW[r][4 * col + 0][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][col][2]);

				// Exclude  11 11 11 11 <= 01 01 01 01
				GRBLinExpr Ws0_Sum = StateW[r][4 * col + 0][0] + StateW[r][4 * col + 1][0] + StateW[r][4 * col + 2][0] + StateW[r][4 * col + 3][0];
				GRBLinExpr Ws1_Sum = StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1];
				GRBVar Ws0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ws1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws0_Ind, 1, Ws0_Sum == 0);
				model.addGenConstrIndicator(Ws0_Ind, 0, Ws0_Sum >= 1);
				model.addGenConstrIndicator(Ws1_Ind, 1, Ws1_Sum == 4);
				model.addGenConstrIndicator(Ws1_Ind, 0, Ws1_Sum <= 3);
				GRBVar Ws_And[2] = { Ws0_Ind,Ws1_Ind };
				GRBVar Ws_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(Ws_Ind, Ws_And, 2);
				model.addGenConstrIndicator(Ws_Ind, 1, StateZ[r][4 * col + 0][0] + StateZ[r][4 * col + 1][0] + StateZ[r][4 * col + 2][0] + StateZ[r][4 * col + 3][0] + StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1] <= 7);

			}

			// In Key-Recovery (Wd = 0): Probabilistic Extension <-
			{
				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][col][2]);

				GRBVar MC_Tmp01[2];
				MC_Tmp01[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp01[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], MC_Tmp01[0], MC_Tmp01[1], Wd_Not);
				GRBVar MC_Tmp02[2];
				MC_Tmp02[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp02[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], MC_Tmp02[0], MC_Tmp02[1], Wd_Not);
				GRBVar MC_Tmp03[2];
				MC_Tmp03[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp03[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp03[0], MC_Tmp03[1], Wd_Not);
				GRBVar MC_Tmp12[2];
				MC_Tmp12[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp12[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], MC_Tmp12[0], MC_Tmp12[1], Wd_Not);
				GRBVar MC_Tmp13[2];
				MC_Tmp13[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp13[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp13[0], MC_Tmp13[1], Wd_Not);
				GRBVar MC_Tmp23[2];
				MC_Tmp23[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				MC_Tmp23[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				_2XOR_probabilistic(model, StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], MC_Tmp23[0], MC_Tmp23[1], Wd_Not);


				// Row 0
				_2XOR_probabilistic(model, MC_Tmp12[0], MC_Tmp12[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col][0], StateW[r][4 * col][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp13[0], MC_Tmp13[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col][0], StateW[r][4 * col][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp23[0], MC_Tmp23[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col][0], StateW[r][4 * col][1], Wd_Not);

				// Row 1
				_2XOR_probabilistic(model, MC_Tmp02[0], MC_Tmp02[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp03[0], MC_Tmp03[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp23[0], MC_Tmp23[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 1][0], StateW[r][4 * col + 1][1], Wd_Not);

				// Row 2
				_2XOR_probabilistic(model, MC_Tmp01[0], MC_Tmp01[1], StateZ[r][4 * col + 3][0], StateZ[r][4 * col + 3][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp03[0], MC_Tmp03[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp13[0], MC_Tmp13[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 2][0], StateW[r][4 * col + 2][1], Wd_Not);

				// Row 3
				_2XOR_probabilistic(model, MC_Tmp01[0], MC_Tmp01[1], StateZ[r][4 * col + 2][0], StateZ[r][4 * col + 2][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp02[0], MC_Tmp02[1], StateZ[r][4 * col + 1][0], StateZ[r][4 * col + 1][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], Wd_Not);
				_2XOR_probabilistic(model, MC_Tmp12[0], MC_Tmp12[1], StateZ[r][4 * col + 0][0], StateZ[r][4 * col + 0][1], StateW[r][4 * col + 3][0], StateW[r][4 * col + 3][1], Wd_Not);
			}

			// Determine the Cell-Condition
			{
				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][col][2]);

				// Zs1=0 => Zc=0
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][1], 0, StateZ[r][4 * col + 0][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 1][1], 0, StateZ[r][4 * col + 1][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 2][1], 0, StateZ[r][4 * col + 2][3] == 0);
				model.addGenConstrIndicator(StateZ[r][4 * col + 3][1], 0, StateZ[r][4 * col + 3][3] == 0);

				// Wd=1 => Zc=0
				model.addGenConstrIndicator(StateW[r][4 * col + 0][2], 1, StateZ[r][4 * col + 0][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 0][2], 1, StateZ[r][4 * col + 1][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 0][2], 1, StateZ[r][4 * col + 2][3] == 0);
				model.addGenConstrIndicator(StateW[r][4 * col + 0][2], 1, StateZ[r][4 * col + 3][3] == 0);

				// ∑Ws1=3 => Zc=0
				model.addGenConstrIndicator(StateZ[r][4 * col + 0][3], 1, StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateZ[r][4 * col + 1][3], 1, StateW[r][4 * col + 0][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateZ[r][4 * col + 2][3], 1, StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 3][1] <= 2);
				model.addGenConstrIndicator(StateZ[r][4 * col + 3][3], 1, StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] <= 2);

				// ∑Ws1=1 & ∑Zs1=3  => ∑Zc=2
				// ∑Ws1=2 & ∑Zs1=2  => ∑Zc=1
				// ∑Ws1=0 & ∑Zs1=2  => ∑Zc=2
				//          ∑Zs1=1  => ∑Zc=1
				GRBLinExpr Ws1_Sum = StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1];
				GRBLinExpr Zs1_Sum = StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1];
				GRBLinExpr Zc_Sum = StateZ[r][4 * col + 0][3] + StateZ[r][4 * col + 1][3] + StateZ[r][4 * col + 2][3] + StateZ[r][4 * col + 3][3];


				GRBVar Ws1_Sum0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum0_Ind, 1, Ws1_Sum == 0);
				model.addGenConstrIndicator(Ws1_Sum0_Ind, 0, Ws1_Sum >= 1);

				GRBVar Ws1_Sum1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Ws1_Sum1 == Ws1_Sum - 1);
				GRBVar Ws1_Sum1_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Ws1_Sum1_Abs, Ws1_Sum1);
				GRBVar Ws1_Sum1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum1_Ind, 1, Ws1_Sum == 1);
				model.addGenConstrIndicator(Ws1_Sum1_Ind, 0, Ws1_Sum1_Abs >= 1);

				GRBVar Ws1_Sum2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Ws1_Sum2 == Ws1_Sum - 2);
				GRBVar Ws1_Sum2_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Ws1_Sum2_Abs, Ws1_Sum2);
				GRBVar Ws1_Sum2_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Ws1_Sum2_Ind, 1, Ws1_Sum == 2);
				model.addGenConstrIndicator(Ws1_Sum2_Ind, 0, Ws1_Sum2_Abs >= 1);

				GRBVar Zs1_Sum1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Zs1_Sum1 == Zs1_Sum - 1);
				GRBVar Zs1_Sum1_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Zs1_Sum1_Abs, Zs1_Sum1);
				GRBVar Zs1_Sum1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum1_Ind, 1, Zs1_Sum == 1);
				model.addGenConstrIndicator(Zs1_Sum1_Ind, 0, Zs1_Sum1_Abs >= 1);

				GRBVar Zs1_Sum2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Zs1_Sum2 == Zs1_Sum - 2);
				GRBVar Zs1_Sum2_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Zs1_Sum2_Abs, Zs1_Sum2);
				GRBVar Zs1_Sum2_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum2_Ind, 1, Zs1_Sum == 2);
				model.addGenConstrIndicator(Zs1_Sum2_Ind, 0, Zs1_Sum2_Abs >= 1);

				GRBVar Zs1_Sum3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(Zs1_Sum3 == Zs1_Sum - 3);
				GRBVar Zs1_Sum3_Abs = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER); model.addGenConstrAbs(Zs1_Sum3_Abs, Zs1_Sum3);
				GRBVar Zs1_Sum3_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(Zs1_Sum3_Ind, 1, Zs1_Sum == 3);
				model.addGenConstrIndicator(Zs1_Sum3_Ind, 0, Zs1_Sum3_Abs >= 1);



				GRBVar Cond_Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And0[3] = { Wd_Not,Ws1_Sum1_Ind, Zs1_Sum3_Ind };
				model.addGenConstrAnd(Cond_Ind0, Cond_And0, 3);
				model.addGenConstrIndicator(Cond_Ind0, 1, Zc_Sum == 2);

				GRBVar Cond_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And1[3] = { Wd_Not,Ws1_Sum2_Ind, Zs1_Sum2_Ind };
				model.addGenConstrAnd(Cond_Ind1, Cond_And1, 3);
				model.addGenConstrIndicator(Cond_Ind1, 1, Zc_Sum == 1);

				GRBVar Cond_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And2[3] = { Wd_Not,Ws1_Sum0_Ind, Zs1_Sum2_Ind };
				model.addGenConstrAnd(Cond_Ind2, Cond_And2, 3);
				model.addGenConstrIndicator(Cond_Ind2, 1, Zc_Sum == 2);

				GRBVar Cond_Ind3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Cond_And3[3] = { Wd_Not,Zs1_Sum1_Ind };
				model.addGenConstrAnd(Cond_Ind3, Cond_And3, 2);
				model.addGenConstrIndicator(Cond_Ind3, 1, Zc_Sum == 1);

			}

			// Conditions Satisfied to Generate Quartets
			// Zc=0 => Zc'=0
			// Zc'=1 => Zc=1
			model.addGenConstrIndicator(StateZ[r][4 * col + 0][3], 0, StateZ[r][4 * col + 0][4] == 0);
			model.addGenConstrIndicator(StateZ[r][4 * col + 0][4], 1, StateZ[r][4 * col + 0][3] == 1);
			model.addGenConstrIndicator(StateZ[r][4 * col + 1][3], 0, StateZ[r][4 * col + 1][4] == 0);
			model.addGenConstrIndicator(StateZ[r][4 * col + 1][4], 1, StateZ[r][4 * col + 1][3] == 1);
			model.addGenConstrIndicator(StateZ[r][4 * col + 2][3], 0, StateZ[r][4 * col + 2][4] == 0);
			model.addGenConstrIndicator(StateZ[r][4 * col + 2][4], 1, StateZ[r][4 * col + 2][3] == 1);
			model.addGenConstrIndicator(StateZ[r][4 * col + 3][3], 0, StateZ[r][4 * col + 3][4] == 0);
			model.addGenConstrIndicator(StateZ[r][4 * col + 3][4], 1, StateZ[r][4 * col + 3][3] == 1);

		}
	}
}

// W_r ART X_r
void AddRoundKey_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY, vector<vector<GRBVar>>& Plaintext, vector<vector<GRBVar>>& Ciphertext) {
	// internal round 
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			// in Distinguisher
			_2XOR_deterministic(model, StateW[r][i][0], StateW[r][i][1], key[r + 1][i][0], key[r + 1][i][1], StateX[r + 1][i][0], StateX[r + 1][i][1], StateW[r][i][2]);
			// in KeyRecovery
			GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Wd_Not == 1 - StateW[r][i][2]);
			_2XOR_deterministic(model, StateX[r + 1][i][0], StateX[r + 1][i][1], key[r + 1][i][0], key[r + 1][i][1], StateW[r][i][0], StateW[r][i][1], Wd_Not);

			model.addConstr(StateW[r][i][2] <= StateX[r + 1][i][2]);
			model.addConstr(StateX[r + 1][i][3] == 0);
			model.addConstr(StateX[r + 1][i][4] == 0);
		}

	}
	// Beginning: X0 + key[0] -> Plaintext
	// End: Y[ROUND-1] + key[r] -> ciphertext
	for (int i = 0; i < state; i++) {
		GRBVar X0d_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(X0d_Not == 1 - StateX[0][i][2]);
		_2XOR_deterministic(model, StateX[0][i][0], StateX[0][i][1], key[0][i][0], key[0][i][1], Plaintext[i][0], Plaintext[i][1], X0d_Not);

		_2XOR_deterministic(model, StateY[ROUND - 1][i][0], StateY[ROUND - 1][i][1], key[ROUND][i][0], key[ROUND][i][1], Ciphertext[i][0], Ciphertext[i][1], StateY[ROUND - 1][i][2]);
	}

}

void AddRoundKey_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateY, vector<vector<GRBVar>>& Plaintext, vector<vector<GRBVar>>& Ciphertext) {
	// internal round 
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			// in Distinguisher
			_2XOR_deterministic(model, StateX[r + 1][i][0], StateX[r + 1][i][1], key[r + 1][i][0], key[r + 1][i][1], StateW[r][i][0], StateW[r][i][1], StateX[r + 1][i][2]);
			// in KeyRecovery
			GRBVar Xd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Xd_Not == 1 - StateX[r + 1][i][2]);
			_2XOR_deterministic(model, StateW[r][i][0], StateW[r][i][1], key[r + 1][i][0], key[r + 1][i][1], StateX[r + 1][i][0], StateX[r + 1][i][1], Xd_Not);

			model.addConstr(StateW[r][i][2] >= StateX[r + 1][i][2]);
			model.addConstr(StateW[r][i][3] == 0);
			model.addConstr(StateW[r][i][4] == 0);
		}

	}
	// Beginning: X0 + key[0] -> Plaintext
	// End: Y[ROUND-1] + key[r] -> ciphertext
	for (int i = 0; i < state; i++) {
		GRBVar Yd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Yd_Not == 1 - StateY[ROUND - 1][i][2]);
		_2XOR_deterministic(model, StateX[0][i][0], StateX[0][i][1], key[0][i][0], key[0][i][1], Plaintext[i][0], Plaintext[i][1], StateX[0][i][2]);

		_2XOR_deterministic(model, StateY[ROUND - 1][i][0], StateY[ROUND - 1][i][1], key[ROUND][i][0], key[ROUND][i][1], Ciphertext[i][0], Ciphertext[i][1], Yd_Not);
	}

}


// Data Complexity D = 2 ^ {n + c_B + c_f - △B - ▽F + LG(g) / 2 + 1};
// c_B + c_F - △B - △F < 2c (c: cell size)
void BeyondFullCodebook(GRBModel& model, vector<vector<GRBVar>>& Up_plaintext, vector<vector<GRBVar>>& Lo_ciphertext, vector<vector<vector<GRBVar>>>& Up_StateY, vector<vector<vector<GRBVar>>>& Up_StateW, vector<vector<vector<GRBVar>>>& Lo_StateX, vector<vector<vector<GRBVar>>>& Lo_StateZ) {
	GRBLinExpr rB = 0;
	GRBLinExpr rF = 0;
	GRBLinExpr cB = 0;
	GRBLinExpr cF = 0;
	for (int i = 0; i < state; i++) {
		rB += (1 - Up_plaintext[i][1]);
		rF += (1 - Lo_ciphertext[i][1]);
	}
	for (int r = 0; r < midRound; r++) {
		for (int i = 0; i < state; i++) {
			cB += Up_StateY[r][i][3];
		}
	}
	for (int r = 0; r < midRound; r++) {
		for (int i = 0; i < state; i++) {
			cB += Up_StateW[r][i][3];
		}
	}

	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			cF += Lo_StateX[r][i][3];
		}
	}
	for (int r = midRound; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			cF += Lo_StateZ[r][i][3];
		}
	}

	model.addConstr(cB + cF <= rB + rF - 2);// +2);
	//model.addConstr(cF <= rF-1);
	//model.addConstr(cB == rB );
}


void KeyBridge(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_Roundkey, vector<vector<vector<GRBVar>>>& Lo_Roundkey, GRBQuadExpr& key_involved_independent, vector<GRBVar>& independent) {

	for (int i = 0; i < state; i++) {
		GRBLinExpr cnt0 = 0;
		GRBLinExpr cnt1 = 0;
		GRBLinExpr cnt2 = 0;

		for (int r = 1; r < midRound; r++) {
			if (r % 2 == 1) cnt0 += Up_Roundkey[r][i][2];
			if (r % 2 == 0) cnt1 += Up_Roundkey[r][i][2];
		}

		for (int r = midRound; r < ROUND; r++) {
			if (r % 2 == 1) cnt0 += Lo_Roundkey[r][i][2];
			if (r % 2 == 0) cnt1 += Lo_Roundkey[r][i][2];
		}

		cnt2 += Up_Roundkey[0][i][2] + Lo_Roundkey[ROUND][i][2];

		GRBVar Ind0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Ind0, 0, cnt0 <= 0);
		model.addGenConstrIndicator(Ind0, 1, cnt0 >= 1);
		model.addGenConstrIndicator(Ind1, 0, cnt1 <= 0);
		model.addGenConstrIndicator(Ind1, 1, cnt1 >= 1);
		model.addGenConstrIndicator(Ind2, 0, cnt2 <= 0);
		model.addGenConstrIndicator(Ind2, 1, cnt2 >= 1);

		independent[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		independent[i + 16] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		independent[i + 32] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);

		model.addConstr(independent[i] == cnt0);
		model.addConstr(independent[i + 16] == cnt1);
		model.addConstr(independent[i + 32] == cnt2);

		GRBVar Ind_Sum = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Ind_Sum, 1, Ind0 + Ind1 + Ind2 >= 2);
		model.addGenConstrIndicator(Ind_Sum, 0, Ind0 + Ind1 + Ind2 <= 1);


		key_involved_independent += Ind_Sum * 2 + (1 - Ind_Sum) * (Ind0 + Ind1 + Ind2);
	}
}

void InvolvedRoundkeys_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };

	GRBVar key_temp[midRound + 1][16];
	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < state; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(key_temp[r][i] == key[r][i][2]);
		}
	}

	for (int r = 0; r < midRound; r++) {
		// For conditions in Y
		for (int i = 0; i < state; i++) {
			model.addGenConstrIndicator(StateY[r][i][3], 1, key_temp[r][i] == 1);
		}

		// For conditions in W
		for (int col = 0; col < 4; col++) {
			GRBVar Zr0s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr1s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr2s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr3s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Zr0s1_Not == 1 - StateZ[r][4 * col + 0][1]);
			model.addConstr(Zr1s1_Not == 1 - StateZ[r][4 * col + 1][1]);
			model.addConstr(Zr2s1_Not == 1 - StateZ[r][4 * col + 2][1]);
			model.addConstr(Zr3s1_Not == 1 - StateZ[r][4 * col + 3][1]);
			// Row0
			{
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 0][3] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 0][3] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 0][3] };

				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row1
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 1][3] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 1][3] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 1][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row2
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 2][3] };
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 2][3] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 2][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row3
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 3][3] };
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 3][3] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 3][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
			}
		}

		// Key_Temp[r] to Key_Temp[r-1]
		for (int col = 0; col < 4; col++) {
			GRBVar MC_And0[3] = { key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 2]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And1[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 2]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And2[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And3[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 2]] };

			GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

			model.addGenConstrAnd(Ind_And0, MC_And0, 3);
			model.addGenConstrAnd(Ind_And1, MC_And1, 3);
			model.addGenConstrAnd(Ind_And2, MC_And2, 3);
			model.addGenConstrAnd(Ind_And3, MC_And3, 3);

			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 0], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 1], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 2], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 3], 1, Ind_And3 == 1);
		}
	}
}

void InvolvedRoundkeys_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };
	int shift_inv[16] = { 0,7,14,9,5,2,11,12,15,8,1,6,10,13,4,3 };

	// Key_Temp
	GRBVar key_temp[ROUND + 1][state];
	for (int r = 0; r < ROUND + 1; r++) {
		for (int i = 0; i < state; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrIndicator(key_temp[r][i], 1, key[r][i][2] == 1);
		}
	}

	// Last Round conditions of X
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(StateX[ROUND - 1][i][3], 1, key_temp[ROUND][i] == 1);
	}

	// State_Temp for conditions of Z
	GRBVar state_temp[ROUND][state];
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			state_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}

	for (int r = midRound; r < ROUND - 1; r++) {
		// conditions of Z
		for (int col = 0; col < 4; col++) {
			GRBVar Wr0s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr1s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr2s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr3s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Wr0s1_Not == 1 - StateW[r][4 * col + 0][1]);
			model.addConstr(Wr1s1_Not == 1 - StateW[r][4 * col + 1][1]);
			model.addConstr(Wr2s1_Not == 1 - StateW[r][4 * col + 2][1]);
			model.addConstr(Wr3s1_Not == 1 - StateW[r][4 * col + 3][1]);
			// Row0
			{
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 0][3] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 0][3] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 0][3] };

				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row1
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 1][3] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 1][3] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 1][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row2
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 2][3] };
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 2][3] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 2][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row3
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 3][3] };
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 3][3] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 3][3] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
			}

		}
		// conditions of X
		for (int i = 0; i < state; i++) {
			model.addGenConstrIndicator(StateX[r][i][3], 1, state_temp[r][i] == 1);
		}
		// state_temp[r] -> key_temp[r]
		// key_temp[r-1] -> key_temp[r]
		for (int col = 0; col < 4; col++) {
			GRBVar Key_And0[3] = { key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 2],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And1[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 2],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And2[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And3[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 2] };

			GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

			model.addGenConstrAnd(Ind_And0, Key_And0, 3);
			model.addGenConstrAnd(Ind_And1, Key_And1, 3);
			model.addGenConstrAnd(Ind_And2, Key_And2, 3);
			model.addGenConstrAnd(Ind_And3, Key_And3, 3);

			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 0]], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 1]], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 2]], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 3]], 1, Ind_And3 == 1);

			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 0]], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 1]], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 2]], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 3]], 1, Ind_And3 == 1);
		}
	}
	// key_temp[ROUND-1]==key[ROUND]
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(key_temp[ROUND - 1][i], 1, key[ROUND][i][2] == 1);
	}


}

void KeyRecovery_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };

	for (int r = 0; r < midRound; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}

	GRBVar key_temp[midRound + 1][16];
	for (int r = 0; r < midRound + 1; r++) {
		for (int i = 0; i < state; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(key_temp[r][i] == key[r][i][3]);
		}
	}

	for (int r = 0; r < midRound; r++) {
		// For conditions in Y
		for (int i = 0; i < state; i++) {
			model.addGenConstrIndicator(StateY[r][i][4], 1, key_temp[r][i] == 1);
		}

		// For conditions in W
		for (int col = 0; col < 4; col++) {
			GRBVar Zr0s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr1s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr2s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Zr3s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Zr0s1_Not == 1 - StateZ[r][4 * col + 0][1]);
			model.addConstr(Zr1s1_Not == 1 - StateZ[r][4 * col + 1][1]);
			model.addConstr(Zr2s1_Not == 1 - StateZ[r][4 * col + 2][1]);
			model.addConstr(Zr3s1_Not == 1 - StateZ[r][4 * col + 3][1]);
			// Row0
			{
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 0][4] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 0][4] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 0][4] };

				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row1
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 1][4] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 1][4] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 1][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row2
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 2][4] };
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 2][4] };
				GRBVar MC_And3[2] = { Zr3s1_Not ,StateW[r][4 * col + 2][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, key_temp[r][shift[4 * col + 3]] == 1);
			}
			// Row3
			{
				GRBVar MC_And0[2] = { Zr0s1_Not ,StateW[r][4 * col + 3][4] };
				GRBVar MC_And1[2] = { Zr1s1_Not ,StateW[r][4 * col + 3][4] };
				GRBVar MC_And2[2] = { Zr2s1_Not ,StateW[r][4 * col + 3][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);

				model.addGenConstrIndicator(Ind_And0, 1, key_temp[r][shift[4 * col + 0]] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, key_temp[r][shift[4 * col + 1]] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, key_temp[r][shift[4 * col + 2]] == 1);
			}
		}

		// Key_Temp[r] to Key_Temp[r-1]
		for (int col = 0; col < 4; col++) {
			GRBVar MC_And0[3] = { key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 2]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And1[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 2]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And2[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 3]] };
			GRBVar MC_And3[3] = { key_temp[r][shift[4 * col + 0]],key_temp[r][shift[4 * col + 1]],key_temp[r][shift[4 * col + 2]] };

			GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

			model.addGenConstrAnd(Ind_And0, MC_And0, 3);
			model.addGenConstrAnd(Ind_And1, MC_And1, 3);
			model.addGenConstrAnd(Ind_And2, MC_And2, 3);
			model.addGenConstrAnd(Ind_And3, MC_And3, 3);

			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 0], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 1], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 2], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(key_temp[r + 1][4 * col + 3], 1, Ind_And3 == 1);
		}
	}
}

void KeyRecovery_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}

	int shift[16] = { 0, 10, 5, 15, 14, 4, 11, 1, 9, 3, 12, 6, 7, 13, 2, 8 };
	int shift_inv[16] = { 0,7,14,9,5,2,11,12,15,8,1,6,10,13,4,3 };

	// Key_Temp
	GRBVar key_temp[ROUND + 1][state];
	for (int r = 0; r < ROUND + 1; r++) {
		for (int i = 0; i < state; i++) {
			key_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addGenConstrIndicator(key_temp[r][i], 1, key[r][i][3] == 1);
		}
	}

	// Last Round conditions of X
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(StateX[ROUND - 1][i][4], 1, key_temp[ROUND][i] == 1);
	}

	// State_Temp for conditions of Z
	GRBVar state_temp[ROUND][state];
	for (int r = 0; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			state_temp[r][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}

	for (int r = midRound; r < ROUND - 1; r++) {
		// conditions of Z
		for (int col = 0; col < 4; col++) {
			GRBVar Wr0s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr1s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr2s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Wr3s1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			model.addConstr(Wr0s1_Not == 1 - StateW[r][4 * col + 0][1]);
			model.addConstr(Wr1s1_Not == 1 - StateW[r][4 * col + 1][1]);
			model.addConstr(Wr2s1_Not == 1 - StateW[r][4 * col + 2][1]);
			model.addConstr(Wr3s1_Not == 1 - StateW[r][4 * col + 3][1]);
			// Row0
			{
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 0][4] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 0][4] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 0][4] };

				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row1
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 1][4] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 1][4] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 1][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row2
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 2][4] };
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 2][4] };
				GRBVar MC_And3[2] = { Wr3s1_Not ,StateZ[r][4 * col + 2][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And3, MC_And3, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And3, 1, state_temp[r + 1][4 * col + 3] == 1);
			}
			// Row3
			{
				GRBVar MC_And0[2] = { Wr0s1_Not ,StateZ[r][4 * col + 3][4] };
				GRBVar MC_And1[2] = { Wr1s1_Not ,StateZ[r][4 * col + 3][4] };
				GRBVar MC_And2[2] = { Wr2s1_Not ,StateZ[r][4 * col + 3][4] };

				GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

				model.addGenConstrAnd(Ind_And0, MC_And0, 2);
				model.addGenConstrAnd(Ind_And1, MC_And1, 2);
				model.addGenConstrAnd(Ind_And2, MC_And2, 2);

				model.addGenConstrIndicator(Ind_And0, 1, state_temp[r + 1][4 * col + 0] == 1);
				model.addGenConstrIndicator(Ind_And1, 1, state_temp[r + 1][4 * col + 1] == 1);
				model.addGenConstrIndicator(Ind_And2, 1, state_temp[r + 1][4 * col + 2] == 1);
			}

		}
		// conditions of X
		for (int i = 0; i < state; i++) {
			model.addGenConstrIndicator(StateX[r][i][4], 1, state_temp[r][i] == 1);
		}
		// state_temp[r] -> key_temp[r]
		// key_temp[r-1] -> key_temp[r]
		for (int col = 0; col < 4; col++) {
			GRBVar Key_And0[3] = { key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 2],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And1[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 2],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And2[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 3] };
			GRBVar Key_And3[3] = { key_temp[r + 1][4 * col + 0],key_temp[r + 1][4 * col + 1],key_temp[r + 1][4 * col + 2] };

			GRBVar Ind_And0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
			GRBVar Ind_And3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

			model.addGenConstrAnd(Ind_And0, Key_And0, 3);
			model.addGenConstrAnd(Ind_And1, Key_And1, 3);
			model.addGenConstrAnd(Ind_And2, Key_And2, 3);
			model.addGenConstrAnd(Ind_And3, Key_And3, 3);

			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 0]], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 1]], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 2]], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(state_temp[r][shift[4 * col + 3]], 1, Ind_And3 == 1);

			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 0]], 1, Ind_And0 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 1]], 1, Ind_And1 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 2]], 1, Ind_And2 == 1);
			model.addGenConstrIndicator(key_temp[r][shift[4 * col + 3]], 1, Ind_And3 == 1);
		}
	}
	// key_temp[ROUND-1]==key[ROUND]
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(key_temp[ROUND - 1][i], 1, key[ROUND][i][3] == 1);
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
	model.addConstr(KR_count <= 30);
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

void Em_Construct_ID_All(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY, vector<vector<GRBVar>>& cond) {

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
	for (int k = 0; k < 3; k++) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << independent[16 * k + i + j].get(GRB_DoubleAttr_X) << ' ';
			}cout << endl;
		}
		cout << endl;
	}
	cout << endl;

	cout << "Upper Trail:" << endl;
	cout << "        P" << endl;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < state; j += 4) {
			cout << Up_plaintext[i + j][0].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][1].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][2].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][3].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][4].get(GRB_DoubleAttr_X) << ' ';
		}
	}cout << endl;

	for (int r = 0; r < midRound + 1; r++) {
		cout << r << "      K" << endl;
		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < state; j += 4) {
				cout << Up_Roundkey[r][i + j][0].get(GRB_DoubleAttr_X) << Up_Roundkey[r][i + j][1].get(GRB_DoubleAttr_X) << Up_Roundkey[r][i + j][2].get(GRB_DoubleAttr_X) << Up_Roundkey[r][i + j][3].get(GRB_DoubleAttr_X) << Up_Roundkey[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << Up_StateX[r][i + j][0].get(GRB_DoubleAttr_X) << Up_StateX[r][i + j][1].get(GRB_DoubleAttr_X) << Up_StateX[r][i + j][2].get(GRB_DoubleAttr_X) << Up_StateX[r][i + j][3].get(GRB_DoubleAttr_X) << Up_StateX[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl << "SB" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << Up_StateY[r][i + j][0].get(GRB_DoubleAttr_X) << Up_StateY[r][i + j][1].get(GRB_DoubleAttr_X) << Up_StateY[r][i + j][2].get(GRB_DoubleAttr_X) << Up_StateY[r][i + j][3].get(GRB_DoubleAttr_X) << Up_StateY[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}cout << endl << "SR" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << Up_StateZ[r][i + j][0].get(GRB_DoubleAttr_X) << Up_StateZ[r][i + j][1].get(GRB_DoubleAttr_X) << Up_StateZ[r][i + j][2].get(GRB_DoubleAttr_X) << Up_StateZ[r][i + j][3].get(GRB_DoubleAttr_X) << Up_StateZ[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "MC" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << Up_StateW[r][i + j][0].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][1].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][2].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][3].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "ART" << endl;
	}


	cout << "Lower Trail :" << endl;

	for (int r = midRound; r < ROUND; r++) {
		if (r != ROUND - 1) {
			cout << r << "      K" << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_Roundkey[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateX[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "SB" << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateY[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "SR" << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateZ[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateZ[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateZ[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateZ[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateZ[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "MC" << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateW[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateW[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateW[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateW[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateW[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "ARK" << endl << endl;
		}

		else {
			cout << r << "      K " << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_Roundkey[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_Roundkey[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}cout << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateX[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateX[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << "SB" << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_StateY[r][i + j][0].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][1].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][2].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][3].get(GRB_DoubleAttr_X) << Lo_StateY[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl;
			
			cout << "        K" << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_Roundkey[r + 1][i + j][0].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][1].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][2].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][3].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			
			cout << endl;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_ciphertext[i + j][0].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][1].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][2].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][3].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl;
		}
	}
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
		vector<vector<vector<GRBVar>>> Up_Roundkey(ROUND + 1, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		keySchedule(model, Up_Roundkey);
		vector<vector<vector<GRBVar>>> Lo_Roundkey(ROUND + 1, vector<vector<GRBVar>>(state, vector<GRBVar>(5)));
		keySchedule(model, Lo_Roundkey);

		//// Upper Trail Up0&Up1: Rk Encryption
		SubBytes_U(model, Up_StateX, Up_StateY);
		Shuffle_U(model, Up_StateY, Up_StateZ);
		MixColumns_U(model, Up_StateZ, Up_StateW);
		AddRoundKey_U(model, Up_Roundkey, Up_StateW, Up_StateX, Up_StateY, Up_plaintext, Up_ciphertext);

		//Lower Trail Lo0&Lo1: Rk Decryption
		SubBytes_L(model, Lo_StateX, Lo_StateY);
		Shuffle_L(model, Lo_StateY, Lo_StateZ);
		MixColumns_L(model, Lo_StateZ, Lo_StateW);
		AddRoundKey_L(model, Up_Roundkey, Lo_StateW, Lo_StateX, Lo_StateY, Lo_plaintext, Lo_ciphertext);

		// Data Complexity
		BeyondFullCodebook(model, Up_plaintext, Lo_ciphertext, Up_StateY, Up_StateW, Lo_StateX, Lo_StateZ);

		// Key Bridge 
		GRBQuadExpr key_involved_independent;
		vector<GRBVar> independent(48);
		KeyBridge(model, Up_Roundkey, Up_Roundkey, key_involved_independent, independent);

		// Involved Keys
		InvolvedRoundkeys_U(model, Up_Roundkey, Up_StateY, Up_StateZ, Up_StateW);
		InvolvedRoundkeys_L(model, Up_Roundkey, Lo_StateX, Lo_StateZ, Lo_StateW);

		KeyRecovery_U(model, Up_Roundkey, Up_StateY, Up_StateZ, Up_StateW);
		KeyRecovery_L(model, Up_Roundkey, Lo_StateX, Lo_StateZ, Lo_StateW);

		// T_guess
		GRBLinExpr T_guess = 0;
		for (int r = 0; r < ROUND; r++) {
			for (int i = 0; i < state; i++) {
				if (r <= midRound) {
					T_guess += Up_Roundkey[r][i][3];
				}
				else {
					T_guess += Up_Roundkey[r][i][3];
				}
			}
		}
		model.addConstr(T_guess <= 15);

		// Key Recovery
		//KeyRecovery(model, Up_Roundkey, Up_Roundkey);

		// Em - Contradiction Construction
		vector<vector<GRBVar>> cond(9, vector<GRBVar>(4));
		Em_Construct_ID_All(model, Up_StateX, Lo_StateY, cond);

		//// Test - specified values
		//model.addConstr(Up_StateY[2][0][2] == 1);
		//model.addConstr(Lo_StateX[7][0][2] == 1);
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
			test1 += Up_plaintext[i][1];
			test2 += Lo_ciphertext[i][1];
		}
		model.addConstr(test1 <= 15);
		model.addConstr(test2 <= 15);

		/*GRBLinExpr U1 = 0;
		GRBLinExpr U2 = 0;
		GRBLinExpr L1 = 0;
		GRBLinExpr L2 = 0;
		for (int i = 0; i < state; i++) {
			U1 += Up_StateY[2][i][0];
			U2 += Up_StateY[2][i][1];

			L1 += Lo_StateZ[7][i][0];
			L2 += Lo_StateZ[7][i][1];
		}
		model.addConstr(U1 == 15);
		model.addConstr(U2 == 15);
		model.addConstr(Up_StateY[2][10][0] + Up_StateY[2][10][1] == 0);
		model.addConstr(Up_StateW[1][10][0] + Up_StateW[1][10][1] == 0);

		model.addConstr(L1 == 14);
		model.addConstr(L2 == 14);
		model.addConstr(Lo_StateZ[7][4][0] + Lo_StateZ[7][4][1] == 0);
		model.addConstr(Lo_StateZ[7][7][0] + Lo_StateZ[7][7][1] == 0);

		model.addConstr(Lo_StateW[7][5][0] + Lo_StateW[7][5][1] == 2);
		model.addConstr(Lo_StateW[7][6][0] + Lo_StateW[7][6][1] == 2);*/



		// Obj Function and Optimize
		GRBQuadExpr obj = 0;
		GRBLinExpr obj1 = 0;
		GRBLinExpr obj2 = 0;
		GRBLinExpr obj3 = 0;
		GRBQuadExpr obj4 = 0;

		for (int i = 0; i < state; i++) {
			obj1 += Up_plaintext[i][1] + Lo_ciphertext[i][1];
		}

		for (int r = 0; r < midRound; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Up_StateY[r][i][3];
			}
		}
		for (int r = 0; r < midRound; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Up_StateW[r][i][3];
			}
		}

		for (int r = midRound; r < ROUND; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Lo_StateX[r][i][3];
			}
		}
		for (int r = midRound; r < ROUND - 1; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= Lo_StateZ[r][i][3];
			}
		}

		for (int r = 0; r < ROUND + 1; r++) {
			for (int i = 0; i < state; i++) {
				obj3 -= Up_Roundkey[r][i][2];
			}
		}
		obj4 -= key_involved_independent;
		obj = (obj2 + obj3 + obj4);
		//obj = (obj1 + obj2 + obj3 + obj4);
		//model.set("NonConvex", "2.0");
		// model.addQConstr(obj4 >= -31);
		model.addConstr(obj2 >= -31);
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