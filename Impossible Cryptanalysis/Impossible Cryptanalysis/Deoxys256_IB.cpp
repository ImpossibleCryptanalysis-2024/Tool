#include "gurobi_c++.h"
#include <iostream>

using namespace std;

#define ROUND 10
#define midRound 4
// #define minDistLength 7
#define label 5
#define r1 4
#define r2 4
#define state 16

// Initialize roundkey variables
// &
// Deoxys-256 Key Schedule
void keySchedule(GRBModel& model, vector<vector<vector<GRBVar>>>& roundkey) {
	int h[16] = { 1,6,11,12,5,10,15,0,9,14,3,4,13,2,7,8 };
	int hh[16] = { 7,0,13,10,11,4,1,14,15,8,5,2,3,12,9,6 };
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

	// Subtweakey Difference Cancellation 
	// & 
	// Key Bridge from linear key schedule
	// 
	// counter for s0|s1=11 and k=1 in ROUND+1 rounds：0,1,ROUND+1
	for (int i = 0; i < state; i++) {
		GRBQuadExpr KS1 = 0;
		//GRBLinExpr KS2 = 0;
		int j = i;
		for (int r = 0; r < ROUND + 1; r++) {
			KS1 += roundkey[r][j][0] * roundkey[r][j][1];
			//KS2 += roundkey[r][j][4];
			j = hh[j];
		}
		GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addQConstr(KS1 <= ROUND + 1 - ROUND * e1);
		model.addQConstr(KS1 >= ROUND + 1 - (ROUND + 1) * e1);
		//GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		//model.addConstr(KS2 <= ROUND + 1 - ROUND * e2);
		//model.addConstr(KS2 >= ROUND + 1 - (ROUND + 1) * e2);
	}

	// Related-key Setting
	GRBQuadExpr ks = 0;
	for (int r = 0; r < ROUND + 1; r++) {
		for (int i = 0; i < state; i++) {
			ks += roundkey[r][i][0] * roundkey[r][i][1];
		}
	}
	model.addQConstr(ks <= 16 * (ROUND + 1) - 1);

	//// Single-Key Setting
	//for (int r = 0; r < ROUND + 1; r++) {
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
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);

		model.addConstr(plaintext[i][2] == 0);
		model.addConstr(StateX[midRound][i][2] == 1);
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
		}
		plaintext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);
		ciphertext[i][4] = model.addVar(0.0, 2.0, 0.0, GRB_INTEGER);

		model.addConstr(StateY[midRound][i][2] == 1);
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

// Y_r SR Z_r
void ShiftRow_U(GRBModel& model, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	int shift[16] = { 0,5,10,15,4,9,14,3,8,13,2,7,12,1,6,11 };
	for (int r = 0; r < ROUND; r++) {
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

void ShiftRow_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateZ) {
	int shift[16] = { 0,5,10,15,4,9,14,3,8,13,2,7,12,1,6,11 };
	for (int r = 0; r < ROUND; r++) {
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
			// In both distinguisher and key-recovery
			{
				// Σ Zs1+Ws1 \in {0, 1, 2, 3, 8} 
				GRBLinExpr MC1 = 0;
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				for (int i = 0; i < 4; i++) MC1 += StateZ[r][col * 4 + i][1] + StateW[r][col * 4 + i][1];
				model.addConstr(MC1 <= 8 - 5 * e1);
				model.addConstr(MC1 >= 8 - 8 * e1);

				// Σ Zs0*Zs1+Ws0*Ws1 \in {0, 1, 2, 3, 8}
				GRBQuadExpr MC2 = 0;
				GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				for (int i = 0; i < 4; i++) MC2 += StateZ[r][col * 4 + i][0] * StateZ[r][col * 4 + i][1] + StateW[r][col * 4 + i][0] * StateW[r][col * 4 + i][1];
				model.addQConstr(MC2 <= 8 - 5 * e2);
				model.addQConstr(MC2 >= 8 - 8 * e2);
			}

			// In Distinguisher (Zd = 1): Deterministic Propagation
			{
				// ∑ Ws1 \in {0, 4}
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(StateZ[r][4 * col][2] * (StateW[r][col * 4 + 0][1] + StateW[r][col * 4 + 1][1] + StateW[r][col * 4 + 2][1] + StateW[r][col * 4 + 3][1]) == 4 * e1);

				// If Ws1=0 => ∑ Ws0 \in {0, 4}
				GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Ws1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Ws1_Not == 1 - StateW[r][4 * col][1]);
				GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Ind1 == Ws1_Not * StateZ[r][col * 4][2]);
				model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][col * 4 + 0][0] + StateW[r][col * 4 + 1][0] + StateW[r][col * 4 + 2][0] + StateW[r][col * 4 + 3][0] == 4 * e2);

				//  Z=(00, 11, 11, 11) <=> W=(00, 00, 00, 00) when Zd=1
				GRBLinExpr Ws_Sum = StateW[r][col * 4 + 0][0] + StateW[r][col * 4 + 0][1] + StateW[r][col * 4 + 1][0] + StateW[r][col * 4 + 1][1] + StateW[r][col * 4 + 2][0] + StateW[r][col * 4 + 2][1] + StateW[r][col * 4 + 3][0] + StateW[r][col * 4 + 3][1];
				GRBVar MC_Temp1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				GRBVar MC_Temp2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				// MC_Temp1 = ∑ Zs0*Zs1 = 3
				model.addQConstr(MC_Temp1 == StateZ[r][col * 4 + 0][0] * StateZ[r][col * 4 + 0][1] + StateZ[r][col * 4 + 1][0] * StateZ[r][col * 4 + 1][1] + StateZ[r][col * 4 + 2][0] * StateZ[r][col * 4 + 2][1] + StateZ[r][col * 4 + 3][0] * StateZ[r][col * 4 + 3][1]);
				// MC_Temp2 = ∑ (1-Zs0)*(1-Zs1) = 1
				model.addQConstr(MC_Temp2 == (1 - StateZ[r][col * 4 + 0][0]) * (1 - StateZ[r][col * 4 + 0][1]) + (1 - StateZ[r][col * 4 + 1][0]) * (1 - StateZ[r][col * 4 + 1][1]) + (1 - StateZ[r][col * 4 + 2][0]) * (1 - StateZ[r][col * 4 + 2][1]) + (1 - StateZ[r][col * 4 + 3][0]) * (1 - StateZ[r][col * 4 + 3][1]));

				// " => "
				GRBVar MC_Temp1_minus3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp1_minus3 == MC_Temp1 - 3);
				GRBVar MC_Temp1_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp1_Abs, MC_Temp1_minus3);
				GRBVar MC_Flag0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag0, 0, MC_Temp1_Abs >= 1);

				GRBVar MC_Temp2_minus1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp2_minus1 == MC_Temp2 - 1);
				GRBVar MC_Temp2_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp2_Abs, MC_Temp2_minus1);
				GRBVar MC_Flag1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag1, 0, MC_Temp2_Abs >= 1);

				GRBVar MC_Temp_And[3] = { MC_Flag0,MC_Flag1,StateZ[r][col * 4][2] };
				GRBVar MC_Flag2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(MC_Flag2, MC_Temp_And, 3);
				model.addGenConstrIndicator(MC_Flag2, 1, Ws_Sum == 0);


				//  " <= "
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, Ws_Sum >= 1);
				GRBVar MC_Flag3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Flag3 == MC_Ind2 * StateZ[r][col * 4][2]);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp1 == 3);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp2 == 1);
			}

			// In Key-Recovery (Zd = 0): Probabilistic Extension
			{
				// StateZ has s0|s1=00 iff StateW = 00 11 11 11
				// ∑ (1-Zs0)*(1-Zs1) \in {0, 4}
				GRBQuadExpr Zs_MulSum = (1 - StateZ[r][col * 4 + 0][0]) * (1 - StateZ[r][col * 4 + 0][1]) + (1 - StateZ[r][col * 4 + 1][0]) * (1 - StateZ[r][col * 4 + 1][1]) + (1 - StateZ[r][col * 4 + 2][0]) * (1 - StateZ[r][col * 4 + 2][1]) + (1 - StateZ[r][col * 4 + 3][0]) * (1 - StateZ[r][col * 4 + 3][1]);
				GRBVar MC_Temp0 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addQConstr(Zs_MulSum == MC_Temp0);
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(StateZ[r][col * 4][2], 0, MC_Temp0 == 4 * e1);

				// Z = (00,00,00,00) <=> W = (00,11,11,11) When Zd=0

				GRBLinExpr Zs_Sum = StateZ[r][col * 4 + 0][0] + StateZ[r][col * 4 + 0][1] + StateZ[r][col * 4 + 1][0] + StateZ[r][col * 4 + 1][1] + StateZ[r][col * 4 + 2][0] + StateZ[r][col * 4 + 2][1] + StateZ[r][col * 4 + 3][0] + StateZ[r][col * 4 + 3][1];
				GRBVar MC_Temp1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				GRBVar MC_Temp2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				// MC_Temp1 = ∑ Ws0*Ws1 = 3
				model.addQConstr(MC_Temp1 == StateW[r][col * 4 + 0][0] * StateW[r][col * 4 + 0][1] + StateW[r][col * 4 + 1][0] * StateW[r][col * 4 + 1][1] + StateW[r][col * 4 + 2][0] * StateW[r][col * 4 + 2][1] + StateW[r][col * 4 + 3][0] * StateW[r][col * 4 + 3][1]);
				// MC_Temp2 = ∑ (1-Ws0)*(1-Ws1) = 1
				model.addQConstr(MC_Temp2 == (1 - StateW[r][col * 4 + 0][0]) * (1 - StateW[r][col * 4 + 0][1]) + (1 - StateW[r][col * 4 + 1][0]) * (1 - StateW[r][col * 4 + 1][1]) + (1 - StateW[r][col * 4 + 2][0]) * (1 - StateW[r][col * 4 + 2][1]) + (1 - StateW[r][col * 4 + 3][0]) * (1 - StateW[r][col * 4 + 3][1]));

				// " <= "
				GRBVar MC_Temp1_minus3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp1_minus3 == MC_Temp1 - 3);
				GRBVar MC_Temp1_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp1_Abs, MC_Temp1_minus3);
				GRBVar MC_Flag0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag0, 0, MC_Temp1_Abs >= 1);

				GRBVar MC_Temp2_minus1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp2_minus1 == MC_Temp2 - 1);
				GRBVar MC_Temp2_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp2_Abs, MC_Temp2_minus1);
				GRBVar MC_Flag1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag1, 0, MC_Temp2_Abs >= 1);

				GRBVar Zd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Zd_Not == 1 - StateZ[r][col * 4][2]);
				GRBVar MC_Temp_And[3] = { MC_Flag0,MC_Flag1,Zd_Not };
				GRBVar MC_Flag2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(MC_Flag2, MC_Temp_And, 3);
				model.addGenConstrIndicator(MC_Flag2, 1, Zs_Sum == 0);


				//  " => "
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, Zs_Sum >= 1);
				GRBVar MC_Flag3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Flag3 == MC_Ind2 * Zd_Not);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp1 == 3);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp2 == 1);
			}

			// Determine the Cell-Condition
			// Zd=0 & ∑ Zs1<=3 & Ws1=1 => Wc=1 
			{
				GRBVar Zd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Zd_Not == 1 - StateZ[r][4 * col][2]);

				GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind1, 0, StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1] == 4);
				model.addGenConstrIndicator(MC_Ind1, 1, StateZ[r][4 * col + 0][1] + StateZ[r][4 * col + 1][1] + StateZ[r][4 * col + 2][1] + StateZ[r][4 * col + 3][1] <= 3);

				GRBVar MC_Temp_AND0[3] = { Zd_Not,MC_Ind1,StateW[r][4 * col + 0][1] };
				GRBVar MC_Temp_AND1[3] = { Zd_Not,MC_Ind1,StateW[r][4 * col + 1][1] };
				GRBVar MC_Temp_AND2[3] = { Zd_Not,MC_Ind1,StateW[r][4 * col + 2][1] };
				GRBVar MC_Temp_AND3[3] = { Zd_Not,MC_Ind1,StateW[r][4 * col + 3][1] };
				model.addGenConstrAnd(StateW[r][4 * col + 0][3], MC_Temp_AND0, 3);
				model.addGenConstrAnd(StateW[r][4 * col + 1][3], MC_Temp_AND1, 3);
				model.addGenConstrAnd(StateW[r][4 * col + 2][3], MC_Temp_AND2, 3);
				model.addGenConstrAnd(StateW[r][4 * col + 3][3], MC_Temp_AND3, 3);
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

// Z_r MC W_r
void MixColumns_L(GRBModel& model, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < ROUND - 1; r++) {
		// Wd <= Zd
		model.addConstr(StateW[r][0][2] <= StateZ[r][0][2]);

		for (int col = 0; col < 4; col++) {
			// In both distinguisher and key-recovery
			{
				// Σ Ws1+Zs1 \in {0, 1, 2, 3, 8} 
				GRBLinExpr MC1 = 0;
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				for (int i = 0; i < 4; i++) MC1 += StateW[r][col * 4 + i][1] + StateZ[r][col * 4 + i][1];
				model.addConstr(MC1 <= 8 - 5 * e1);
				model.addConstr(MC1 >= 8 - 8 * e1);

				// Σ Ws0*Ws1+Zs0*Zs1 \in {0, 1, 2, 3, 8}
				GRBQuadExpr MC2 = 0;
				GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				for (int i = 0; i < 4; i++) MC2 += StateW[r][col * 4 + i][0] * StateW[r][col * 4 + i][1] + StateZ[r][col * 4 + i][0] * StateZ[r][col * 4 + i][1];
				model.addQConstr(MC2 <= 8 - 5 * e2);
				model.addQConstr(MC2 >= 8 - 8 * e2);
			}

			// In Distinguisher (Wd = 1): Deterministic Propagation
			{
				// ∑ Zs1 \in {0, 4}
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(StateW[r][4 * col][2] * (StateZ[r][col * 4 + 0][1] + StateZ[r][col * 4 + 1][1] + StateZ[r][col * 4 + 2][1] + StateZ[r][col * 4 + 3][1]) == 4 * e1);

				// If Zs1=0 => ∑ Zs0 \in {0, 4}
				GRBVar e2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				GRBVar Zs1_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Zs1_Not == 1 - StateZ[r][4 * col][1]);
				GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Ind1 == Zs1_Not * StateW[r][col * 4][2]);
				model.addGenConstrIndicator(MC_Ind1, 1, StateZ[r][col * 4 + 0][0] + StateZ[r][col * 4 + 1][0] + StateZ[r][col * 4 + 2][0] + StateZ[r][col * 4 + 3][0] == 4 * e2);

				//  W=(00, 11, 11, 11) <=> Z=(00, 00, 00, 00) when Wd=1
				GRBLinExpr Zs_Sum = StateZ[r][col * 4 + 0][0] + StateZ[r][col * 4 + 0][1] + StateZ[r][col * 4 + 1][0] + StateZ[r][col * 4 + 1][1] + StateZ[r][col * 4 + 2][0] + StateZ[r][col * 4 + 2][1] + StateZ[r][col * 4 + 3][0] + StateZ[r][col * 4 + 3][1];
				GRBVar MC_Temp1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				GRBVar MC_Temp2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				// MC_Temp1 = ∑ Ws0*Ws1 = 3
				model.addQConstr(MC_Temp1 == StateW[r][col * 4 + 0][0] * StateW[r][col * 4 + 0][1] + StateW[r][col * 4 + 1][0] * StateW[r][col * 4 + 1][1] + StateW[r][col * 4 + 2][0] * StateW[r][col * 4 + 2][1] + StateW[r][col * 4 + 3][0] * StateW[r][col * 4 + 3][1]);
				// MC_Temp2 = ∑ (1-Ws0)*(1-Ws1) = 1
				model.addQConstr(MC_Temp2 == (1 - StateW[r][col * 4 + 0][0]) * (1 - StateW[r][col * 4 + 0][1]) + (1 - StateW[r][col * 4 + 1][0]) * (1 - StateW[r][col * 4 + 1][1]) + (1 - StateW[r][col * 4 + 2][0]) * (1 - StateW[r][col * 4 + 2][1]) + (1 - StateW[r][col * 4 + 3][0]) * (1 - StateW[r][col * 4 + 3][1]));

				// " => "
				GRBVar MC_Temp1_minus3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp1_minus3 == MC_Temp1 - 3);
				GRBVar MC_Temp1_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp1_Abs, MC_Temp1_minus3);
				GRBVar MC_Flag0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag0, 0, MC_Temp1_Abs >= 1);

				GRBVar MC_Temp2_minus1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp2_minus1 == MC_Temp2 - 1);
				GRBVar MC_Temp2_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp2_Abs, MC_Temp2_minus1);
				GRBVar MC_Flag1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag1, 0, MC_Temp2_Abs >= 1);

				GRBVar MC_Temp_And[3] = { MC_Flag0,MC_Flag1,StateW[r][col * 4][2] };
				GRBVar MC_Flag2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(MC_Flag2, MC_Temp_And, 3);
				model.addGenConstrIndicator(MC_Flag2, 1, Zs_Sum == 0);


				//  " <= "
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, Zs_Sum >= 1);
				GRBVar MC_Flag3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Flag3 == MC_Ind2 * StateW[r][col * 4][2]);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp1 == 3);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp2 == 1);
			}

			// In Key-Recovery (Wd = 0): Probabilistic Extension
			{
				// StateW has s0|s1=00 iff StateZ = 00 11 11 11
				// ∑ (1-Ws0)*(1-Ws1) \in {0, 4}
				GRBQuadExpr Ws_MulSum = (1 - StateW[r][col * 4 + 0][0]) * (1 - StateW[r][col * 4 + 0][1]) + (1 - StateW[r][col * 4 + 1][0]) * (1 - StateW[r][col * 4 + 1][1]) + (1 - StateW[r][col * 4 + 2][0]) * (1 - StateW[r][col * 4 + 2][1]) + (1 - StateW[r][col * 4 + 3][0]) * (1 - StateW[r][col * 4 + 3][1]);
				GRBVar MC_Temp0 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addQConstr(Ws_MulSum == MC_Temp0);
				GRBVar e1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(StateW[r][col * 4][2], 0, MC_Temp0 == 4 * e1);

				// W = (00,00,00,00) <=> Z = (00,11,11,11) Zhen Wd=0

				GRBLinExpr Ws_Sum = StateW[r][col * 4 + 0][0] + StateW[r][col * 4 + 0][1] + StateW[r][col * 4 + 1][0] + StateW[r][col * 4 + 1][1] + StateW[r][col * 4 + 2][0] + StateW[r][col * 4 + 2][1] + StateW[r][col * 4 + 3][0] + StateW[r][col * 4 + 3][1];
				GRBVar MC_Temp1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				GRBVar MC_Temp2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
				// MC_Temp1 = ∑ Zs0*Zs1 = 3
				model.addQConstr(MC_Temp1 == StateZ[r][col * 4 + 0][0] * StateZ[r][col * 4 + 0][1] + StateZ[r][col * 4 + 1][0] * StateZ[r][col * 4 + 1][1] + StateZ[r][col * 4 + 2][0] * StateZ[r][col * 4 + 2][1] + StateZ[r][col * 4 + 3][0] * StateZ[r][col * 4 + 3][1]);
				// MC_Temp2 = ∑ (1-Zs0)*(1-Zs1) = 1
				model.addQConstr(MC_Temp2 == (1 - StateZ[r][col * 4 + 0][0]) * (1 - StateZ[r][col * 4 + 0][1]) + (1 - StateZ[r][col * 4 + 1][0]) * (1 - StateZ[r][col * 4 + 1][1]) + (1 - StateZ[r][col * 4 + 2][0]) * (1 - StateZ[r][col * 4 + 2][1]) + (1 - StateZ[r][col * 4 + 3][0]) * (1 - StateZ[r][col * 4 + 3][1]));

				// " <= "
				GRBVar MC_Temp1_minus3 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp1_minus3 == MC_Temp1 - 3);
				GRBVar MC_Temp1_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp1_Abs, MC_Temp1_minus3);
				GRBVar MC_Flag0 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag0, 0, MC_Temp1_Abs >= 1);

				GRBVar MC_Temp2_minus1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER); model.addConstr(MC_Temp2_minus1 == MC_Temp2 - 1);
				GRBVar MC_Temp2_Abs = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
				model.addGenConstrAbs(MC_Temp2_Abs, MC_Temp2_minus1);
				GRBVar MC_Flag1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Flag1, 0, MC_Temp2_Abs >= 1);

				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][col * 4][2]);
				GRBVar MC_Temp_And[3] = { MC_Flag0,MC_Flag1,Wd_Not };
				GRBVar MC_Flag2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrAnd(MC_Flag2, MC_Temp_And, 3);
				model.addGenConstrIndicator(MC_Flag2, 1, Ws_Sum == 0);


				//  " => "
				GRBVar MC_Ind2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind2, 0, Ws_Sum >= 1);
				GRBVar MC_Flag3 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addQConstr(MC_Flag3 == MC_Ind2 * Wd_Not);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp1 == 3);
				model.addGenConstrIndicator(MC_Flag3, 1, MC_Temp2 == 1);
			}

			// Determine the Cell-Condition
			// Wd=0 & ∑ Ws1<=3 & Zs1=1 => Zc=1 
			{
				GRBVar Wd_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addConstr(Wd_Not == 1 - StateW[r][4 * col][2]);

				GRBVar MC_Ind1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				model.addGenConstrIndicator(MC_Ind1, 0, StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1] == 4);
				model.addGenConstrIndicator(MC_Ind1, 1, StateW[r][4 * col + 0][1] + StateW[r][4 * col + 1][1] + StateW[r][4 * col + 2][1] + StateW[r][4 * col + 3][1] <= 3);

				GRBVar MC_Temp_AND0[3] = { Wd_Not,MC_Ind1,StateZ[r][4 * col + 0][1] };
				GRBVar MC_Temp_AND1[3] = { Wd_Not,MC_Ind1,StateZ[r][4 * col + 1][1] };
				GRBVar MC_Temp_AND2[3] = { Wd_Not,MC_Ind1,StateZ[r][4 * col + 2][1] };
				GRBVar MC_Temp_AND3[3] = { Wd_Not,MC_Ind1,StateZ[r][4 * col + 3][1] };
				model.addGenConstrAnd(StateZ[r][4 * col + 0][3], MC_Temp_AND0, 3);
				model.addGenConstrAnd(StateZ[r][4 * col + 1][3], MC_Temp_AND1, 3);
				model.addGenConstrAnd(StateZ[r][4 * col + 2][3], MC_Temp_AND2, 3);
				model.addGenConstrAnd(StateZ[r][4 * col + 3][3], MC_Temp_AND3, 3);
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
// X_r ART Plaintext (whiten key)
// Z_r ART Ciphertext (Last Round)
void AddRoundKey_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX, vector<vector<GRBVar>>& plaintext, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<GRBVar>>& ciphertext) {
	// whiten-key
	for (int i = 0; i < state; i++) {
		model.addConstr(1 - StateX[0][i][1] - key[0][i][1] + plaintext[i][1] >= 0);
		model.addConstr(StateX[0][i][1] - plaintext[i][1] >= 0);
		model.addConstr(key[0][i][1] - plaintext[i][1] >= 0);
		model.addConstr(1 - StateX[0][i][0] - key[0][i][0] + plaintext[i][0] >= 0);
		model.addConstr(StateX[0][i][1] + key[0][i][0] + plaintext[i][0] - 1 >= 0);
		model.addConstr(StateX[0][i][0] + key[0][i][1] + plaintext[i][0] - 1 >= 0);
		model.addConstr(2 - StateX[0][i][0] - StateX[0][i][1] + key[0][i][0] - plaintext[i][0] >= 0);
		model.addConstr(StateX[0][i][0] - key[0][i][0] - key[0][i][1] - plaintext[i][0] + 2 >= 0);

		// plaintext & StateX[0]: c=0;
		model.addConstr(plaintext[i][3] == 0);
		model.addConstr(StateX[0][i][3] == 0);
		model.addConstr(StateX[0][i][4] == 0);
	}
	// Last Round: ciphertext output
	for (int i = 0; i < state; i++) {
		model.addConstr(1 - StateZ[ROUND - 1][i][1] - key[ROUND][i][1] + ciphertext[i][1] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][1] - ciphertext[i][1] >= 0);
		model.addConstr(key[ROUND][i][1] - ciphertext[i][1] >= 0);
		model.addConstr(1 - StateZ[ROUND - 1][i][0] - key[ROUND][i][0] + ciphertext[i][0] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][1] + key[ROUND][i][0] + ciphertext[i][0] - 1 >= 0);
		model.addConstr(StateZ[ROUND - 1][i][0] + key[ROUND][i][1] + ciphertext[i][0] - 1 >= 0);
		model.addConstr(2 - StateZ[ROUND - 1][i][0] - StateZ[ROUND - 1][i][1] + key[ROUND][i][0] - ciphertext[i][0] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][0] - key[ROUND][i][0] - key[ROUND][i][1] - ciphertext[i][0] + 2 >= 0);

		// ciphertext: c=0;
		model.addConstr(ciphertext[i][3] == 0);
		model.addConstr(ciphertext[i][4] == 0);
	}
	// internal round 
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			model.addQConstr(StateW[r][i][2] * (1 - StateW[r][i][1] - key[r + 1][i][1] + StateX[r + 1][i][1]) + (1 - StateW[r][i][2]) * (1 - StateX[r + 1][i][1] - key[r + 1][i][1] + StateW[r][i][1]) >= 0);
			model.addQConstr(StateW[r][i][2] * (StateW[r][i][1] - StateX[r + 1][i][1]) + (1 - StateW[r][i][2]) * (StateX[r + 1][i][1] - StateW[r][i][1]) >= 0);
			model.addQConstr(StateW[r][i][2] * (key[r + 1][i][1] - StateX[r + 1][i][1]) + (1 - StateW[r][i][2]) * (key[r + 1][i][1] - StateW[r][i][1]) >= 0);
			model.addQConstr(StateW[r][i][2] * (1 - StateW[r][i][0] - key[r + 1][i][0] + StateX[r + 1][i][0]) + (1 - StateW[r][i][2]) * (1 - StateX[r + 1][i][0] - key[r + 1][i][0] + StateW[r][i][0]) >= 0);
			model.addQConstr(StateW[r][i][2] * (StateW[r][i][1] + key[r + 1][i][0] + StateX[r + 1][i][0] - 1) + (1 - StateW[r][i][2]) * (StateX[r + 1][i][1] + key[r + 1][i][0] + StateW[r][i][0] - 1) >= 0);
			model.addQConstr(StateW[r][i][2] * (StateW[r][i][0] + key[r + 1][i][1] + StateX[r + 1][i][0] - 1) + (1 - StateW[r][i][2]) * (StateX[r + 1][i][0] + key[r + 1][i][1] + StateW[r][i][0] - 1) >= 0);
			model.addQConstr(StateW[r][i][2] * (2 - StateW[r][i][0] - StateW[r][i][1] + key[r + 1][i][0] - StateX[r + 1][i][0]) + (1 - StateW[r][i][2]) * (2 - StateX[r + 1][i][0] - StateX[r + 1][i][1] + key[r + 1][i][0] - StateW[r][i][0]) >= 0);
			model.addQConstr(StateW[r][i][2] * (StateW[r][i][0] - key[r + 1][i][0] - key[r + 1][i][1] - StateX[r + 1][i][0] + 2) + (1 - StateW[r][i][2]) * (StateX[r + 1][i][0] - key[r + 1][i][0] - key[r + 1][i][1] - StateW[r][i][0] + 2) >= 0);
			model.addConstr(StateW[r][i][2] <= StateX[r + 1][i][2]);
			model.addConstr(StateX[r][i][3] == 0);
			model.addConstr(StateX[r][i][4] == 0);
		}
	}


}

void AddRoundKey_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateW, vector<vector<vector<GRBVar>>>& StateX, vector<vector<GRBVar>>& plaintext, vector<vector<vector<GRBVar>>>& StateZ, vector<vector<GRBVar>>& ciphertext) {
	// whiten-key
	for (int i = 0; i < state; i++) {
		model.addConstr(1 - StateX[0][i][1] - key[0][i][1] + plaintext[i][1] >= 0);
		model.addConstr(StateX[0][i][1] - plaintext[i][1] >= 0);
		model.addConstr(key[0][i][1] - plaintext[i][1] >= 0);
		model.addConstr(1 - StateX[0][i][0] - key[0][i][0] + plaintext[i][0] >= 0);
		model.addConstr(StateX[0][i][1] + key[0][i][0] + plaintext[i][0] - 1 >= 0);
		model.addConstr(StateX[0][i][0] + key[0][i][1] + plaintext[i][0] - 1 >= 0);
		model.addConstr(2 - StateX[0][i][0] - StateX[0][i][1] + key[0][i][0] - plaintext[i][0] >= 0);
		model.addConstr(StateX[0][i][0] - key[0][i][0] - key[0][i][1] - plaintext[i][0] + 2 >= 0);
	}
	// Last Round: ciphertext output
	for (int i = 0; i < state; i++) {
		model.addConstr(1 - StateZ[ROUND - 1][i][1] - key[ROUND][i][1] + ciphertext[i][1] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][1] - ciphertext[i][1] >= 0);
		model.addConstr(key[ROUND][i][1] - ciphertext[i][1] >= 0);
		model.addConstr(1 - StateZ[ROUND - 1][i][0] - key[ROUND][i][0] + ciphertext[i][0] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][1] + key[ROUND][i][0] + ciphertext[i][0] - 1 >= 0);
		model.addConstr(StateZ[ROUND - 1][i][0] + key[ROUND][i][1] + ciphertext[i][0] - 1 >= 0);
		model.addConstr(2 - StateZ[ROUND - 1][i][0] - StateZ[ROUND - 1][i][1] + key[ROUND][i][0] - ciphertext[i][0] >= 0);
		model.addConstr(StateZ[ROUND - 1][i][0] - key[ROUND][i][0] - key[ROUND][i][1] - ciphertext[i][0] + 2 >= 0);
	}
	// internal round 
	for (int r = 0; r < ROUND - 1; r++) {
		for (int i = 0; i < state; i++) {
			model.addQConstr(StateX[r + 1][i][2] * (1 - StateX[r + 1][i][1] - key[r + 1][i][1] + StateW[r][i][1]) + (1 - StateX[r + 1][i][2]) * (1 - StateW[r][i][1] - key[r + 1][i][1] + StateX[r + 1][i][1]) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (StateX[r + 1][i][1] - StateW[r][i][1]) + (1 - StateX[r + 1][i][2]) * (StateW[r][i][1] - StateX[r + 1][i][1]) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (key[r + 1][i][1] - StateW[r][i][1]) + (1 - StateX[r + 1][i][2]) * (key[r + 1][i][1] - StateX[r + 1][i][1]) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (1 - StateX[r + 1][i][0] - key[r + 1][i][0] + StateW[r][i][0]) + (1 - StateX[r + 1][i][2]) * (1 - StateW[r][i][0] - key[r + 1][i][0] + StateX[r + 1][i][0]) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (StateX[r + 1][i][1] + key[r + 1][i][0] + StateW[r][i][0] - 1) + (1 - StateX[r + 1][i][2]) * (StateW[r][i][1] + key[r + 1][i][0] + StateX[r + 1][i][0] - 1) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (StateX[r + 1][i][0] + key[r + 1][i][1] + StateW[r][i][0] - 1) + (1 - StateX[r + 1][i][2]) * (StateW[r][i][0] + key[r + 1][i][1] + StateX[r + 1][i][0] - 1) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (2 - StateX[r + 1][i][0] - StateX[r + 1][i][1] + key[r + 1][i][0] - StateW[r][i][0]) + (1 - StateX[r + 1][i][2]) * (2 - StateW[r][i][0] - StateW[r][i][1] + key[r + 1][i][0] - StateX[r + 1][i][0]) >= 0);
			model.addQConstr(StateX[r + 1][i][2] * (StateX[r + 1][i][0] - key[r + 1][i][0] - key[r + 1][i][1] - StateW[r][i][0] + 2) + (1 - StateX[r + 1][i][2]) * (StateW[r][i][0] - key[r + 1][i][0] - key[r + 1][i][1] - StateX[r + 1][i][0] + 2) >= 0);
			model.addConstr(StateX[r + 1][i][2] <= StateW[r][i][2]);
		}
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
			cB += Up_StateY[r][i][3] + Up_StateW[r][i][3];
		}
	}

	for (int r = midRound; r < ROUND; r++) {
		for (int i = 0; i < state; i++) {
			cF += Lo_StateX[r][i][3] + Lo_StateZ[r][i][3];
		}
	}

	model.addConstr(cB + cF <= rB + rF);// +2);
}


void KeyBridge(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_Roundkey, vector<vector<vector<GRBVar>>>& Lo_Roundkey, GRBQuadExpr& key_involved_independent, vector<GRBVar>& independent) {

	int hh[16] = { 7,0,13,10,11,4,1,14,15,8,5,2,3,12,9,6 };

	for (int i = 0; i < state; i++) {
		GRBLinExpr cnt = 0;
		int j = i;
		for (int r = 0; r < ROUND + 1; r++) {
			if (r < midRound) {
				cnt += Up_Roundkey[r][j][2];

			}
			else {
				cnt += Lo_Roundkey[r][j][2];

			}
			j = hh[j];
		}

		GRBVar Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrIndicator(Ind, 0, cnt <= 1);
		model.addGenConstrIndicator(Ind, 1, cnt >= 2);

		independent[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addConstr(independent[i] == cnt);

		key_involved_independent += (Ind * 2 + (1 - Ind) * cnt);
	}
}

void InvolvedRoundkeys_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < midRound; r++) {

		// K(r+1)[0, 1, 2, 3][i] = AND {Kr[0, 5, 10, 15][i]}
		// K(r+1)[4, 5, 6, 7][i] = AND {Kr[3, 4, 9, 14][i]}
		// K(r+1)[8, 9, 10, 11][i] = AND {Kr[2, 7, 8, 13][i]}
		// K(r+1)[12, 13, 14, 15][i] = AND {Kr[1, 6, 11, 12][i]}
		GRBVar Col0[4] = { key[r][0][2],key[r][5][2],key[r][10][2],key[r][15][2] };
		GRBVar Col1[4] = { key[r][3][2],key[r][4][2],key[r][9][2],key[r][14][2] };
		GRBVar Col2[4] = { key[r][2][2],key[r][7][2],key[r][8][2],key[r][13][2] };
		GRBVar Col3[4] = { key[r][1][2],key[r][6][2],key[r][11][2],key[r][12][2] };

		for (int i = 0; i < state; i++) {
			// Yc=1 => Ki=1
			model.addGenConstrIndicator(StateY[r][i][3], 1, key[r][i][2] == 1);
		}
		GRBVar Key_Ind[4];
		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[0], Col0, 4);
		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[1], Col1, 4);
		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[2], Col2, 4);
		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[3], Col3, 4);

		for (int i = 0; i < 4; i++) {
			// Wc=1 => Ki=1
			model.addGenConstrIndicator(StateW[r][0 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][1 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][2 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][3 + 4 * i][3], 1, Key_Ind[i] == 1);

			model.addGenConstrIndicator(key[r + 1][0 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][1 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][2 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][3 + 4 * i][2], 1, Key_Ind[i] == 1);
		}

		//for (int i = 0; i < 4; i++) {
		//	model.addGenConstrAnd(key[r + 1][0 + i][2], Col0, 4);
		//	model.addGenConstrAnd(key[r + 1][4 + i][2], Col1, 4);
		//	model.addGenConstrAnd(key[r + 1][8 + i][2], Col2, 4);
		//	model.addGenConstrAnd(key[r + 1][12 + i][2], Col3, 4);
		//}
	}
}

void InvolvedRoundkeys_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateZ) {
	// ShiftRow
	int sr[16] = { 0,5,10,15,4,9,14,3,8,13,2,7,12,1,6,11 };
	// Last Round
	// X(ROUND-1)[c]=1 => K(ROUND)[involved]=1
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(StateX[ROUND - 1][sr[i]][3], 1, key[ROUND][i][2] == 1);
	}

	for (int r = midRound; r < ROUND - 1; r++) {
		// K(r+1)[0, 1, 2, 3][i] = AND {Kr[0, 7, 10, 13][i]}
		// K(r+1)[4, 5, 6, 7][i] = AND {Kr[1, 4, 11, 14][i]}
		// K(r+1)[8, 9, 10, 11][i] = AND {Kr[2, 5, 8, 15][i]}
		// K(r+1)[12, 13, 14, 15][i] = AND {Kr[3, 6, 9, 12][i]}
		GRBVar Col0[4] = { key[r + 2][0][2],key[r + 2][7][2],key[r + 2][10][2],key[r + 2][13][2] };
		GRBVar Col1[4] = { key[r + 2][1][2],key[r + 2][4][2],key[r + 2][11][2],key[r + 2][14][2] };
		GRBVar Col2[4] = { key[r + 2][2][2],key[r + 2][5][2],key[r + 2][8][2],key[r + 2][15][2] };
		GRBVar Col3[4] = { key[r + 2][3][2],key[r + 2][6][2],key[r + 2][9][2],key[r + 2][12][2] };


		for (int i = 0; i < state; i++) {
			// Xr[c]=1 => Kr+1[involved]=1
			model.addGenConstrIndicator(StateX[r][sr[i]][3], 1, key[r + 1][i][2] == 1);
		}
		GRBVar Key_Ind[4];
		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[0], Col0, 4);
		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[1], Col1, 4);
		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[2], Col2, 4);
		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[3], Col3, 4);

		for (int i = 0; i < 4; i++) {
			// Z(r)[c]=1 => K(r+2)i=1
			model.addGenConstrIndicator(StateZ[r][0 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][1 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][2 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][3 + 4 * i][3], 1, Key_Ind[i] == 1);

			model.addGenConstrIndicator(key[r + 1][0 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][1 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][2 + 4 * i][2], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][3 + 4 * i][2], 1, Key_Ind[i] == 1);
		}

		//for (int i = 0; i < 4; i++) {
		//	/*model.addGenConstrAnd(key[r + 1][0 + i][2], Col0, 4);
		//	model.addGenConstrAnd(key[r + 1][4 + i][2], Col1, 4);
		//	model.addGenConstrAnd(key[r + 1][8 + i][2], Col2, 4);
		//	model.addGenConstrAnd(key[r + 1][12 + i][2], Col3, 4);*/
		//	model.addGenConstrIndicator(key[r + 1][0 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][1 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][2 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][3 + 4 * i][2], 1, Key_Ind[i] == 1);
		//}
	}
}

void KeyRecovery_U(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateY, vector<vector<vector<GRBVar>>>& StateW) {
	for (int r = 0; r < midRound; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}
	
	for (int r = 0; r < midRound; r++) {

		// K(r+1)[0, 1, 2, 3][i] = AND {Kr[0, 5, 10, 15][i]}
		// K(r+1)[4, 5, 6, 7][i] = AND {Kr[3, 4, 9, 14][i]}
		// K(r+1)[8, 9, 10, 11][i] = AND {Kr[2, 7, 8, 13][i]}
		// K(r+1)[12, 13, 14, 15][i] = AND {Kr[1, 6, 11, 12][i]}
		GRBVar Col0[4] = { key[r][0][3],key[r][5][3],key[r][10][3],key[r][15][3] };
		GRBVar Col1[4] = { key[r][3][3],key[r][4][3],key[r][9][3],key[r][14][3] };
		GRBVar Col2[4] = { key[r][2][3],key[r][7][3],key[r][8][3],key[r][13][3] };
		GRBVar Col3[4] = { key[r][1][3],key[r][6][3],key[r][11][3],key[r][12][3] };

		for (int i = 0; i < state; i++) {
			// Yc=1 => Ki=1
			model.addGenConstrIndicator(StateY[r][i][4], 1, key[r][i][3] == 1);
		}
		GRBVar Key_Ind[4];
		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[0], Col0, 4);
		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[1], Col1, 4);
		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[2], Col2, 4);
		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[3], Col3, 4);

		for (int i = 0; i < 4; i++) {
			// Wc=1 => Ki=1
			model.addGenConstrIndicator(StateW[r][0 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][1 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][2 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateW[r][3 + 4 * i][4], 1, Key_Ind[i] == 1);

			model.addGenConstrIndicator(key[r + 1][0 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][1 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][2 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][3 + 4 * i][3], 1, Key_Ind[i] == 1);
		}

		//for (int i = 0; i < 4; i++) {
		//	model.addGenConstrAnd(key[r + 1][0 + i][2], Col0, 4);
		//	model.addGenConstrAnd(key[r + 1][4 + i][2], Col1, 4);
		//	model.addGenConstrAnd(key[r + 1][8 + i][2], Col2, 4);
		//	model.addGenConstrAnd(key[r + 1][12 + i][2], Col3, 4);
		//}
	}
}

void KeyRecovery_L(GRBModel& model, vector<vector<vector<GRBVar>>>& key, vector<vector<vector<GRBVar>>>& StateX, vector<vector<vector<GRBVar>>>& StateZ) {
	for (int r = 0; r <ROUND; r++) {
		for (int i = 0; i < 16; i++) {
			model.addConstr(key[r][i][2] >= key[r][i][3]);
		}
	}
	
	// ShiftRow
	int sr[16] = { 0,5,10,15,4,9,14,3,8,13,2,7,12,1,6,11 };
	// Last Round
	// X(ROUND-1)[c]=1 => K(ROUND)[involved]=1
	for (int i = 0; i < state; i++) {
		model.addGenConstrIndicator(StateX[ROUND - 1][sr[i]][4], 1, key[ROUND][i][3] == 1);
	}

	for (int r = midRound; r < ROUND - 1; r++) {
		// K(r+1)[0, 1, 2, 3][i] = AND {Kr[0, 7, 10, 13][i]}
		// K(r+1)[4, 5, 6, 7][i] = AND {Kr[1, 4, 11, 14][i]}
		// K(r+1)[8, 9, 10, 11][i] = AND {Kr[2, 5, 8, 15][i]}
		// K(r+1)[12, 13, 14, 15][i] = AND {Kr[3, 6, 9, 12][i]}
		GRBVar Col0[4] = { key[r + 2][0][3],key[r + 2][7][3],key[r + 2][10][3],key[r + 2][13][3] };
		GRBVar Col1[4] = { key[r + 2][1][3],key[r + 2][4][3],key[r + 2][11][3],key[r + 2][14][3] };
		GRBVar Col2[4] = { key[r + 2][2][3],key[r + 2][5][3],key[r + 2][8][3],key[r + 2][15][3] };
		GRBVar Col3[4] = { key[r + 2][3][3],key[r + 2][6][3],key[r + 2][9][3],key[r + 2][12][3] };


		for (int i = 0; i < state; i++) {
			// Xr[c]=1 => Kr+1[involved]=1
			model.addGenConstrIndicator(StateX[r][sr[i]][4], 1, key[r + 1][i][3] == 1);
		}
		GRBVar Key_Ind[4];
		Key_Ind[0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[0], Col0, 4);
		Key_Ind[1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[1], Col1, 4);
		Key_Ind[2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[2], Col2, 4);
		Key_Ind[3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addGenConstrAnd(Key_Ind[3], Col3, 4);

		for (int i = 0; i < 4; i++) {
			// Z(r)[c]=1 => K(r+2)i=1
			model.addGenConstrIndicator(StateZ[r][0 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][1 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][2 + 4 * i][4], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(StateZ[r][3 + 4 * i][4], 1, Key_Ind[i] == 1);

			model.addGenConstrIndicator(key[r + 1][0 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][1 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][2 + 4 * i][3], 1, Key_Ind[i] == 1);
			model.addGenConstrIndicator(key[r + 1][3 + 4 * i][3], 1, Key_Ind[i] == 1);
		}

		//for (int i = 0; i < 4; i++) {
		//	/*model.addGenConstrAnd(key[r + 1][0 + i][2], Col0, 4);
		//	model.addGenConstrAnd(key[r + 1][4 + i][2], Col1, 4);
		//	model.addGenConstrAnd(key[r + 1][8 + i][2], Col2, 4);
		//	model.addGenConstrAnd(key[r + 1][12 + i][2], Col3, 4);*/
		//	model.addGenConstrIndicator(key[r + 1][0 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][1 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][2 + 4 * i][2], 1, Key_Ind[i] == 1);
		//	model.addGenConstrIndicator(key[r + 1][3 + 4 * i][2], 1, Key_Ind[i] == 1);
		//}
	}
}

void KeyRecovery(GRBModel& model, vector<vector<vector<GRBVar>>>& upkey, vector<vector<vector<GRBVar>>>& lokey) {
	GRBLinExpr KR_count = 0;
	for (int r = 0; r < ROUND + 1; r++) {
		for (int i = 0; i < state; i++) {
			KR_count += upkey[r][i][2] + lokey[r][i][2];
		}
	}
	model.addConstr(KR_count <= 31);
}

void Em_Construct_BCT(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY) {
	GRBLinExpr Switch = 0;
	GRBVar midround_value[16];
	for (int i = 0; i < state; i++) {
		midround_value[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys0_Not = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		model.addConstr(Xs0_Not == 1 - Up_StateX[midRound][i][0]);
		model.addConstr(Ys0_Not == 1 - Lo_StateY[midRound][i][0]);
		GRBVar MidTemp_AND[4] = { Xs0_Not,Up_StateX[midRound][i][1],Ys0_Not,Lo_StateY[midRound][i][1] };
		model.addGenConstrAnd(midround_value[i], MidTemp_AND, 4);
		Switch += midround_value[i];
	}
	model.addConstr(Switch >= 1);
}

void Em_Construct_All(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY, vector<vector<GRBVar>>& cond) {

	// 1-round Em 
	// with [BCT]
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

	// 2-round Em 

	// with [DDT+BCT]
	// Col 0 of MixColumns: X_r[0, 5, 10, 15} , Y_r+1[0, 1, 2, 3], 
	{
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][0][0] + Up_StateX[midRound][5][0] + Up_StateX[midRound][10][0] + Up_StateX[midRound][15][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][0][1] + Up_StateX[midRound][5][1] + Up_StateX[midRound][10][1] + Up_StateX[midRound][15][1];
		GRBQuadExpr Ys_MulSum = Lo_StateY[midRound + 1][0][1] * (1 - Lo_StateY[midRound + 1][0][0]) + Lo_StateY[midRound + 1][1][1] * (1 - Lo_StateY[midRound + 1][1][0]) + Lo_StateY[midRound + 1][2][1] * (1 - Lo_StateY[midRound + 1][2][0]) + Lo_StateY[midRound + 1][3][1] * (1 - Lo_StateY[midRound + 1][3][0]);
		GRBVar Ys_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Ys_Temp == Ys_MulSum);

		GRBVar Xs0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Xs0_Ind, 0, Xs0_Sum <= 1);
		model.addGenConstrIndicator(Xs0_Ind, 1, Xs0_Sum >= 2);
		model.addGenConstrIndicator(Xs1_Ind, 0, Xs1_Sum <= 3);
		model.addGenConstrIndicator(Xs1_Ind, 1, Xs1_Sum == 4);
		model.addGenConstrIndicator(Ys_Ind, 0, Ys_Temp == 0);
		model.addGenConstrIndicator(Ys_Ind, 1, Ys_Temp >= 1);

		cond[1][0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Xs0_Ind,Xs1_Ind,Ys_Ind };
		model.addGenConstrAnd(cond[1][0], Em2_And, 3);
	}
	// Col 1 of MixColumns: X_r[3, 4, 9, 14} , Y_r+1[4, 5, 6, 7], 
	{
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][3][0] + Up_StateX[midRound][4][0] + Up_StateX[midRound][9][0] + Up_StateX[midRound][14][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][3][1] + Up_StateX[midRound][4][1] + Up_StateX[midRound][9][1] + Up_StateX[midRound][14][1];
		GRBQuadExpr Ys_MulSum = Lo_StateY[midRound + 1][4][1] * (1 - Lo_StateY[midRound + 1][4][0]) + Lo_StateY[midRound + 1][5][1] * (1 - Lo_StateY[midRound + 1][5][0]) + Lo_StateY[midRound + 1][6][1] * (1 - Lo_StateY[midRound + 1][6][0]) + Lo_StateY[midRound + 1][7][1] * (1 - Lo_StateY[midRound + 1][7][0]);
		GRBVar Ys_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Ys_Temp == Ys_MulSum);

		GRBVar Xs0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Xs0_Ind, 0, Xs0_Sum <= 1);
		model.addGenConstrIndicator(Xs0_Ind, 1, Xs0_Sum >= 2);
		model.addGenConstrIndicator(Xs1_Ind, 0, Xs1_Sum <= 3);
		model.addGenConstrIndicator(Xs1_Ind, 1, Xs1_Sum == 4);
		model.addGenConstrIndicator(Ys_Ind, 0, Ys_Temp == 0);
		model.addGenConstrIndicator(Ys_Ind, 1, Ys_Temp >= 1);

		cond[1][1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Xs0_Ind,Xs1_Ind,Ys_Ind };
		model.addGenConstrAnd(cond[1][1], Em2_And, 3);
	}
	// Col 2 of MixColumns: X_r[2, 7, 8, 13} , Y_r+1[8, 9, 10, 11], 
	{
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][2][0] + Up_StateX[midRound][7][0] + Up_StateX[midRound][8][0] + Up_StateX[midRound][13][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][2][1] + Up_StateX[midRound][7][1] + Up_StateX[midRound][8][1] + Up_StateX[midRound][13][1];
		GRBQuadExpr Ys_MulSum = Lo_StateY[midRound + 1][8][1] * (1 - Lo_StateY[midRound + 1][8][0]) + Lo_StateY[midRound + 1][9][1] * (1 - Lo_StateY[midRound + 1][9][0]) + Lo_StateY[midRound + 1][10][1] * (1 - Lo_StateY[midRound + 1][10][0]) + Lo_StateY[midRound + 1][11][1] * (1 - Lo_StateY[midRound + 1][11][0]);
		GRBVar Ys_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Ys_Temp == Ys_MulSum);

		GRBVar Xs0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Xs0_Ind, 0, Xs0_Sum <= 1);
		model.addGenConstrIndicator(Xs0_Ind, 1, Xs0_Sum >= 2);
		model.addGenConstrIndicator(Xs1_Ind, 0, Xs1_Sum <= 3);
		model.addGenConstrIndicator(Xs1_Ind, 1, Xs1_Sum == 4);
		model.addGenConstrIndicator(Ys_Ind, 0, Ys_Temp == 0);
		model.addGenConstrIndicator(Ys_Ind, 1, Ys_Temp >= 1);

		cond[1][2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Xs0_Ind,Xs1_Ind,Ys_Ind };
		model.addGenConstrAnd(cond[1][2], Em2_And, 3);
	}
	// Col 3 of MixColumns: X_r[1, 6, 11, 12} , Y_r+1[12, 13, 14, 15], 
	{
		GRBLinExpr Xs0_Sum = Up_StateX[midRound][1][0] + Up_StateX[midRound][6][0] + Up_StateX[midRound][11][0] + Up_StateX[midRound][12][0];
		GRBLinExpr Xs1_Sum = Up_StateX[midRound][1][1] + Up_StateX[midRound][6][1] + Up_StateX[midRound][11][1] + Up_StateX[midRound][12][1];
		GRBQuadExpr Ys_MulSum = Lo_StateY[midRound + 1][12][1] * (1 - Lo_StateY[midRound + 1][12][0]) + Lo_StateY[midRound + 1][13][1] * (1 - Lo_StateY[midRound + 1][13][0]) + Lo_StateY[midRound + 1][14][1] * (1 - Lo_StateY[midRound + 1][14][0]) + Lo_StateY[midRound + 1][15][1] * (1 - Lo_StateY[midRound + 1][15][0]);
		GRBVar Ys_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Ys_Temp == Ys_MulSum);

		GRBVar Xs0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Xs0_Ind, 0, Xs0_Sum <= 1);
		model.addGenConstrIndicator(Xs0_Ind, 1, Xs0_Sum >= 2);
		model.addGenConstrIndicator(Xs1_Ind, 0, Xs1_Sum <= 3);
		model.addGenConstrIndicator(Xs1_Ind, 1, Xs1_Sum == 4);
		model.addGenConstrIndicator(Ys_Ind, 0, Ys_Temp == 0);
		model.addGenConstrIndicator(Ys_Ind, 1, Ys_Temp >= 1);

		cond[1][3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Xs0_Ind,Xs1_Ind,Ys_Ind };
		model.addGenConstrAnd(cond[1][3], Em2_And, 3);
	}

	// with [BCT+DDT]
	// Col 0 of MixColumns: X_r[0, 5, 10, 15} , Y_r+1[0, 1, 2, 3], 
	{
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][0][0] + Lo_StateY[midRound + 1][1][0] + Lo_StateY[midRound + 1][2][0] + Lo_StateY[midRound + 1][3][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][0][1] + Lo_StateY[midRound + 1][1][1] + Lo_StateY[midRound + 1][2][1] + Lo_StateY[midRound + 1][3][1];
		GRBQuadExpr Xs_MulSum = Up_StateX[midRound][0][1] * (1 - Up_StateX[midRound][0][0]) + Up_StateX[midRound][5][1] * (1 - Up_StateX[midRound][5][0]) + Up_StateX[midRound][10][1] * (1 - Up_StateX[midRound][10][0]) + Up_StateX[midRound][15][1] * (1 - Up_StateX[midRound][15][0]);
		GRBVar Xs_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Xs_Temp == Xs_MulSum);

		GRBVar Ys0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Ys0_Ind, 0, Ys0_Sum <= 1);
		model.addGenConstrIndicator(Ys0_Ind, 1, Ys0_Sum >= 2);
		model.addGenConstrIndicator(Ys1_Ind, 0, Ys1_Sum <= 3);
		model.addGenConstrIndicator(Ys1_Ind, 1, Ys1_Sum == 4);
		model.addGenConstrIndicator(Xs_Ind, 0, Xs_Temp == 0);
		model.addGenConstrIndicator(Xs_Ind, 1, Xs_Temp >= 1);



		cond[2][0] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Ys0_Ind,Ys1_Ind,Xs_Ind };
		model.addGenConstrAnd(cond[2][0], Em2_And, 3);
	}
	// Col 1 of MixColumns: X_r[3, 4, 9, 14} , Y_r+1[4, 5, 6, 7], 
	{
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][4][0] + Lo_StateY[midRound + 1][5][0] + Lo_StateY[midRound + 1][6][0] + Lo_StateY[midRound + 1][7][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][4][1] + Lo_StateY[midRound + 1][5][1] + Lo_StateY[midRound + 1][6][1] + Lo_StateY[midRound + 1][7][1];
		GRBQuadExpr Xs_MulSum = Up_StateX[midRound][3][1] * (1 - Up_StateX[midRound][3][0]) + Up_StateX[midRound][4][1] * (1 - Up_StateX[midRound][4][0]) + Up_StateX[midRound][9][1] * (1 - Up_StateX[midRound][9][0]) + Up_StateX[midRound][14][1] * (1 - Up_StateX[midRound][14][0]);
		GRBVar Xs_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Xs_Temp == Xs_MulSum);

		GRBVar Ys0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Ys0_Ind, 0, Ys0_Sum <= 1);
		model.addGenConstrIndicator(Ys0_Ind, 1, Ys0_Sum >= 2);
		model.addGenConstrIndicator(Ys1_Ind, 0, Ys1_Sum <= 3);
		model.addGenConstrIndicator(Ys1_Ind, 1, Ys1_Sum == 4);
		model.addGenConstrIndicator(Xs_Ind, 0, Xs_Temp == 0);
		model.addGenConstrIndicator(Xs_Ind, 1, Xs_Temp >= 1);

		cond[2][1] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Ys0_Ind,Ys1_Ind,Xs_Ind };
		model.addGenConstrAnd(cond[2][1], Em2_And, 3);
	}
	// Col 2 of MixColumns: X_r[2, 7, 8, 13} , Y_r+1[8, 9, 10, 11], 
	{
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][8][0] + Lo_StateY[midRound + 1][9][0] + Lo_StateY[midRound + 1][10][0] + Lo_StateY[midRound + 1][11][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][8][1] + Lo_StateY[midRound + 1][9][1] + Lo_StateY[midRound + 1][10][1] + Lo_StateY[midRound + 1][11][1];
		GRBQuadExpr Xs_MulSum = Up_StateX[midRound][2][1] * (1 - Up_StateX[midRound][2][0]) + Up_StateX[midRound][7][1] * (1 - Up_StateX[midRound][7][0]) + Up_StateX[midRound][8][1] * (1 - Up_StateX[midRound][8][0]) + Up_StateX[midRound][13][1] * (1 - Up_StateX[midRound][13][0]);
		GRBVar Xs_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Xs_Temp == Xs_MulSum);

		GRBVar Ys0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Ys0_Ind, 0, Ys0_Sum <= 1);
		model.addGenConstrIndicator(Ys0_Ind, 1, Ys0_Sum >= 2);
		model.addGenConstrIndicator(Ys1_Ind, 0, Ys1_Sum <= 3);
		model.addGenConstrIndicator(Ys1_Ind, 1, Ys1_Sum == 4);
		model.addGenConstrIndicator(Xs_Ind, 0, Xs_Temp == 0);
		model.addGenConstrIndicator(Xs_Ind, 1, Xs_Temp >= 1);

		cond[2][2] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Ys0_Ind,Ys1_Ind,Xs_Ind };
		model.addGenConstrAnd(cond[2][2], Em2_And, 3);
	}
	// Col 3 of MixColumns: X_r[1, 6, 11, 12} , Y_r+1[12, 13, 14, 15], 
	{
		GRBLinExpr Ys0_Sum = Lo_StateY[midRound + 1][12][0] + Lo_StateY[midRound + 1][13][0] + Lo_StateY[midRound + 1][14][0] + Lo_StateY[midRound + 1][15][0];
		GRBLinExpr Ys1_Sum = Lo_StateY[midRound + 1][12][1] + Lo_StateY[midRound + 1][13][1] + Lo_StateY[midRound + 1][14][1] + Lo_StateY[midRound + 1][15][1];
		GRBQuadExpr Xs_MulSum = Up_StateX[midRound][1][1] * (1 - Up_StateX[midRound][1][0]) + Up_StateX[midRound][6][1] * (1 - Up_StateX[midRound][6][0]) + Up_StateX[midRound][11][1] * (1 - Up_StateX[midRound][11][0]) + Up_StateX[midRound][12][1] * (1 - Up_StateX[midRound][12][0]);
		GRBVar Xs_Temp = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER);
		model.addQConstr(Xs_Temp == Xs_MulSum);

		GRBVar Ys0_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Ys1_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Xs_Ind = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

		model.addGenConstrIndicator(Ys0_Ind, 0, Ys0_Sum <= 1);
		model.addGenConstrIndicator(Ys0_Ind, 1, Ys0_Sum >= 2);
		model.addGenConstrIndicator(Ys1_Ind, 0, Ys1_Sum <= 3);
		model.addGenConstrIndicator(Ys1_Ind, 1, Ys1_Sum == 4);
		model.addGenConstrIndicator(Xs_Ind, 0, Xs_Temp == 0);
		model.addGenConstrIndicator(Xs_Ind, 1, Xs_Temp >= 1);

		cond[2][3] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		GRBVar Em2_And[3] = { Ys0_Ind,Ys1_Ind,Xs_Ind };
		model.addGenConstrAnd(cond[2][3], Em2_And, 3);
	}

	GRBVar cc[9] = {
		cond[0][0],
		cond[1][0], cond[1][1], cond[1][2], cond[1][3],
		cond[2][0], cond[2][1], cond[2][2], cond[2][3]
	};


	GRBVar c = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
	model.addGenConstrOr(c, cc, 9);
	model.addConstr(c == 1);
	model.addConstr(cond[0][0] == 1);
}


void Output_Print(GRBModel& model, vector<vector<vector<GRBVar>>>& Up_Roundkey, vector<vector<vector<GRBVar>>>& Lo_Roundkey, vector<vector<GRBVar>>& Up_plaintext, vector<vector<GRBVar>>& Up_ciphertext, vector<vector<vector<GRBVar>>>& Up_StateX, vector<vector<vector<GRBVar>>>& Up_StateY, vector<vector<vector<GRBVar>>>& Up_StateZ, vector<vector<vector<GRBVar>>>& Up_StateW, vector<vector<GRBVar>>& Lo_plaintext, vector<vector<GRBVar>>& Lo_ciphertext, vector<vector<vector<GRBVar>>>& Lo_StateX, vector<vector<vector<GRBVar>>>& Lo_StateY, vector<vector<vector<GRBVar>>>& Lo_StateZ, vector<vector<vector<GRBVar>>>& Lo_StateW, vector<vector<GRBVar>>& cond, vector<GRBVar>& independent, GRBLinExpr obj1, GRBLinExpr obj2, GRBLinExpr obj3, GRBQuadExpr obj4) {


	cout << "P/C D_Pattern: " << obj1.getValue() << endl;
	cout << "Condition: " << obj2.getValue() << endl;
	cout << "keys: " << obj3.getValue() << endl;
	cout << "Independent: " << obj4.getValue() << endl << endl;

	cout << "Contradiction:" << endl;
	cout << cond[0][0].get(GRB_DoubleAttr_X) << endl;
	for (int i = 1; i < 3; i++) {
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
		for (int j = 0; j < state; j += 4) {
			cout << Up_plaintext[i + j][0].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][1].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][2].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][3].get(GRB_DoubleAttr_X) << Up_plaintext[i + j][4].get(GRB_DoubleAttr_X) << ' ';
		}
	}cout << endl;

	for (int r = 0; r < ROUND; r++) {
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
		}cout << endl << "MC" << endl;

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < state; j += 4) {
				cout << Up_StateW[r][i + j][0].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][1].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][2].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][3].get(GRB_DoubleAttr_X) << Up_StateW[r][i + j][4].get(GRB_DoubleAttr_X) << ' ';
			}
		}
		cout << endl << "ARK" << endl << endl;

	}


	cout << "Lower Trail :" << endl;
	for (int r = 0; r < ROUND; r++) {
		if (r != ROUND - 1) {
			cout << r << "   K" << endl;
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
			cout << r << "  K" << endl;
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


			cout << " K" << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_Roundkey[r + 1][i + j][0].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][1].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][2].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][3].get(GRB_DoubleAttr_X) << Lo_Roundkey[r + 1][i + j][4].get(GRB_DoubleAttr_X) << ' ';
				}
			}
			cout << endl << " C " << endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < state; j += 4) {
					cout << Lo_ciphertext[i + j][0].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][1].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][2].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][3].get(GRB_DoubleAttr_X) << Lo_ciphertext[i + j][4].get(GRB_DoubleAttr_X) << ' ';
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
		ShiftRow_U(model, Up_StateY, Up_StateZ);
		MixColumns_U(model, Up_StateZ, Up_StateW);
		AddRoundKey_U(model, Up_Roundkey, Up_StateW, Up_StateX, Up_plaintext, Up_StateZ, Up_ciphertext);

		//Lower Trail Lo0&Lo1: Rk Decryption
		SubBytes_L(model, Lo_StateX, Lo_StateY);
		ShiftRow_L(model, Lo_StateY, Lo_StateZ);
		MixColumns_L(model, Lo_StateZ, Lo_StateW);
		AddRoundKey_L(model, Lo_Roundkey, Lo_StateW, Lo_StateX, Lo_plaintext, Lo_StateZ, Lo_ciphertext);

		// Data Complexity
		BeyondFullCodebook(model, Up_plaintext, Lo_ciphertext, Up_StateY, Up_StateW, Lo_StateX, Lo_StateZ);

		// Key Bridge 
		GRBQuadExpr key_involved_independent;
		vector<GRBVar> independent(32);
		KeyBridge(model, Up_Roundkey, Lo_Roundkey, key_involved_independent, independent);

		// Involved Keys
		InvolvedRoundkeys_U(model, Up_Roundkey, Up_StateY, Up_StateW);
		InvolvedRoundkeys_L(model, Lo_Roundkey, Lo_StateX, Lo_StateZ);
		KeyRecovery_U(model, Up_Roundkey, Up_StateY, Up_StateW);
		KeyRecovery_L(model, Lo_Roundkey, Lo_StateY, Lo_StateW);

		// Key Recovery
		// KeyRecovery(model, Up_Roundkey, Lo_Roundkey);

		// Em - Contradiction Construction
		//Em_Construct_BCT(model, Up_StateX, Lo_StateY);
		vector<vector<GRBVar>> cond(3, vector<GRBVar>(4));
		Em_Construct_All(model, Up_StateX, Lo_StateY, cond);

		//// Minimum rounds distinguisher covers
		//model.addConstr(Up_StateW[0][0][2] == 1);
		//model.addConstr(Lo_StateX[8][0][2] == 1);
		//GRBLinExpr Dist_Length = 0;
		//for (int r = 0; r < midRound; r++) {
		//	Dist_Length += Up_StateW[r][0][2];
		//}
		//for (int r = midRound; r < ROUND; r++) {
		//	Dist_Length += Lo_StateX[r][0][2];
		//}
		//model.addConstr(Dist_Length >= minDistLength);

		// T_guess
		GRBLinExpr T_guess = 0;
		for (int r = 0; r < ROUND; r++) {
			for (int i = 0; i < state; i++) {
				if (r <= midRound) {
					T_guess += Up_Roundkey[r][i][3];
				}
				else {
					T_guess += Lo_Roundkey[r][i][3];
				}
			}
		}
		model.addConstr(T_guess <= 15);

		// Allocated
		model.addConstr(Up_StateX[4][15][0] == 0);
		model.addConstr(Up_StateX[4][15][1] == 1);
		model.addConstr(Lo_StateY[4][15][0] == 0);
		model.addConstr(Lo_StateY[4][15][1] == 1);
		model.addConstr(Lo_StateX[8][0][0] == 0);
		model.addConstr(Lo_StateX[8][0][1] == 1);
		model.addConstr(Lo_StateX[8][11][0] == 0);
		model.addConstr(Lo_StateX[8][11][1] == 1);
	

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
				obj2 -= (Up_StateY[r][i][3] + Up_StateW[r][i][3]);
			}
		}

		for (int r = midRound; r < ROUND; r++) {
			for (int i = 0; i < state; i++) {
				obj2 -= (Lo_StateX[r][i][3] + Lo_StateZ[r][i][3]);
			}
		}

		for (int r = 0; r < ROUND + 1; r++) {
			for (int i = 0; i < state; i++) {
				if (r < midRound) {
					obj3 -= Up_Roundkey[r][i][2];
				}
				else {
					obj3 -= Lo_Roundkey[r][i][2];
				}
			}
		}

		obj4 -= key_involved_independent;
		obj = (obj2 + obj3 + obj4);

		//model.set("NonConvex", "2.0");
		model.setObjective(obj, GRB_MAXIMIZE);
		model.optimize();

		// Output 
		Output_Print(model, Up_Roundkey, Lo_Roundkey, Up_plaintext, Up_ciphertext, Up_StateX, Up_StateY, Up_StateZ, Up_StateW, Lo_plaintext, Lo_ciphertext, Lo_StateX, Lo_StateY, Lo_StateZ, Lo_StateW, cond, independent, obj1, obj2, obj3, obj4);
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << "Exception during optimization" << endl;
	}

}