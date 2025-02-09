MILP Model for the paper "Optimizing Key Recovery in Impossible Cryptanalysis and Its Automated Tool". The release version will be open-sourced after being accepted.

Full search space provided. You can set the `minDistLength` in the macro definition based on the known number of rounds of distinguisher and modify the main() function to shrink the search space. The single-key or related-key can be set in the `keyschedule()` function.

Test Environment: Ubuntu 20.04.06, Gurobi 10.01.

Model: 
 - CryptanalyticTables.cpp:  Computation of Tables including DBCT*, iUBCT, iLBCT...
 - Deoxys256_IB.cpp: Impossible Boomerang Attack on Deoxys-256
 - Deoxys384_IB.cpp: Impossible Boomerang Attack on Deoxys-384
 - Midori64_ID.cpp: Impossible Differential Attack on Midori-64
 - SKINNYn_ID.cpp: Impossible Differential Attack on SKINNY-64-64 and SKINNY-128-128
 - SKINNY2n_ID.cpp: Impossible Differential Attack on SKINNY-64-128 and SKINNY-128-256
 - SKINNY3n_ID.cpp: Impossible Differential Attack on SKINNY-64-192 and SKINNY-128-384
 - SKINNY3n_IB.cpp: Impossible Boomerang Attack on SKINNY-128-384

Macro definition:
 - ROUND: Rounds of full attack
 - midRound: Pre-defined round number of E_m (contradiction round)

The main variable declarations are as follows:
// Variables
// This model focus on the difference propagation.
// E = Ef ○ Ed ○ Eb
// 
// Attribute for state variables 
// 0 / 1: s0 s1							2: d											3: c									4: rb 
//		   0 0: any but nonzero			   0: key recovery								   0: not cell-condition				   0: Dedicatedly allocated for c=0
//		   0 1: fixed but nonzero		   1: distinguisher							       1: cell-condition (or filter)		   1: rb' (used for pre-guess in quartets construction)  
//         1 0: any (can be zero)								   					       d=0 => c=0							   2: rb* (used for guess in Guess-Filter)				
//		   1 1: zero																       d=1 => c=0 or 1						   c=0 => rb=0
//																																   c=1 => rb>=1						 		  
// 
// Attribute for roundkey variables 
// 0 / 1: s0 s1							2: i											3: g									4: k 
//		   0 0: any but nonzero			   0: not involved in key-recovey (Eb/Ef)		   0: Dedicatedly allocated for i=0		   0: not known 
//		   0 1: fixed but nonzero		   1: in key-recovey phase (Eb/Ef)				   1: in pre-guess						   1: known
//         1 0: any (can be zero)								   						   2: in Guess-Filter					   g>=1 => k=1		
//		   1 1: zero																	   i=0 => g=0							   Key bridge (deduce the known roundkey by key schedule)
//
