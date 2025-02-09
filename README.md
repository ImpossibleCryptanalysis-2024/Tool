This is the source code of the MILP Model for the paper ["Optimizing Key Recovery in Impossible Cryptanalysis and Its Automated Tool"](https://eprint.iacr.org/2025/158).

## Test environment: 
```shell
Ubuntu 20.04.06, Gurobi 10.01.
```

## File: 
> * Impossible Cryptanalysis.sln: Solution for the work on Visual Studio
>> * CryptanalyticTables.cpp:  Computation of Tables including DBCT*, iUBCT, iLBCT...
>> * Deoxys256_IB.cpp: Impossible Boomerang Attack on Deoxys-256
>> * Deoxys384_IB.cpp: Impossible Boomerang Attack on Deoxys-384
>> * Midori64_ID.cpp: Impossible Differential Attack on Midori-64
>> * SKINNYn_ID.cpp: Impossible Differential Attack on SKINNY-64-64 and SKINNY-128-128
>> * SKINNY2n_ID.cpp: Impossible Differential Attack on SKINNY-64-128 and SKINNY-128-256
>> * SKINNY3n_ID.cpp: Impossible Differential Attack on SKINNY-64-192 and SKINNY-128-384
>> * SKINNY3n_IB.cpp: Impossible Boomerang Attack on SKINNY-128-384

## Parameters:
 - `ROUND`: Rounds of full attack
 - `midRound`: Pre-defined round number of E_m (contradiction round)
 - `minDistLength`: Number of rounds of the existing distinguishers (Full search space provided in the code. You can set the `minDistLength` in the macro definition based on the known number of rounds of distinguisher and modify the `main()` function to shrink the search space. The single-key or related-key can be set in the `keyschedule()` function.)

## Main variable declarations:
This model focus on the difference propagation.
E = Ef ○ Ed ○ Eb

### Attribute for state variables 
```cpp
0. / 1. s0 s1
0 0: any but nonzero			
0 1: fixed but nonzero		 
1 0: any (can be zero)				
1 1: zero	
```
```cpp
2. d		
0: key recovery	
1: distinguisher	
```
```cpp
3. c									    
0: not cell-condition				      
1: cell-condition (or filter)		   
d=0 => c=0							                
d=1 => c=0 or 1				
```
```cpp
4. rb 
0: Dedicatedly allocated for c=0
1: rb' (used for pre-guess in quartets(pairs) construction)  
2: rb* (used for guess in Guess-Filter)				
 c=0 => rb=0
 c=1 => rb>=1
``` 																																	 

### Attribute for key variables 
```cpp
0. / 1. s0 s1
0 0: any but nonzero
0 1: fixed but nonzero
1 0: any (can be zero)
1 1: zero
```
```cpp
2. i
0: not involved in key-recovey (Eb/Ef)		 
1: in key-recovey phase (Eb/Ef)			
2: in Guess-Filter			
```
```cpp
3. g					
0: Dedicatedly allocated for i=0		
1: pre-guess					
i=0 => g=0				
```
```cpp
4. k 
0: not known 
1: known
g>=1 => k=1		
Key bridge (deduce the known roundkey by key schedule)
```
