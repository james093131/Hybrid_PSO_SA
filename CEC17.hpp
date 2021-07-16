#include<stdio.h>
#include<fstream>
#include<iostream>
#include<sstream>
#include<stdlib.h>
#include <string.h>
#include<time.h>
#include<vector>
#include <algorithm>
#include<math.h>
#include<float.h>
#include <sys/stat.h>
#include <iomanip>
#include "cec17_test_func.c"
using namespace std;
class CEC17{
    public:
    void CEC_Results_Records(int NFE ,int MAX_NFE,double Current_Best,double *Each_Run_Evaluation_Best)
    {
        if( NFE == MAX_NFE*0.01)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[0] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.02)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[1] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.05)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[2] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.1)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[3] += Current_Best;

        }
        else if( NFE == MAX_NFE*0.2)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[4] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.3)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[5] += Current_Best;

        }
        else if( NFE == MAX_NFE*0.4)
        {
            
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[6] += Current_Best;   

        }
        else if( NFE == MAX_NFE*0.5)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[7] += Current_Best;

        }
        else if( NFE == MAX_NFE*0.6)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[8] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.7)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[9] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.8)
        {    
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[10] += Current_Best;
        }
        else if( NFE == MAX_NFE*0.9)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[11] += Current_Best;
        }
        else if( NFE == MAX_NFE*1.0)
        {
            cout<<NFE<<' '<<Current_Best<<endl;
            Each_Run_Evaluation_Best[12] += Current_Best;
        }

            
    }
    void cec17_cal(double *P,double *Obj,int DIM,int F)
    {
        cec17_test_func(&P[0], &Obj[0], DIM,1,F);
        Obj[0] -= F*100.0;
    }

    void CEC_Classify(int F,double START,double END,double RUN_STD,double RUN_AVG,int DIM)
    {
        fstream file;           
        string A = "PPSO_2017_CEC_Classify"+to_string(DIM)+".txt";
        file.open(A,ios::app);
        file << scientific << setprecision(8);
        file<<F<<' '<<' '<<RUN_AVG<<' '<<RUN_STD<<' '<<(END - START) / CLOCKS_PER_SEC<<endl;
    }
    void Run_Classify(double *Run_Result,int F,int run,int DIM)
    {
        fstream file; 
        string DIR = "./17_RUN_RESULT/";        
        string A = DIR+to_string(DIM)+"D/"+to_string(F)+"_Run_Classify.txt";
        file.open(A,ios::out);
        file << scientific << setprecision(8);
        for(int i=0;i<run;i++)
        {
            file<<Run_Result[i]<<endl;
        }
    }
};
