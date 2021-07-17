#include "CEC17.hpp"

typedef vector<char> c1d;
typedef vector<c1d> c2d;
typedef vector<c2d> c3d;
typedef vector<c3d> c4d;
typedef vector<int> i1d;
typedef vector<i1d> i2d;
typedef vector<i2d> i3d;
typedef vector<i3d> i4d;
typedef vector<double>d1d;
typedef vector<d1d> d2d;
typedef vector<d2d> d3d;
typedef vector<d3d> d4d;

// class PSO_SA : CEC21{
class PSO_SA : CEC17{

    public:
        // d1d Each_Run_Evaluation_Best;
        d1d Each_Run_Result;

    public:
        void run(int run,int dim,int pop,int F,int F_T,int MAX_NFE)
        {
            srand((unsigned)time(NULL));
            double START,END;
            START = clock();
            int r = 0;
            INI_RUN(run);
            while(r <run )
            {
                INI(dim,pop);
                for(int i=0;i<pop;i++)
                {
                    RANDOM_INI(dim,i,PSO_inf.Particle);
                    Evaluation(pop,dim,F,F_T);
                }

                while(NFE < MAX_NFE)
                {
                    Current_inf.Previous_Best_Value = Current_inf.Current_Best_Value;
                    Update_Velocity(pop,dim,MAX_NFE);
                    Update_Position(pop,dim);
                    Evaluation(pop,dim,F,F_T);
                 
                    SA_or_not(dim, F, F_T);
                    
                    // cout<<NFE<<' '<< Current_inf.Current_Best_Value<<endl;
                }
                Each_Run_Result[r] = Current_inf.Current_Best_Value;
                r++;
            }
            END = clock();
            OUTPUT(dim,F,F_T,MAX_NFE,run,START,END);
        }

    private:
        struct SA_Coef
        {
            double Temp;
            double Dec_Temp;
            double totalMoves;
            double movesAccepted;
        };
        struct PSO_Coef
        {
            double W;
            double W_min;
            double W_max;
            double V_max;
            double C1;
            double C2;
        };
        struct PSO_Parameter
        {
            d2d Particle;
            d1d Objective;
            d2d Velocity;  
        };
        struct Current_best
        {
            double Current_Best_Value;
            d1d Current_Best_Coordinate;  
            double Previous_Best_Value;
        };
        struct Personal_best
        {
            d1d Personal_Best_Value;
            d2d Personal_Best_Coordinate;        
        };
        SA_Coef sa_set;
        PSO_Coef pso_set;
        Current_best Current_inf;
        Personal_best Personal_inf;
        PSO_Parameter PSO_inf;
        double max = 100.0;
        double min = -100.0;
        int NFE = 0 ;


    private:
        inline double randomDouble()
        { 
            return (double) rand() / (RAND_MAX + 1.0);
        }
        void RANDOM_INI(int DIM,int index,d2d &arr)
        {
            for(int i=0;i<DIM;i++)
            {
                double a = (max - min) * rand() / (RAND_MAX + 1.0) + min;
                arr[index][i] = a;
            }

        }
        void INI(int DIM,int pop)
        {
            cout << scientific << setprecision(8);
            PSO_inf.Particle.clear();
            PSO_inf.Particle.swap(PSO_inf.Particle);

            PSO_inf.Velocity.clear();
            PSO_inf.Velocity.swap(PSO_inf.Velocity);

            PSO_inf.Objective.clear();
            PSO_inf.Objective.swap(PSO_inf.Objective);

            Personal_inf.Personal_Best_Coordinate.clear();
            Personal_inf.Personal_Best_Coordinate.swap(Personal_inf.Personal_Best_Coordinate);

            Personal_inf.Personal_Best_Value.clear();
            Personal_inf.Personal_Best_Value.swap(Personal_inf.Personal_Best_Value);
        
            Current_inf.Current_Best_Coordinate.clear();
            Current_inf.Current_Best_Coordinate.swap(Current_inf.Current_Best_Coordinate);

            PSO_inf.Particle.assign(pop,d1d(DIM));
            PSO_inf.Velocity.assign(pop,d1d(DIM,0));
            PSO_inf.Objective.resize(pop);


            Current_inf.Current_Best_Coordinate.resize(DIM);
            Current_inf.Current_Best_Value = DBL_MAX;
            Current_inf.Previous_Best_Value = DBL_MAX;

            Personal_inf.Personal_Best_Coordinate.assign(pop,d1d(DIM));
            Personal_inf.Personal_Best_Value.resize(pop,DBL_MAX);

            sa_set.Dec_Temp = 0.95;
            sa_set.Temp = 1.0;
            sa_set.totalMoves = 0;
            sa_set.movesAccepted = 0;

            pso_set.C1 = 0.8;
            pso_set.C2 = 1.7;
            pso_set.W_max = 0.55;
            pso_set.W_min = 0.1;
            pso_set.V_max = 0.1;
            max = 100;
            min = -100;
            NFE = 0;
        }   
        void INI_RUN(int run)
        {
            Each_Run_Result.resize(run,DBL_MAX);
        }
        void Update_Velocity(int pop,int DIM,int MAX_NFE)
        {
            W_update(MAX_NFE);
            for(int i=0;i<pop;i++)
            {
                for(int j=0;j<DIM;j++)
                {
                    double r1 = randomDouble();
                    double r2 = randomDouble();
                    PSO_inf.Velocity[i][j] =  pso_set.W*PSO_inf.Velocity[i][j] \
                    + pso_set.C1*r1*( Personal_inf.Personal_Best_Coordinate[i][j] - PSO_inf.Particle[i][j] ) \
                    + pso_set.C2*r2*( Current_inf.Current_Best_Coordinate[j] - PSO_inf.Particle[i][j] );

                    if(PSO_inf.Velocity[i][j] > pso_set.V_max*(max-min))
                        PSO_inf.Velocity[i][j] = pso_set.V_max*(max-min);
                    else if(PSO_inf.Velocity[i][j] < -pso_set.V_max*(max-min))
                        PSO_inf.Velocity[i][j] = -pso_set.V_max*(max-min);

                }
            }
        }
        void Update_Position(int pop,int DIM)
        {
            for(int i=0;i<pop;i++)
            {
                for(int j=0;j<DIM;j++)
                {
                    PSO_inf.Particle[i][j] += PSO_inf.Velocity[i][j] ;

                    if(PSO_inf.Particle[i][j] > max)
                        PSO_inf.Particle[i][j] = max;
                    else if(PSO_inf.Particle[i][j] < min)
                        PSO_inf.Particle[i][j] = min;
                }
            }
        }
        void W_update(int MAX_NFE)
        {
            pso_set.W = pso_set.W_max -  ( (pso_set.W_max - pso_set.W_min)/MAX_NFE ) *NFE;
        }
        void Evaluation(int pop,int DIM,int F,int F_T)
        {
            for(int i=0;i<pop;i++)
            {   
                
                // CEC21::cec21_cal(&PSO_inf.Particle[i][0], PSO_inf.Objective[i], DIM,F,F_T);
                CEC17::cec17_cal(&PSO_inf.Particle[i][0],&PSO_inf.Objective[i], DIM,F);
                NFE++;

                if(PSO_inf.Objective[i] <= Personal_inf.Personal_Best_Value[i])
                {
                    Personal_inf.Personal_Best_Value[i] = PSO_inf.Objective[i];
                    Personal_inf.Personal_Best_Coordinate[i].assign(PSO_inf.Particle[i].begin(),PSO_inf.Particle[i].end());

                    if(PSO_inf.Objective[i] <= Current_inf.Current_Best_Value)
                    {
                        Current_inf.Current_Best_Value = PSO_inf.Objective[i];
                        Current_inf.Current_Best_Coordinate.assign(PSO_inf.Particle[i].begin(),PSO_inf.Particle[i].end());
                    }
                }
            }
        }
        void SA_or_not(int DIM,int F,int F_T )
        {
            if(Current_inf.Previous_Best_Value == Current_inf.Current_Best_Value)
            {
                SA_part(DIM,F,F_T);
            }
            else
            {   
                sa_set.Temp = sa_set.Temp * sa_set.Dec_Temp;
            }
        }
        void SA_part(int DIM,int F,int F_T)
        {
            sa_set.totalMoves ++;
            double V_step = SA_Vector();
            d1d Trial_X = Current_inf.Current_Best_Coordinate;
            double Trial_obj = Current_inf.Current_Best_Value;
            for(int i=0;i<DIM;i++)
            {
                double r = randomDouble()*2-1;
                Trial_X[i] = Trial_X[i] + r*V_step;
            }
            CEC17::cec17_cal(&Trial_X[0],&Trial_obj, DIM,F);
            NFE++;

            if(Trial_obj <= Current_inf.Current_Best_Value)
            {
                Current_inf.Current_Best_Value = Trial_obj;
                Current_inf.Current_Best_Coordinate = Trial_X;
                sa_set.Temp = sa_set.Temp * sa_set.Dec_Temp;
                sa_set.movesAccepted ++;
                SA_part(DIM,F,F_T);
                
                
            }
            else
            {
                double Accept_P = exp( -(Trial_obj - Current_inf.Current_Best_Value)/sa_set.Temp);
                double Ran_P =  randomDouble();
                if( Accept_P >= Ran_P)
                {
                    Current_inf.Current_Best_Value = Trial_obj;
                    Current_inf.Current_Best_Coordinate = Trial_X;
                    sa_set.Temp = sa_set.Temp * sa_set.Dec_Temp;
                    sa_set.movesAccepted ++;
                    SA_part(DIM,F,F_T);
                }
            }

        }
        double SA_Vector()
        {
            double phi = sa_set.movesAccepted / sa_set.totalMoves;
            // cout<< phi <<endl;
            double V_bi = (max - min)/2 ;
            double V_step = 0.0;
            if(phi > 0.6)
            {
                V_step = 1+2*(phi -0.6)/0.6;
            }
            else if (phi < 0.4)
            {
                V_step = 1+2*(0.4 - phi)/0.4;
            }
            else{
                V_step = 1;
            }
            return V_step;
        }
        void OUTPUT(int DIM,int Function,int F_T,int MAX_NFE,int run,double START,double END)
        {
            double STD = 0;
            double AVG = 0;
            for(int i=0;i<run;i++)
            {
                AVG+= Each_Run_Result[i];
            }
            AVG /= run;
            for(int i=0;i<run;i++)
            {
                STD += pow(Each_Run_Result[i]-AVG,2);
                STD /= run;
                STD = sqrt(STD);
            }
            cout<<"# CEC Testing Function : "<<Function<<endl;
            cout<<"# CEC Transform : "<<F_T<<endl;
            cout<<"# Run : "<<run<<endl;
            cout<<"# Evaluation : "<<MAX_NFE<<endl;
            cout<<"# Dimension : "<<DIM<<endl;
            cout<<"# Average Objective Value :"<<endl<<AVG<<endl;
            cout<<"# Std :"<<endl<<STD<<endl;
            cout<<"# Execution Time :"<<endl<<(END - START) / CLOCKS_PER_SEC<<"(s)"<<endl;
            // CEC21::CEC_Classify(Function,F_T,START,END,STD,AVG,DIM);
            CEC17::CEC_Classify(Function,START,END,STD,AVG,DIM);
            // CEC21::Run_Classify(&Each_Run_Result[0],Function,F_T,run,DIM);
            CEC17::Run_Classify(&Each_Run_Result[0],Function,run,DIM);
        }
};