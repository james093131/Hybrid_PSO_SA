#include "normal.hpp"

int main(int argc,const char *argv[])
{
    int run = atoi(argv[1]);
    int pop = atoi(argv[2]);
    int dim = atoi(argv[3]);
    int Function_Number = atoi(argv[4]);
    int Function_Transform = atoi(argv[5]);
    int MAX_NFE = atoi(argv[6]);


    
    if( argc > 1 )
    {
        PSO_SA pso_sa;
        pso_sa.run(run,dim,pop,Function_Number,Function_Transform,MAX_NFE);
    }
      
    else 
    {
        PSO_SA pso_sa;
        pso_sa.run(1,10,50,1,0,100000);

    }
   
    return 0;
}