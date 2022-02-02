//=============================================================================
//SimplePend.cs Defines a class for simulating a simple pendulum
//=============================================================================
using System;

namespace Sim
{
    public class SimplePend
    {
        private double len = 1.1;  //pendulum length
        private double g =9.81;  //gravitational field strength

        int n = 2;
        private double [] x; //array of states
        private double [] f; //right side of eqn
        private double [] f2; //right side of eqn
        private double [] f3; //right side of eqn
        private double [] f4; //right side of eqn

        private double [] k1; //right side of eqn
        private double [] k2; //right side of eqn
        private double [] k3; //right side of eqn
        private double [] k4; //right side of eqn

        private double [] p1; //right side of eqn
        private double [] p2; //right side of eqn
        private double [] p3; //right side of eqn
        private double [] p4; //right side of eqn

        //--------------------------------------------------------------------
        //constructor
        //--------------------------------------------------------------------
        public SimplePend()
        {
            //Console.WriteLine("Inside Constructor");
            x = new double[n];
            f = new double[n];
            f2 = new double[n];
            f3 = new double[n];
            f4 = new double[n];
            k1 = new double[n];
            k2 = new double[n];
            k3 = new double[n];
            k4 = new double[n];
            p1 = new double[n];
            p2 = new double[n];
            p3 = new double[n];
            p4 = new double[n];
            

            x[0] = 1.0; //rad
            x[1] = 0.0; //omega starts from rest
        }

        //--------------------------------------------------------------------
        //step: perform one integration step via Eulers method
        //-------------------------------------------------------------------- 

        public void step(double dt)
        {
            //Console.WriteLine($"{x[0].ToString()} {x[1].ToString()}");
            rhsFunc(x,f);
            int i;
            for(i=0;i<n;++i)
            {
                x[i] = x[i] + f[i]*dt;            
            }
            Console.WriteLine($"{x[0].ToString()} {x[1].ToString()}");
        }
        //--------------------------------------------------------------------
        //rhsFunc: function to calculate rhs of pendulum odes
        //-------------------------------------------------------------------- 
        public void rhsFunc(double[] st, double[] ff)
        {
            ff[0] = st[1];
            ff[1] = -g/len * Math.Sin(st[0]);
        }

        public void stepRK4(double dt)
        {
            //Console.WriteLine($"{x[0].ToString()} {x[1].ToString()}");
            
            rhsFuncRK4(x,f);
            k1 = f;
            p1[0]= 0.5*k1[0]*dt +x[0];
            p1[1]= 0.5*k1[1]*dt +x[1];

            rhsFuncRK4(p1,f2);
            k2 = f2;
            p2[0]= 0.5*k2[0]*dt+x[0];
            p2[1]= 0.5*k2[1]*dt+x[1];

            rhsFuncRK4(p2,f3);
            k3= f3;
            p3[0]= k3[0]*dt+x[0];
            p3[1]= k3[1]*dt+x[1];

            
            rhsFuncRK4(p3,f4);
            k4 = f4;
 
            
            
            int i;
            for(i=0;i<n;++i)
            {   

                //x[i] = x[i] + (1/6)*dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);  
                x[i] = x[i] + (1.0/6.0)*dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
                    
            }
            //Console.WriteLine($"{x[0].ToString()} {x[1].ToString()}");
        }
        //--------------------------------------------------------------------
        //rhsFunc: function to calculate rhs of pendulum odes
        //-------------------------------------------------------------------- 
        public void rhsFuncRK4(double[] st, double[] ff)
        {
            ff[0] = st[1];
            ff[1] = -g/len * Math.Sin(st[0]);
        }

    
        
        //--------------------------------------------------------------------
        //Getters and setters
        //-------------------------------------------------------------------- 
        public double L
        {
            get{return(len);}
            set
            {
                if(value>0.0)
                    len=value;
            }
        }

                public double G
        {
            get{return(g);}
            set
            {
                if(value>=0.0)
                    g=value;
            }
        }
            public double theta
            {
                get{return x[0];}

                set{x[0]=value;}
            }

            
            public double thetaDot
            {
                get{return x[1];}

                set{x[1]=value;}
            }
    }  
}