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

        //--------------------------------------------------------------------
        //constructor
        //--------------------------------------------------------------------
        public SimplePend()
        {
            //Console.WriteLine("Inside Constructor");
            x = new double[n];
            f = new double[n];

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
            //Console.WriteLine($"{f[0].ToString()} {f[1].ToString()}");
        }
        //--------------------------------------------------------------------
        //rhsFunc: function to calculate rhs of pendulum odes
        //-------------------------------------------------------------------- 
        public void rhsFunc(double[] st, double[] ff)
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