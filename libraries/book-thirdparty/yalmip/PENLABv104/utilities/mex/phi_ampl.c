/* -----------------------------------------------

  phi.c 
  
  Penalty function for generalized augmented lagrangian function
  copied from Pennlp
----------------------------------------------- */
#include <math.h>

double phi2(double t)
  {
  extern double R;
  
  if(R < 0)
    {
    if (t < R)
      { 
      return (-(1+R)*(1+R)*log((1+2*R-t) / (1+R)) + R + .5*R*R);
      }
    
    else 
      return (t + .5*t*t);
    }
  
  else
    {
    if (t < R)
      { 
      return(-log(1-t));
      }
    
    else 
      return (((1 - 2*R)*t + .5*t*t - .5*(2*R - 3*R*R)) / (1 - R) / (1 - R) - log(1 - R));
    }
  
  }

double D_phi2(double t)
  {
  extern double R;
  
  if(R < 0)
    {
    if(t < R) 
      {
      return ((1+R)*(1+R) / (1+2*R-t));
      }
    
    else return (1. + t);
    }
  
  else
    {
    if(t < R) 
      {
      return (1 / (1-t));
      }
    
    else 
      return ((1 - 2*R + t) / (1 - R) / (1 - R));
    
    }
  }

double D2_phi2(double t)
  {
  extern double R;
  
  if(R < 0)
    {
    if(t < R)
      {
      return ((1+R)*(1+R) / ((1+2*R-t)*(1+2*R-t)));
      }  
    
    else 
      return (1.);
    }
  
  else
    {
    if(t < R)
      {
      return (1 / (1 - t) / (1 - t));
      }  
    
    else
      return (1 / (1 - R) / (1 - R));
    }
  }

  /* -----------------------------------------------
  
    for nonconvex functions
    
----------------------------------------------- */

double phi(double t)
  {
  extern double R;
  
  if(R < 0)
    {
    if (t < R)
      { 
      return ((-4*(1+R)*(1+R)*(1+R))/(t-3*R-2) - 2 - 3*R - 1.5*R*R);
      }
    
    else 
      return(t + .5*t*t);
    }
  
  else
    {
    if (t < R)
      { 
      return(4./(2-t) - 2);
      }
    
    else
      return(((8 - 12*R)*t + 4*t*t + 2*R*R*R) / (2 - R) / (2 - R) / (2 - R));
    }
  }

double D_phi(double t)
  {
  extern double R;
  
  if(R < 0)
    {
    if(t < R) 
      {
      return ((4*(1+R)*(1+R)*(1+R))/((t-3*R-2)*(t-3*R-2)));
      }
    
    else 
      return (1. + t);
    }
  
  else
    {
    if (t < R)
      { 
      return (4 / (2 - t) / (2 - t));
      }
    
    else 
      return ((8 - 12*R + 8*t) / (2 - R) / (2 - R) / (2 - R));
    }
  }

double D2_phi(double t)
  {
  extern double R;
  
  if(R < 0)
    {if(t < R)
    {
    return ((-8*(1+R)*(1+R)*(1+R))/((t-3*R-2)*(t-3*R-2)*(t-3*R-2)));
    }  
  
  else return (1.);
    }
  
  else
    {
    if (t < R)
      { 
      return (8 / (2 - t) / (2 - t) / (2 - t));
      }
    
    else 
      return (8 / (2 - R) / (2 - R) / (2 - R));
    }
  }

double phi_bar(double t)
  {
  if (t>0)
    return HUGE_VAL;
      return -log(-t);
  }

double D_phi_bar(double t)
  {
      return -1./t;
  }

double D2_phi_bar(double t)
  {
      return 1./(t*t);
  }

 /* -----------------------------------------------
  
    for strict functions
    
----------------------------------------------- */

double phi3(double t) {
//  if (t>1.9999999999999)
  if (t>0.9999999999999)
    return HUGE_VAL;

      return(-log(1-t));
//  return(4./(2. - t) - 2.);
}

double D_phi3(double t) {
//  return (4. / ((2. - t) * (2. - t)));
      return (1 / (1-t));
}

double D2_phi3(double t) {
      return (1 / (1 - t) / (1 - t));
//  return (8. / ((2. - t) * (2. - t) * (2. - t)));
}
