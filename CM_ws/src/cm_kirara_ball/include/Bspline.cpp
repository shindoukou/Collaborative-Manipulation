 
 
 
 
 
 
 
 
#include "Bspline.h"
 
 
CBSpline::CBSpline(void)
{
}
 
 
CBSpline::~CBSpline(void)
{
}
 
 
 
 
 
 
 
 
void CBSpline::TwoOrderBSplineSmooth(CPosition *pt,int Num)
{
	CPosition *temp=new CPosition[Num];
	for(int i=0;i<Num;i++)
		temp[i]=pt[i];
 
	temp[0].x=2*temp[0].x-temp[1].x;                   
	temp[0].y=2*temp[0].y-temp[1].y;
 
	temp[Num-1].x=2*temp[Num-1].x-temp[Num-2].x;
	temp[Num-1].y=2*temp[Num-1].y-temp[Num-2].y;
 
	CPosition NodePt1,NodePt2,NodePt3;
	double t;
    for(int i=0;i<Num-2;i++)
	{
		NodePt1=temp[i]; NodePt2=temp[i+1]; NodePt3=temp[i+2];
		if(i==0)                                      
		{   
			t=0;
			pt[i].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
			pt[i].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
			t=0.5;
			pt[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
			pt[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
		}else if(i==Num-3)                           
		{
			t=0.5;
			pt[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
			pt[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
			t=1;
			pt[i+2].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
			pt[i+2].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
		}else                                       
		{
			t=0.5;
			pt[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
			pt[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
		}
	}
	delete []temp;
}
 
 
 
 
 
 
 
 
 
 
void CBSpline::TwoOrderBSplineInterpolatePt(CPosition *&pt,int &Num,int *InsertNum)
{
	if(pt==NULL || InsertNum==NULL) return;
 
	int InsertNumSum=0;                                
	for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];
 
	CPosition *temp=new CPosition[Num];                
	for(int i=0;i<Num;i++)
		temp[i]=pt[i];
 
	temp[0].x=2*temp[0].x-temp[1].x;                   
	temp[0].y=2*temp[0].y-temp[1].y;
 
	temp[Num-1].x=2*temp[Num-1].x-temp[Num-2].x;
	temp[Num-1].y=2*temp[Num-1].y-temp[Num-2].y;
 
	delete []pt;                                       
 
	pt=new CPosition[Num+InsertNumSum];              
 
	CPosition NodePt1,NodePt2,NodePt3,NodePt4;         
 
	double t;
	int totalnum=0;
	for(int i=0;i<Num-1;i++)                           
	{
		if(i==0)                                       
		{   
			NodePt2=temp[i]; NodePt3=temp[i+1]; NodePt4=temp[i+2];     
 
			double dt=0.5/(InsertNum[i]+1);
			for(int j=0;j<InsertNum[i]+1;j++)
			{
				t=0+dt*j;
				pt[totalnum].x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
				pt[totalnum].y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
				totalnum++;
			}
		}else if(i==Num-2)                             
		{
			NodePt1=temp[i-1]; NodePt2=temp[i]; NodePt3=temp[i+1];
 
			double dt=0.5/(InsertNum[i]+1);
			for(int j=0;j<InsertNum[i]+2;j++)
			{
				t=0.5+dt*j;
				pt[totalnum].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
				pt[totalnum].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
				totalnum++;
			}
		}else                                      
		{
			NodePt1=temp[i-1],NodePt2=temp[i]; NodePt3=temp[i+1]; NodePt4=temp[i+2];     
 
			int LeftInsertNum,RightInsertNum;           
			double rightoffset=0;                       
			double Leftdt=0,Rightdt=0;                  
			if(InsertNum[i]==0 )
			{
				LeftInsertNum=0;
				RightInsertNum=0;
			}else if(InsertNum[i]%2==1)                 
			{
				RightInsertNum=InsertNum[i]/2;
				LeftInsertNum=RightInsertNum+1;
				Leftdt=0.5/(LeftInsertNum);
				Rightdt=0.5/(RightInsertNum+1);
				rightoffset=Rightdt;
			}else                                       
			{
				RightInsertNum=InsertNum[i]/2;
				LeftInsertNum=RightInsertNum;
				Leftdt=0.5/(LeftInsertNum+0.5);
				Rightdt=0.5/(RightInsertNum+0.5);
				rightoffset=Rightdt/2;
			}
 
			for(int j=0;j<LeftInsertNum+1;j++)
			{
				t=0.5+Leftdt*j;
				pt[totalnum].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
				pt[totalnum].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
				totalnum++;
			}
 
			for(int j=0;j<RightInsertNum;j++)
			{				
				t=rightoffset+Rightdt*j;
				pt[totalnum].x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
				pt[totalnum].y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num=Num+InsertNumSum;
 
}
 
 
 
 
 
double CBSpline::F02(double t)
{
	return 0.5*(t-1)*(t-1);
}
double CBSpline::F12(double t)
{
	return 0.5*(-2*t*t+2*t+1);
}
double CBSpline::F22(double t)
{
	return 0.5*t*t;
}
 
 
 
 
 
 
 
 
void CBSpline::ThreeOrderBSplineSmooth(CPosition *pt,int Num)
{
	CPosition *temp=new CPosition[Num+2];
	for(int i=0;i<Num;i++)
		temp[i+1]=pt[i];
 
	temp[0].x=2*temp[1].x-temp[2].x;                   
	temp[0].y=2*temp[1].y-temp[2].y;
 
	temp[Num+1].x=2*temp[Num].x-temp[Num-1].x;
	temp[Num+1].y=2*temp[Num].y-temp[Num-1].y;
 
	CPosition NodePt1,NodePt2,NodePt3,NodePt4;
	double t;
	for(int i=0;i<Num-1;i++)
	{
		NodePt1=temp[i]; NodePt2=temp[i+1]; NodePt3=temp[i+2]; NodePt4=temp[i+3];
 
		if(i==Num-4)                           
		{
			t=0;
			pt[i].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
			pt[i].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
			t=1;
			pt[i+1].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
			pt[i+1].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
		}else                                       
		{
			t=0;
			pt[i].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
			pt[i].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
		}
	}
	delete []temp;
}
 
 
 
 
 
 
 
 
 
 
void CBSpline::ThreeOrderBSplineInterpolatePt(CPosition *&pt,int &Num,int *InsertNum)
{
	if(pt==NULL || InsertNum==NULL) return;
 
	int InsertNumSum=0;                                
	for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];
 
	CPosition *temp=new CPosition[Num+2];
	for(int i=0;i<Num;i++)
		temp[i+1]=pt[i];
 
	temp[0].x=2*temp[1].x-temp[2].x;                   
	temp[0].y=2*temp[1].y-temp[2].y;
 
	temp[Num+1].x=2*temp[Num].x-temp[Num-1].x;
	temp[Num+1].y=2*temp[Num].y-temp[Num-1].y;
 
	CPosition NodePt1,NodePt2,NodePt3,NodePt4;
	double t;
 
	delete []pt;                                       
 
	pt=new CPosition[Num+InsertNumSum];              
 
	int totalnum=0;
	for(int i=0;i<Num-1;i++)                           
	{
		NodePt1=temp[i]; NodePt2=temp[i+1]; NodePt3=temp[i+2]; NodePt4=temp[i+3];
		double dt=1.0/(InsertNum[i]+1);
 
		for(int j=0;j<InsertNum[i]+1;j++)
		{
			t=dt*j;
			pt[totalnum].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
			pt[totalnum].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
			totalnum++;
		}
 
		if(i==Num-2){               
			t=1;
			pt[totalnum].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
			pt[totalnum].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
			totalnum++;
		}
	}
 
	delete []temp;
	Num=Num+InsertNumSum;
 
}
 
 
 
 
 
 
double CBSpline::F03(double t)
{
	return 1.0/6*(-t*t*t+3*t*t-3*t+1);
}
double CBSpline::F13(double t)
{
	return 1.0/6*(3*t*t*t-6*t*t+4);
}
double CBSpline::F23(double t)
{
	return 1.0/6*(-3*t*t*t+3*t*t+3*t+1);
}
double CBSpline::F33(double t)
{
	return 1.0/6*t*t*t;
}