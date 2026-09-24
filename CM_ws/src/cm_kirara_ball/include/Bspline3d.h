 
























#pragma once
 
 
typedef struct tagPosition3d
{
    double  x;
    double  y;
	double  z;
	tagPosition3d(double _x,double _y,double  _z) { x=_x; y=_y;z=_z;}
	tagPosition3d() {};
	bool operator==(const tagPosition3d & pt) { return (x==pt.x && y==pt.y && z==pt.z);} 
} CPosition3d;
 
typedef struct tagPosition1d
{
    double  x;
	tagPosition1d(double _x) { x=_x;}
	tagPosition1d() {};
	bool operator==(const tagPosition1d & pt) { return (x==pt.x);} 
} CPosition1d;
 
typedef struct tagPosition6d
{
    double  x;
    double  y;
	double  z;
	double  f;
	double  g;
	double  h;

	tagPosition6d(double _x,double _y,double _z,double _f,double _g,double _h) { x=_x; y=_y;z=_z;f=_f; g=_g;h=_h;}
	tagPosition6d() {};
	bool operator==(const tagPosition6d & pt) { return (x==pt.x && y==pt.y && z==pt.z && f==pt.f && g==pt.g && h==pt.h);} 
} CPosition6d;
 
class CBSpline3d
{
	public:
		CBSpline3d(void);
		~CBSpline3d(void);
	
		 
		void TwoOrderBSplineInterpolatePt3d(CPosition3d *&pt,int &Num,int *InsertNum);
		void TwoOrderBSplineInterpolatePt1d(CPosition1d *&pt,int &Num,int *InsertNum);
		void TwoOrderBSplineInterpolatePt6d(CPosition6d *&pt,int &Num,int *InsertNum);
		double F02(double t);
		double F12(double t);
		double F22(double t);
	
};

 
 
CBSpline3d::CBSpline3d(void)
{
}
 
 
CBSpline3d::~CBSpline3d(void)
{
}


void CBSpline3d::TwoOrderBSplineInterpolatePt3d(CPosition3d *&pt,int &Num,int *InsertNum)
{
	if(pt==NULL || InsertNum==NULL) return;
 
	int InsertNumSum=0;                                
	for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];
 
	CPosition3d *temp=new CPosition3d[Num];                
	for(int i=0;i<Num;i++)
		temp[i]=pt[i];
 
	temp[0].x=2*temp[0].x-temp[1].x;                   
	temp[0].y=2*temp[0].y-temp[1].y;
	temp[0].z=2*temp[0].z-temp[1].z;

	temp[Num-1].x=2*temp[Num-1].x-temp[Num-2].x;
	temp[Num-1].y=2*temp[Num-1].y-temp[Num-2].y;
	temp[Num-1].z=2*temp[Num-1].z-temp[Num-2].z;

	delete []pt;                                       
 
	pt=new CPosition3d[Num+InsertNumSum];              
 
	CPosition3d NodePt1,NodePt2,NodePt3,NodePt4;         
 
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
				pt[totalnum].z=F02(t)*NodePt2.z+F12(t)*NodePt3.y+F22(t)*NodePt4.z;
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
				pt[totalnum].z=F02(t)*NodePt1.z+F12(t)*NodePt2.y+F22(t)*NodePt3.z;
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
				pt[totalnum].z=F02(t)*NodePt1.z+F12(t)*NodePt2.z+F22(t)*NodePt3.z;
				totalnum++;
			}
 
			for(int j=0;j<RightInsertNum;j++)
			{				
				t=rightoffset+Rightdt*j;
				pt[totalnum].x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
				pt[totalnum].y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
				pt[totalnum].z=F02(t)*NodePt2.z+F12(t)*NodePt3.z+F22(t)*NodePt4.z;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num=Num+InsertNumSum;
 
}

void CBSpline3d::TwoOrderBSplineInterpolatePt1d(CPosition1d *&pt,int &Num,int *InsertNum)
{
	if(pt==NULL || InsertNum==NULL) return;
 
	int InsertNumSum=0;                                
	for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];
 
	CPosition1d *temp=new CPosition1d[Num];                
	for(int i=0;i<Num;i++)
		temp[i]=pt[i];
 
	temp[0].x=2*temp[0].x-temp[1].x;                   


	temp[Num-1].x=2*temp[Num-1].x-temp[Num-2].x;

	delete []pt;                                       
 
	pt=new CPosition1d[Num+InsertNumSum];              
 
	CPosition1d NodePt1,NodePt2,NodePt3,NodePt4;         
 
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

				totalnum++;
			}
		}
		else if(i==Num-2)                             
		{
			NodePt1=temp[i-1]; NodePt2=temp[i]; NodePt3=temp[i+1];
 
			double dt=0.5/(InsertNum[i]+1);
			for(int j=0;j<InsertNum[i]+2;j++)
			{
				t=0.5+dt*j;
				pt[totalnum].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
				totalnum++;
			}
		}
		else                                      
		{
			NodePt1=temp[i-1],NodePt2=temp[i]; NodePt3=temp[i+1]; NodePt4=temp[i+2];     
 
			int LeftInsertNum,RightInsertNum;           
			double rightoffset=0;                       
			double Leftdt=0,Rightdt=0;                  
			if(InsertNum[i]==0 )
			{
				LeftInsertNum=0;
				RightInsertNum=0;
			}
			else if(InsertNum[i]%2==1)                 
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
				totalnum++;
			}
 
			for(int j=0;j<RightInsertNum;j++)
			{				
				t=rightoffset+Rightdt*j;
				pt[totalnum].x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num=Num+InsertNumSum;
 
}

void CBSpline3d::TwoOrderBSplineInterpolatePt6d(CPosition6d *&pt,int &Num,int *InsertNum)
{
	if(pt==NULL || InsertNum==NULL) return;
 
	int InsertNumSum=0;                                
	for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];
 
	CPosition6d *temp=new CPosition6d[Num];                
	for(int i=0;i<Num;i++)
		temp[i]=pt[i];
 
	temp[0].x=2*temp[0].x-temp[1].x;                   
	temp[0].y=2*temp[0].y-temp[1].y;
	temp[0].z=2*temp[0].z-temp[1].z;
	temp[0].f=2*temp[0].f-temp[1].f;                 
	temp[0].g=2*temp[0].g-temp[1].g;
	temp[0].h=2*temp[0].h-temp[1].h;

	temp[Num-1].x=2*temp[Num-1].x-temp[Num-2].x;
	temp[Num-1].y=2*temp[Num-1].y-temp[Num-2].y;
	temp[Num-1].z=2*temp[Num-1].z-temp[Num-2].z;
	temp[Num-1].f=2*temp[Num-1].f-temp[Num-2].f;
	temp[Num-1].g=2*temp[Num-1].g-temp[Num-2].g;
	temp[Num-1].h=2*temp[Num-1].h-temp[Num-2].h;

	delete []pt;                                       
 
	pt=new CPosition6d[Num+InsertNumSum];              
 
	CPosition6d NodePt1,NodePt2,NodePt3,NodePt4;         
 
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
				pt[totalnum].z=F02(t)*NodePt2.z+F12(t)*NodePt3.z+F22(t)*NodePt4.z;
				pt[totalnum].f=F02(t)*NodePt2.f+F12(t)*NodePt3.f+F22(t)*NodePt4.f;
				pt[totalnum].g=F02(t)*NodePt2.g+F12(t)*NodePt3.g+F22(t)*NodePt4.g;
				pt[totalnum].h=F02(t)*NodePt2.h+F12(t)*NodePt3.h+F22(t)*NodePt4.h;
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
				pt[totalnum].z=F02(t)*NodePt1.z+F12(t)*NodePt2.z+F22(t)*NodePt3.z;
				pt[totalnum].f=F02(t)*NodePt1.f+F12(t)*NodePt2.f+F22(t)*NodePt3.f;
				pt[totalnum].g=F02(t)*NodePt1.g+F12(t)*NodePt2.g+F22(t)*NodePt3.g;
				pt[totalnum].h=F02(t)*NodePt1.h+F12(t)*NodePt2.h+F22(t)*NodePt3.h;
				totalnum++;
			}
		}
		else                                      
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
				pt[totalnum].z=F02(t)*NodePt1.z+F12(t)*NodePt2.z+F22(t)*NodePt3.z;
				pt[totalnum].f=F02(t)*NodePt1.f+F12(t)*NodePt2.f+F22(t)*NodePt3.f;
				pt[totalnum].g=F02(t)*NodePt1.g+F12(t)*NodePt2.g+F22(t)*NodePt3.g;
				pt[totalnum].h=F02(t)*NodePt1.h+F12(t)*NodePt2.h+F22(t)*NodePt3.h;
				totalnum++;
			}
 
			for(int j=0;j<RightInsertNum;j++)
			{				
				t=rightoffset+Rightdt*j;
				pt[totalnum].x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
				pt[totalnum].y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
				pt[totalnum].z=F02(t)*NodePt2.z+F12(t)*NodePt3.z+F22(t)*NodePt4.z;
				pt[totalnum].f=F02(t)*NodePt2.f+F12(t)*NodePt3.f+F22(t)*NodePt4.f;
				pt[totalnum].g=F02(t)*NodePt2.g+F12(t)*NodePt3.g+F22(t)*NodePt4.g;
				pt[totalnum].h=F02(t)*NodePt2.h+F12(t)*NodePt3.h+F22(t)*NodePt4.h;
				totalnum++;
			}
		}
	}
	delete []temp;
	Num=Num+InsertNumSum;
 
}


double CBSpline3d::F02(double t)
{
	return 0.5*(t-1)*(t-1);
}
double CBSpline3d::F12(double t)
{
	return 0.5*(-2*t*t+2*t+1);
}
double CBSpline3d::F22(double t)
{
	return 0.5*t*t;
}
