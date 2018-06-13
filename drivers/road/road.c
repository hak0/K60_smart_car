#include <math.h>
#define row_left_first  0
#define row_left_final  120
#define Threshold_ui 110
#define row_right_first 121
#define row_right_final 240
int i,j,temp,black_lost=0;
int left;
int right;
int *Get_Video;
int *VideoProcess;
int *V_offset;
int Blackleft[];
int Blackright[];
int BlackCenter[];
void road()
{

for(i=0;i<row_left_final;i++)      //左边黑线搜索范围
    for(j=0;j<COL;j++)
      {
	//Get_Video=VideoProcess+V_offset+j;
        if(Blackleft[i][j]-Blackleft[i+1][j]>Threshold_ui)   //下降沿
          {
		left=j;
		break;
	  }
      }
    for(i=121;i<240;i++)      //右边黑线搜索范围
      for(j=0;j<COL;j++)
      {
	//Get_Video=VideoProcess+V_offset+j;
	if(Blackright[i][j]-Blackright[i+1][j]>Threshold_ui)   //上升沿
	{
		right=j;
		break;
	}
      }
if((temp-Blackleft[i][j]<20)&&(temp-Blackleft[i][j]>-20))
//与上一场的该黑点位置进行比较，相差不大就更新
{
	Blackleft[i][j]=temp;                      
}
if((temp-Blackright[i][j]<20)&&(temp-Blackright[i][j]>-20))
//与上一场的该黑点位置进行比较，相差不大就更新
{
	Blackright[i][j]=temp;
}
temp=(Blackleft[i][j]+Blackright[i][j])/2;
if((temp-BlackCenter[i][j]<20)&&(temp-BlackCenter[i][j]>-20))
{
	BlackCenter[i][j]=temp;
	black_lost=0;                       //黑线丢失计数清零
}
}