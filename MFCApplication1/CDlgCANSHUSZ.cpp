// CDlgCANSHUSZ.cpp: 实现文件
//

#include "pch.h"
#include "MFCApplication1.h"
#include "CDlgCANSHUSZ.h"
#include "afxdialogex.h"
#include "string"
//#include <string>
#include <iostream>
#include <atlstr.h>
// CDlgCANSHUSZ 对话框

IMPLEMENT_DYNAMIC(CDlgCANSHUSZ, CDialogEx)




CDlgCANSHUSZ::CDlgCANSHUSZ(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DIALOG_CANSHUSZ, pParent)
{

}

CDlgCANSHUSZ::~CDlgCANSHUSZ()
{
}

void CDlgCANSHUSZ::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgCANSHUSZ, CDialogEx)
	ON_BN_CLICKED(IDOK, &CDlgCANSHUSZ::OnBnClickedOk)
END_MESSAGE_MAP()


// CDlgCANSHUSZ 消息处理程序
BOOL CDlgCANSHUSZ::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	
	//设置参数编辑框内容
	 
	//拟合平面距离参数
	GetDlgItem(IDC_EDIT4)->SetWindowTextW(dis_plane);//拟合平面距离参数

	//统计异常值删除过滤器参数
	GetDlgItem(IDC_MeanK)->SetWindowTextW(MeanK);
	GetDlgItem(IDC_StddevMulThresh)->SetWindowTextW(StddevMulThresh);

	//半径异常值删除过滤器参数
	GetDlgItem(IDC_SearchRadius)->SetWindowTextW(SearchRadius);
	GetDlgItem(IDC_MinNeighborsInRadius)->SetWindowTextW(MinNeighborsInRadius);

	//边界提取参数Alpha
	GetDlgItem(IDC_Alpha)->SetWindowTextW(Alpha);

	//设置拟合直线段参数
	GetDlgItem(IDC_threshold)->SetWindowTextW(threshold);
	GetDlgItem(IDC_max_iterations)->SetWindowTextW(max_iterations);
	GetDlgItem(IDC_point_size)->SetWindowTextW(point_size);

	//设置线段焊接间距参数
	GetDlgItem(IDC_dis_weld)->SetWindowTextW(dis_weld);

	//区域增长法参数
	GetDlgItem(IDC_RegionGrowing_K)->SetWindowTextW(RegionGrowing_K);
	GetDlgItem(IDC_RegionGrowing_Neighbours)->SetWindowTextW(RegionGrowing_Neighbours);
	GetDlgItem(IDC_RegionGrowing_SmoothnessThreshold)->SetWindowTextW(RegionGrowing_SmoothnessThreshold);
	GetDlgItem(IDC_RegionGrowing_CurvatureThreshold)->SetWindowTextW(RegionGrowing_CurvatureThreshold);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CDlgCANSHUSZ::OnBnClickedOk()
{
	//获取平面拟合距离参数
	GetDlgItem(IDC_EDIT4)->GetWindowText(dis_plane); 
	//统计异常值删除过滤器参数
	GetDlgItem(IDC_MeanK)->GetWindowText(MeanK);
	GetDlgItem(IDC_StddevMulThresh)->GetWindowText(StddevMulThresh);
	
	//半径异常值删除参数
	GetDlgItem(IDC_SearchRadius)->GetWindowText(SearchRadius);
	GetDlgItem(IDC_MinNeighborsInRadius)->GetWindowText(MinNeighborsInRadius);

	//提取边界参数
	GetDlgItem(IDC_Alpha)->GetWindowText(Alpha);
	
	//获取拟合直线段参数
	GetDlgItem(IDC_threshold)->GetWindowText(threshold);
	GetDlgItem(IDC_max_iterations)->GetWindowText(max_iterations);
	GetDlgItem(IDC_point_size)->GetWindowText(point_size);

	//获取线段焊接间距参数
	GetDlgItem(IDC_dis_weld)->GetWindowText(dis_weld);

	//获取区域增长参数
	GetDlgItem(IDC_RegionGrowing_K)->GetWindowTextW(RegionGrowing_K);
	GetDlgItem(IDC_RegionGrowing_Neighbours)->GetWindowTextW(RegionGrowing_Neighbours);
	GetDlgItem(IDC_RegionGrowing_SmoothnessThreshold)->GetWindowTextW(RegionGrowing_SmoothnessThreshold);
	GetDlgItem(IDC_RegionGrowing_CurvatureThreshold)->GetWindowTextW(RegionGrowing_CurvatureThreshold);
	
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();




}
