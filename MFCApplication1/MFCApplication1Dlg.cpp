
// MFCApplication1Dlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "MFCApplication1.h"
#include "MFCApplication1Dlg.h"
#include "afxdialogex.h"

#include "CDlgCANSHUSZ.h"  //用于创建对话框CDlgCANSHUSZ，参数设置对话框
#include "Dlg_format_conversion.h"

#include<string>
#include<map> 
#include <pcl/filters/project_inliers.h>  //投影点云

#include<iostream> //数据输入、输出流
#include<boost/filesystem.hpp> //文件系统，用于将多段线写入记事本

#include <pcl/visualization/mouse_event.h> //鼠标事件
#include<pcl/visualization/keyboard_event.h>//键盘事件

#include <pcl/filters/radius_outlier_removal.h>//radius_outlier_removal去噪

#include <iostream>
#include <fstream>



//计算法线


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp> 



//点云分割涉及头文件
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>


#ifdef _DEBUG
//#define new DEBUG_NEW
#endif


CDlgCANSHUSZ dlg_csss; //定义参数设置对话框



//定义框选选择点云的索引
pcl::IndicesPtr indices_Ptr_global;

//定义点选点云
pcl::PointXYZ point_selcted;

boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;//要共享指针类型的，要不然，显示窗口会跳出MFC界面
                                                              //不能声明在头文件中，如果声明在头文件中，则回调函数内不能范围该变量

CTreeCtrl m_Tree;



// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton_open();
	afx_msg void pcdToXYZ();
	
	
	afx_msg void On32776();
	afx_msg void OnFiltering_YZ();
	afx_msg void OnProject();
	afx_msg void OnRadiusOutlierRemovalFilter();
	afx_msg void OncircleLeastFit();
	afx_msg void OnAccuracyAnalysis();
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &CAboutDlg::OnBnClickedButton_open)
	ON_COMMAND(ID_32773, &CAboutDlg::pcdToXYZ)
	

	ON_COMMAND(ID_32776, &CAboutDlg::On32776)
	ON_COMMAND(ID_32777, &CAboutDlg::OnFiltering_YZ)
	ON_COMMAND(ID_32778, &CAboutDlg::OnProject)
	ON_COMMAND(ID_32779, &CAboutDlg::OnRadiusOutlierRemovalFilter)
	ON_COMMAND(ID_32780, &CAboutDlg::OncircleLeastFit)
	ON_COMMAND(ID_32781, &CAboutDlg::OnAccuracyAnalysis)
END_MESSAGE_MAP()


// CMFCApplication1Dlg 对话框

CMFCApplication1Dlg::CMFCApplication1Dlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MFCAPPLICATION1_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMFCApplication1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TREE1, m_Tree);
	DDX_Control(pDX, IDC_LIST1, ListBox_Point);
}

BEGIN_MESSAGE_MAP(CMFCApplication1Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CMFCApplication1Dlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON2, &CMFCApplication1Dlg::OnBnClickedButton2)
	ON_WM_SIZE()
	ON_BN_CLICKED(IDC_BUTTON1, &CMFCApplication1Dlg::OnBnClickedButton1)
	ON_COMMAND(id_menu_sssz, &CMFCApplication1Dlg::Onmenusssz)
	ON_COMMAND(ID_32773, &CAboutDlg::pcdToXYZ)
	
	ON_COMMAND(ID_32776, &CAboutDlg::On32776) //圆柱提取，点云分割菜单响应程序  
	ON_COMMAND(ID_32777, &CAboutDlg::OnFiltering_YZ) //圆柱提取，点云去噪菜单响应程序
	ON_COMMAND(ID_32778, &CAboutDlg::OnProject) //圆柱提取，点云投影菜单响应程序
	ON_COMMAND(ID_32779, &CAboutDlg::OnRadiusOutlierRemovalFilter) //圆柱提取，点云半径滤波菜单响应程序
	ON_COMMAND(ID_32780, &CAboutDlg::OncircleLeastFit) //圆柱提取，最小二乘法拟合圆菜单响应程序
	ON_COMMAND(ID_32781, &CAboutDlg::OnAccuracyAnalysis) //圆柱提取，精度分析菜单响应程序
	
	
	//On32774
	ON_COMMAND(ID_32774, &CMFCApplication1Dlg::OnRadiusOutlierRemoval)
	ON_NOTIFY(NM_CLICK, IDC_TREE1, &CMFCApplication1Dlg::OnNMClickTree1)
	ON_BN_CLICKED(IDC_delete, &CMFCApplication1Dlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_DNoise, &CMFCApplication1Dlg::OnBnClickedDnoise)
	ON_NOTIFY(NM_RCLICK, IDC_TREE1, &CMFCApplication1Dlg::OnNMRClickTree1)
	ON_BN_CLICKED(IDC_savePC, &CMFCApplication1Dlg::OnBnClickedsavepc)
	ON_BN_CLICKED(IDC_BUTTON6, &CMFCApplication1Dlg::OnBnClickedButton6)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON7, &CMFCApplication1Dlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_projection_plane, &CMFCApplication1Dlg::OnBnClickedprojectionplane)
	ON_BN_CLICKED(IDC_boundary, &CMFCApplication1Dlg::OnBnClickedboundary)
	ON_BN_CLICKED(IDC_cluster, &CMFCApplication1Dlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_cluster, &CMFCApplication1Dlg::OnBnClickedcluster)
	ON_BN_CLICKED(IDC_line, &CMFCApplication1Dlg::OnBnClickedline)
	ON_BN_CLICKED(IDC_BUTTON4, &CMFCApplication1Dlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_AddPoint, &CMFCApplication1Dlg::OnBnClickedAddpoint)
	ON_BN_CLICKED(IDC_saveCADcommand, &CMFCApplication1Dlg::OnBnClickedsavecadcommand)
	ON_COMMAND(ID_32774, &CMFCApplication1Dlg::OnRadiusOutlierRemoval)
	ON_BN_CLICKED(IDC_Normal, &CMFCApplication1Dlg::OnBnClickedNormal)
	ON_BN_CLICKED(IDC_BUTTON5, &CMFCApplication1Dlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON8, &CMFCApplication1Dlg::OnBnClickedButton8)
END_MESSAGE_MAP()


//框选点云回调函数，按x键后，框选点云，x键是 框选点云 的 开关
void areapickingcallback(const pcl::visualization::AreaPickingEvent& event, void* userdata)
{

	std::vector<int> indices;
	event.getPointsIndices(indices);
	//pcl::IndicesPtr ind_plane = boost::make_shared<std::vector<int>>(indices);

	pcl::IndicesPtr index_ptr(new std::vector<int>(indices));

	indices_Ptr_global = NULL;

	indices_Ptr_global = index_ptr;

	
}

//点选点云回调函数，shift+鼠标左键选点
void pointpickingcallback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	

	if (event.getPointIndex() == -1)
		return;
	pcl::PointXYZ pt;

	event.getPoint(pt.x, pt.y, pt.z);

	point_selcted = pt;

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_selected(new pcl::PointCloud<pcl::PointXYZ>);

	point_selected->points.push_back(pt);

	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> red(point_selected,255,0,0);

	//data->viewerPtr->removePointCloud("clicked_points");

	//data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	//
	//data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

	m_viewer->removeShape("point_selected");
	m_viewer->addPointCloud(point_selected, red, "point_selected");
	
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point_selected");
}


// CMFCApplication1Dlg 消息处理程序

BOOL CMFCApplication1Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码


	//初始化参数
	dis_plane = 0.08;//拟合平面的参数，点到平面的距离
	//统计异常值删除过滤器参数
	para.MeanK = 20;
	para.StddevMulThresh = 3.9;

	//半径异常值删除过滤器参数
	para.SearchRadius = 0.04;//搜索半径
	para.MinNeighborsInRadius = 45;//半径内最少点数

	//提取点云边界参数
	para.Alpha = 0.1;

	//初始化拟合直线段参数
	para.threshold = 0.03;
	para.max_iterations = 1000;
	para.point_size = 20;

	//初始化线段焊接间距参数 
	para.dis_weld = 1;

	//初始化区域增长算法参数
	para.RegionGrowing_K = 32;
	para.RegionGrowing_Neighbours = 30;
	para.RegionGrowing_SmoothnessThreshold = 7;
	para.RegionGrowing_CurvatureThreshold = 1;

	//初始化参数设置对话框	
	dlg_csss.dis_plane.Format(_T("%lf"), dis_plane);
	//统计异常值删除过滤器参数
	dlg_csss.MeanK.Format(_T("%lf"), para.MeanK);
	dlg_csss.StddevMulThresh.Format(_T("%lf"), para.StddevMulThresh);

	//半径异常值删除过滤器参数
	dlg_csss.SearchRadius.Format(_T("%lf"), para.SearchRadius);
	dlg_csss.MinNeighborsInRadius.Format(_T("%lf"), para.MinNeighborsInRadius);
	//边界提取参数
	dlg_csss.Alpha.Format(_T("%lf"), para.Alpha);

	//初始化拟合直线段参数
	dlg_csss.threshold.Format(_T("%lf"),para.threshold);  //%lf   double转CString
	dlg_csss.max_iterations.Format(_T("%d"),para.max_iterations); //%d int转CString
	dlg_csss.point_size.Format(_T("%d"),para.point_size);

	//初始化线段焊接间距
	dlg_csss.dis_weld.Format(_T("%lf"), para.dis_weld);

	//初始化参数设置窗口 区域增长函数
	dlg_csss.RegionGrowing_K.Format(_T("%d"), para.RegionGrowing_K);
	dlg_csss.RegionGrowing_Neighbours.Format(_T("%d"), para.RegionGrowing_Neighbours);
	dlg_csss.RegionGrowing_SmoothnessThreshold.Format(_T("%lf"), para.RegionGrowing_SmoothnessThreshold);
	dlg_csss.RegionGrowing_CurvatureThreshold.Format(_T("%lf"), para.RegionGrowing_CurvatureThreshold);

	//初始化点云窗口
	CRect rect;
	GetClientRect(&rect);//实时获取MFC窗口大小
	CWnd* bWnd = GetDlgItem(IDC_BUTTON2); //获取按钮
	bWnd->SetWindowPos(NULL, rect.Width() - 50 - 85, 50, 85, 25, SWP_SHOWWINDOW); //设置按钮位置

	CWnd* pWnd = GetDlgItem(IDC_PCDVIEW);//参数为控件ID, 获取Picture Control的id 

	CRect rc;
	//pWnd->SetWindowPos(NULL, 0, 200, rect.Width() - 200, rect.Height()-50, SWP_NOMOVE); //设置 Picture Control 的大小
	
	pWnd->MoveWindow(192, 1, rect.Width() - 200-192, rect.Height()-2, 1);
	pWnd->GetClientRect(&rc);//rc为控件的大小。

	 //pcl显示窗口初始化
	m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));//初始化viewer对象
	//m_viewer->addCoordinateSystem(); //设置对应的坐标系
	m_viewer->setBackgroundColor(0, 0, 0);//设置背景颜色
	//m_viewer->initCameraParameters();//初始化相机的参数
	m_win = m_viewer->getRenderWindow();//将view中的渲染窗口的句柄传递给vtk window
	m_iren = vtkRenderWindowInteractor::New(); //初始化vtkwindow交互的对象 
	m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作

	m_viewer->registerAreaPickingCallback(areapickingcallback);//注册 AreaPicking 事件

	m_viewer->registerPointPickingCallback(pointpickingcallback);//注册 PointPicking 事件

	m_win->SetSize(rc.right - rc.left, rc.bottom - rc.top);//根据当前窗口的大小设置vtk 窗口的大小


	CWnd* viewer_pcWnd;
	viewer_pcWnd = this->GetDlgItem(IDC_PCDVIEW);//获取对应的wnd
	m_win->SetParentId(viewer_pcWnd->m_hWnd);//设置vtk窗口的句柄
	m_iren->SetRenderWindow(m_win);//将vtk交互对象与vtk window绑定 
	m_viewer->createInteractor();
	m_win->Render();//开始渲染	

	
		//初始化菜单
	CMenu menu;
	menu.LoadMenu(IDR_MENU1);
	SetMenu(&menu);

	//初始化树控件
	CWnd* tWnd = GetDlgItem(IDC_TREE1);//参数为控件ID, 获取树控件的句柄
	tWnd->MoveWindow(1,1,190,rect.Height()-2,1);

	////进度条显示区域
	////设置进度条的范围
	//m_progress.SetRange(0, 100);
	////设置进度条当前的位置
	//m_progress.SetPos(75);
	////获取当前进度条的位置
	////progress_pos = m_progress.GetPos();
	////设置进度条每次步进的长度
	//m_progress.SetStep(1);


	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}



void CMFCApplication1Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CMFCApplication1Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
	
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CMFCApplication1Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

////////////////////////////////////////////
void CAboutDlg::OnBnClickedButton_open()
{
	
	 //TODO: 在此添加控件通知处理程序代码
	
}

void CMFCApplication1Dlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	

	CDialogEx::OnOK();
}


void CMFCApplication1Dlg::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);


	// TODO: 在此处添加消息处理程序代码

	CRect rect;
	GetClientRect(&rect);//实时获取MFC窗口大小


	CRect rc;
	CWnd* pWnd = GetDlgItem(IDC_PCDVIEW);//参数为控件ID
	if (pWnd != NULL && m_win != NULL)
	{

		pWnd->MoveWindow(192, 1, rect.Width() - 200 - 192-85, rect.Height() - 2, 1);

		pWnd->GetClientRect(&rc);//rc为控件的大小。

		m_win->SetSize(rc.right - rc.left, rc.bottom - rc.top);//根据当前窗口的大小设置vtk 窗口的大小

	}

	//按钮控件
	CWnd* bWnd = GetDlgItem(IDC_BUTTON2);
	if (bWnd != NULL)
	{
		bWnd->SetWindowPos(NULL, rect.Width() - 50 - 85-85-40, 50, 85, 25, SWP_SHOWWINDOW);
	}
	CWnd* bWnd1 = GetDlgItem(IDC_BUTTON1);
	if (bWnd1!=NULL)
	{
		bWnd1->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 80, 85, 25, SWP_SHOWWINDOW);
	}
	CWnd* bWnd2 = GetDlgItem(IDC_DNoise);
	if (bWnd2 != NULL)
	{
		bWnd2->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 110, 85, 25, SWP_SHOWWINDOW);
	}
	CWnd* bWndIDC_projection_plane = GetDlgItem(IDC_projection_plane);

	if (bWndIDC_projection_plane != NULL)
	{
		bWndIDC_projection_plane->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 140, 85, 25, SWP_SHOWWINDOW);
	}

	CWnd* bWndIDC_boundary = GetDlgItem(IDC_boundary);
	if (bWndIDC_boundary != NULL)
	{
		bWndIDC_boundary->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 170, 85, 25, SWP_SHOWWINDOW);
	}
	
	CWnd* bWndIDC_cluster = GetDlgItem(IDC_cluster);
	if (bWndIDC_cluster != NULL)
	{
		bWndIDC_cluster->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 200, 85, 25, SWP_SHOWWINDOW);
	}

	CWnd* bWndIDC_line = GetDlgItem(IDC_line);
	if (bWndIDC_line != NULL)
	{
		bWndIDC_line->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 230, 85, 25, SWP_SHOWWINDOW);
	}
	//生成多段线按钮
	CWnd* bWndIDC_BUTTON7 = GetDlgItem(IDC_BUTTON7);
	if (bWndIDC_BUTTON7 != NULL)
	{
		bWndIDC_BUTTON7->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 260, 85, 25, SWP_SHOWWINDOW);
	}
	//导出多段线按钮
	CWnd* bWndIDC_BUTTON4 = GetDlgItem(IDC_BUTTON4);
	if (bWndIDC_BUTTON4 != NULL)
	{
		bWndIDC_BUTTON4->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 290, 85, 25, SWP_SHOWWINDOW);
	}
	//保存点云按钮		
	CWnd* bWnd3 = GetDlgItem(IDC_savePC);
	if (bWnd3 != NULL)
	{
		bWnd3->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40,330, 85, 25, SWP_SHOWWINDOW);
	}
	//移除点云按钮
	CWnd* bWndIDC_delete = GetDlgItem(IDC_delete);
	if (bWndIDC_delete != NULL)
	{
		bWndIDC_delete->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 360, 85, 25, SWP_SHOWWINDOW);
	}

	//测试按钮
	CWnd* bWndIDC_BUTTON6 = GetDlgItem(IDC_BUTTON6);
	if (bWndIDC_BUTTON6 != NULL)
	{
		bWndIDC_BUTTON6->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 390, 85, 25, SWP_SHOWWINDOW);
	}

	////存储点文本框

	//CWnd* bWndIDC_Point = GetDlgItem(IDC_Point);
 //   if (bWndIDC_Point != NULL)
 //   {
	//  // bWndIDC_Point->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 410, 85, 25, SWP_SHOWWINDOW);

	//   bWndIDC_Point->MoveWindow(rect.Width() - 50 - 85 - 85 - 40, 430,240,300);
 //   }

	CWnd* bWndIDC_Point = GetDlgItem(IDC_LIST1);
	if (bWndIDC_Point != NULL)
	{
		// bWndIDC_Point->SetWindowPos(NULL, rect.Width() - 50 - 85 - 85 - 40, 410, 85, 25, SWP_SHOWWINDOW);

		bWndIDC_Point->MoveWindow(rect.Width() - 50 - 85 - 85 - 40, 430, 240, 300);
	}

	//添加点按钮
	CWnd* bWndIDC_AddPoint = GetDlgItem(IDC_AddPoint);
	if (bWndIDC_AddPoint != NULL)
	{
		bWndIDC_AddPoint->SetWindowPos(NULL, rect.Width() - 50 - 85, 80, 85, 25, SWP_SHOWWINDOW);
	}

	//移除点按钮
	CWnd* bWndIDC_RemovePoint = GetDlgItem(IDC_RemovePoint);
	if (bWndIDC_RemovePoint != NULL)
	{
		bWndIDC_RemovePoint->SetWindowPos(NULL, rect.Width() - 50 - 85, 110, 85, 25, SWP_SHOWWINDOW);
	}

	//保存点按钮
	CWnd* bWndIDC_SavePoint = GetDlgItem(IDC_SavePoint);
	if (bWndIDC_SavePoint != NULL)
	{
		bWndIDC_SavePoint->SetWindowPos(NULL, rect.Width() - 50 - 85, 140, 85, 25, SWP_SHOWWINDOW);
	}

	//保存CAD（画线）命令 IDC_saveCADcommand
	CWnd* bWndIDC_saveCADcommand = GetDlgItem(IDC_saveCADcommand);
	if (bWndIDC_saveCADcommand != NULL)
	{
		bWndIDC_saveCADcommand->SetWindowPos(NULL, rect.Width() - 50 - 85, 170, 85, 25, SWP_SHOWWINDOW);
	}


	//树控件

	CWnd* tWnd = GetDlgItem(IDC_TREE1);//参数为控件ID, 获取树控件的句柄
	if (tWnd != NULL)
	{
		tWnd->MoveWindow(1, 1, 190, rect.Height() - 2, 1);
	}


}

//求取选中节点父节点的顺序号
int item_to_pXuhao(HTREEITEM item)
{
	////求父节点序号begin
	HTREEITEM pitem = m_Tree.GetParentItem(item);

	HTREEITEM subItem = m_Tree.GetChildItem(pitem);
	int p_cnt = 0; //cnt+1父节点的序号
	while (subItem != NULL)
	{
		p_cnt++;
		CString cs = m_Tree.GetItemText(subItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		subItem = m_Tree.GetNextItem(subItem, TVGN_NEXT);
	}

	////求父节点符号end

	return p_cnt;
}

//求取本节点的顺序号
int item_to_sXuhao(HTREEITEM item)
{
	/// </求本节点序号begin>
	HTREEITEM subItem = m_Tree.GetChildItem(item);
	int s_cnt = 0; //cnt+1父节点的序号
	while (subItem != NULL)
	{
		s_cnt++;
		CString cs = m_Tree.GetItemText(subItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		subItem = m_Tree.GetNextItem(subItem, TVGN_NEXT);
	}
	s_cnt++;
	/// </求本节点序号end>

	return s_cnt;
}

//求选中节点的深度
int item_to_depth(HTREEITEM item)
{
	int depth;
	depth = 0;
	if (item==m_Tree.GetRootItem())
	{
		depth = 0;
		return depth;
	}

	HTREEITEM pitem = m_Tree.GetParentItem(item);

	depth++;

	while (pitem!=m_Tree.GetRootItem())
	{
		pitem = m_Tree.GetParentItem(pitem);
		
		depth++;
	}

	return depth;
	
}

//参数设置菜单项
void CMFCApplication1Dlg::Onmenusssz()
{
	// TODO: 在此添加命令处理程序代码



	//CString text;
	//text.Format(_T("%7.4f"), dis_plane);

	//将参数传递给设置参数窗口
	CString dis_plane_cs;;
	dis_plane_cs.Format(_T("%f"),dis_plane); //double转Cstring
	dlg_csss.dis_plane = dis_plane_cs;

	//统计异常值删除过滤器参数
	CString MeanK_cs;
	MeanK_cs.Format(_T("%f"), para.MeanK);
	dlg_csss.MeanK = MeanK_cs;

	CString StddevMulThresh_cs;
	StddevMulThresh_cs.Format(_T("%f"), para.StddevMulThresh);
	dlg_csss.StddevMulThresh = StddevMulThresh_cs;

	//半径异常值删除过滤器参数
	CString SearchRadius_cs;
	SearchRadius_cs.Format(_T("%f"), para.SearchRadius);
	dlg_csss.SearchRadius = SearchRadius_cs;
	CString MinNeighborsInRadius_cs;
	MinNeighborsInRadius_cs.Format(_T("%f"),para.MinNeighborsInRadius);
	dlg_csss.MinNeighborsInRadius = MinNeighborsInRadius_cs;
	
	//边界提取参数

	CString Alpha_cs;
	Alpha_cs.Format(_T("%f"), para.Alpha);
	dlg_csss.Alpha = Alpha_cs;

	//拟合直线段参数传递给设置参数窗口
	CString threshold_cs;
	threshold_cs.Format(_T("%lf"),para.threshold);
	dlg_csss.threshold = threshold_cs;

	CString max_iterations_cs;
	max_iterations_cs.Format(_T("%d"),para.max_iterations);
	dlg_csss.max_iterations = max_iterations_cs;

	CString point_size_cs;
	point_size_cs.Format(_T("%d"),para.point_size);
	dlg_csss.point_size = point_size_cs;
	
	//线段焊接间距参数
	CString dis_weld_cs;
	dis_weld_cs.Format(_T("%lf"), para.dis_weld);
	dlg_csss.dis_weld = dis_weld_cs;

	//设置区域增长参数
	CString temp;

	temp.Format(_T("%d"), para.RegionGrowing_K);
	dlg_csss.RegionGrowing_K = temp;

	temp.Format(_T("%d"), para.RegionGrowing_Neighbours);
	dlg_csss.RegionGrowing_Neighbours = temp;

	temp.Format(_T("%lf"), para.RegionGrowing_CurvatureThreshold);
	dlg_csss.RegionGrowing_CurvatureThreshold = temp;

	temp.Format(_T("%lf"), para.RegionGrowing_SmoothnessThreshold);
	dlg_csss.RegionGrowing_SmoothnessThreshold = temp;


	//获取参数设置后的参数值
	dlg_csss.DoModal();

	//获取参数
	//获取点云计算参数
	CString  dish;
	dish = dlg_csss.dis_plane;
	dis_plane = _tstof(dish);

	
	temp = dlg_csss.MeanK;
	para.MeanK = _tstof(temp);

	temp = dlg_csss.StddevMulThresh;
	para.StddevMulThresh = _tstof(temp);
	
	temp = dlg_csss.SearchRadius;
	para.SearchRadius= _tstof(temp);

	temp = dlg_csss.MinNeighborsInRadius;
	para.MinNeighborsInRadius= _tstof(temp);

	temp = dlg_csss.Alpha;
	para.Alpha = _tstof(temp);

	temp = dlg_csss.threshold;
	para.threshold = _tstof(temp);

	temp = dlg_csss.max_iterations;
	para.max_iterations = _tstof(temp);

	temp = dlg_csss.point_size;
	para.point_size = _tstof(temp);

	temp = dlg_csss.dis_weld;
	para.dis_weld = _tstof(temp);	

	//区域增长参数
	temp = dlg_csss.RegionGrowing_K;
	para.RegionGrowing_K = _tstof(temp);

	temp = dlg_csss.RegionGrowing_Neighbours;
	para.RegionGrowing_Neighbours = _tstof(temp);

	temp = dlg_csss.RegionGrowing_SmoothnessThreshold;
	para.RegionGrowing_SmoothnessThreshold = _tstof(temp);

	temp = dlg_csss.RegionGrowing_CurvatureThreshold;
	para.RegionGrowing_CurvatureThreshold = _tstof(temp);


}


//打开点云
void CMFCApplication1Dlg::OnBnClickedButton2() 
{
	// TODO: 在此添加控件通知处理程序代码

	CString strFile = _T("");
	CFileDialog    dlgFile(TRUE, NULL, NULL, OFN_HIDEREADONLY, _T("Describe Files (*.pcd)|*.pcd|All Files (*.*)|*.*||"), NULL);

	CString filename;

	filename = dlgFile.GetFileName();

	std::string fn(CW2A(filename.GetString()));
	 
	if (dlgFile.DoModal())
	{
		strFile = dlgFile.GetPathName();

		//Cstring 转string	
		std::string fn(CW2A(dlgFile.GetFileTitle().GetString()));
	
		std::string STDStr(CW2A(strFile.GetString()));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(STDStr, *cloud) == -1)//*打开点云文件
		{
			AfxMessageBox(_T("读入点云数据失败"));
		}
		int key;
		key = 1002; // 1002含义为：1树的深度（为第一层，即跟节点），0父节点（0为没有），0左节点（0为没有），2右节点（2为第二个）
		if (m_Tree.GetCount()>0)
		{
			key = 1002 + m_Tree.GetCount() +m_Tree.GetCount()*10;
		}
		//定义指针名称结构，用于存储点云的名称和指针
		ptrName pn;
		pn.cloud_name = fn;
		pn.cloud_ptr = cloud;

		//将点云存储到图mapCloud中
		mapCloud.insert(mp::value_type(key, pn)); 

		//pcl::PointCloud<pcl::PointXYZ>::Ptr dd = mapCloud.at(12);

		//m_viewer->removeAllPointClouds();//将前一次点云移除  
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
		m_viewer->addPointCloud<pcl::PointXYZ >(cloud, single_color,fn);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作


		//将点云名字添加到树控件中
		
		HTREEITEM treeItem=	m_Tree.InsertItem(CA2T(fn.c_str()));//存放在根节点中
		m_Tree.SetCheck(treeItem, 1);

	}
	
}


//分割点云
void CMFCApplication1Dlg::OnBnClickedButton1()  
{
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM item = m_Tree.GetSelectedItem();
	std::string cloud_name;
	int key_cloud;

	//向树控件中添加剩余点云项和分割点云项
	if (item != NULL)
	{
		cloud_name = "plane1";
		key_cloud = 2103;
		int i = 1;
		//如果没有剩余点云项，则插入剩余点云项，如果有则不插入此项
		CString text = m_Tree.GetItemText(item);

		if (text!="remain")
		{
			HTREEITEM item_remain= m_Tree.InsertItem(_T("remain"), item); //向树控件中插入剩余点云节点
			m_Tree.SetCheck(item_remain, 1);
			HTREEITEM item_plane= m_Tree.InsertItem(CA2T(cloud_name.c_str()), item); //在剩余点云项后插入提取点云平面项
			m_Tree.SetCheck(item_plane, 1);
			m_Tree.Expand(item, TVE_EXPAND);
		}
		else
		{			
			HTREEITEM xitem = m_Tree.GetNextSiblingItem(item);
			
			while (xitem != NULL)
			{
				xitem = m_Tree.GetNextSiblingItem(xitem);
				i++;
				key_cloud++;
			}
			cloud_name = "plane" + std::to_string(i);
			HTREEITEM item_plane=m_Tree.InsertItem(CA2T(cloud_name.c_str()),m_Tree.GetParentItem(item));
			m_Tree.SetCheck(item_plane, 1);
			m_Tree.Expand(item, TVE_EXPAND);
		}
	}


	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);
		//由树控件项文本，获取点云编码key
		int key = name_to_key(text);
		//由key获取点云指针
		cloud_plane_coefficient cpc;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mapCloud.at(key).cloud_ptr;
		//分割提取点云
		cpc = cloud_to_plane(cloud, dis_plane);

		//定义指针名称结构，用于存储点云的名称和指针
		ptrName pn;
		pn.cloud_name = cloud_name;
		pn.cloud_ptr = cpc.cloud_plane;
		pn.plane_coefficient = cpc.plane_coefficient;


		//将分割点云存储到图mapCloud中
		mapCloud.insert(mp::value_type(key_cloud, pn)); // 1002含义为：2树的深度（2为第二层），0父节点（0为没有），0左节点（0为没有），2右节点（2为第二个）

		//显示分割点云
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cpc.cloud_plane, 255, 0, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(cpc.cloud_plane, single_color, pn.cloud_name);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作

		//显示分割剩余点云
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cpc.remain_cloud, 0, 255, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(cpc.remain_cloud, single_color2, "remain");
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作
	
		//将剩余点云存储到map图中
		int key_remain = name_to_key(_T("remain"));

		if (key_remain!=NULL) mapCloud.erase(key_remain);

		ptrName pn_remain;
		pn_remain.cloud_name = "remain" ;
		pn_remain.cloud_ptr = cpc.remain_cloud;

		//将点云存储到图mapCloud中
		mapCloud.insert(mp::value_type(2102, pn_remain)); // 1002含义为：2树的深度（2为第二层），0父节点（0为没有），0左节点（0为没有），2右节点（2为第二个）

		//HTREEITEM rootitem= m_Tree.GetRootItem();		
    } 	
	else
	{
		AfxMessageBox(_T("请选择要分割的点云！！"));
	}
	
}



// 树控件左键单击事件//处理树控件选项的勾选事件
void CMFCApplication1Dlg::OnNMClickTree1(NMHDR* pNMHDR, LRESULT* pResult)
{
	// TODO: 在此添加控件通知处理程序代码

	CPoint I_point;
	UINT uFlag;

	GetCursorPos(&I_point);

	m_Tree.ScreenToClient(&I_point);

	HTREEITEM slectItem = m_Tree.HitTest(I_point, &uFlag);

	if (uFlag == 64) //uFlag=64：点中树枝右面的复选框，uFlag=16:点中树枝节点，uFlag=8:点中的是叶节点的折叠处，uFlag=2：点中复选框右面的灯泡，uFlag=4：点中灯泡右面的文字 
	{
		m_Tree.SelectItem(slectItem);

		CString text = m_Tree.GetItemText(slectItem);
		
		std::string text_string(CW2A(text.GetString())); //cstring 转string



		if (!m_Tree.GetCheck(slectItem)) //勾选
		{
			int key = name_to_key(text);
			
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(mapCloud.at(key).cloud_ptr, 255, 0, 0);
 	
			m_viewer->addPointCloud(mapCloud.at(key).cloud_ptr, single_color, mapCloud.at(key).cloud_name);
	
		}
		else
		{
			m_viewer->removePointCloud(text_string);
		}
	}
	
	*pResult = 0;
}

//移除点云button
void CMFCApplication1Dlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码

	//m_viewer->removeAllPointClouds();//将前一次点云移除  
	//移除选中的点云
	HTREEITEM item=	m_Tree.GetSelectedItem();

	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);

		//int key = _ttoi(text); //将Cstring转换为int
		int key = name_to_key(text);

		std::string fn = mapCloud.at(key).cloud_name;
		m_viewer->removePointCloud(fn);
		mapCloud.erase(key);
	}
	m_Tree.DeleteItem(item);

}

//去除噪声button--统计异常值删除滤波器
void CMFCApplication1Dlg::OnBnClickedDnoise()
{
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM item = m_Tree.GetSelectedItem();

	////求父节点序号begin
	HTREEITEM pitem =m_Tree.GetParentItem(item);

	HTREEITEM subItem = m_Tree.GetChildItem(pitem);
	int p_cnt = 0; //cnt+1父节点的序号
	while (subItem != NULL)
	{
		p_cnt++;
		CString cs = m_Tree.GetItemText(subItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		subItem = m_Tree.GetNextItem(subItem, TVGN_NEXT);
	}
	
	////求父节点符号end

	/// </求本节点序号begin>
	subItem = m_Tree.GetChildItem(item);
	int s_cnt = 0; //cnt+1父节点的序号
	while (subItem != NULL)
	{
		s_cnt++;
		CString cs = m_Tree.GetItemText(subItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		subItem = m_Tree.GetNextItem(subItem, TVGN_NEXT);
	}
	s_cnt++;
	/// </求本节点序号end>
	
	//本点云key
	int key_self = 3000 + p_cnt * 100 + (s_cnt - 1) * 10 + (s_cnt + 1);
	


	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);

		std::string text_s(CW2A(text.GetString()));

		//获取点云的key
		int key_select = name_to_key(text);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mapCloud.at(key_select).cloud_ptr;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		cloud_filtered=statistical_outlier(cloud);

		ptrName pn_filter;
		pn_filter.cloud_name = text_s +"_filtered_"+ std::to_string(key_self);
		pn_filter.cloud_ptr = cloud_filtered;

		//将点云存储到图mapCloud中
		mapCloud.insert(mp::value_type(key_self,pn_filter));

		HTREEITEM item_filtered = m_Tree.InsertItem(CA2T(pn_filter.cloud_name.c_str()),item);
		m_Tree.SetCheck(item_filtered, 1);
		m_Tree.Expand(item, TVE_EXPAND);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 255, 0, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(cloud_filtered, single_color, pn_filter.cloud_name);



	}

}

//树控件右键单击事件
void CMFCApplication1Dlg::OnNMRClickTree1(NMHDR* pNMHDR, LRESULT* pResult)
{
	// TODO: 在此添加控件通知处理程序代码

	CPoint I_point;
	UINT uFlag;

	GetCursorPos(&I_point);

	m_Tree.ScreenToClient(&I_point);

	HTREEITEM slectItem = m_Tree.HitTest(I_point, &uFlag);

	if (uFlag == 4)
	{
		m_Tree.SelectItem(slectItem);

		CString text = m_Tree.GetItemText(slectItem);
		//std::string text_string(CW2A(text.GetString())); //cstring 转string
		
		int key = name_to_key(text);
		//设置点云的颜色，设置为绿色的
		if (key == -1) //如果key=-1,即认为是线段，不是点云
		{
			std::string id(CW2A(text.GetString()));// CString转string
			
			m_viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0, 255, 0, id);
		}
		else //设置点云颜色
		{
			m_viewer->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0, 255, 0, mapCloud.at(key).cloud_name);
		}
    }
	*pResult = 0;
}

//保存选中的点云
void CMFCApplication1Dlg::OnBnClickedsavepc()
{
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM item = m_Tree.GetSelectedItem();

	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);
		//获取点云的key
		int key = name_to_key(text);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mapCloud.at(key).cloud_ptr;

		BOOL isOpen = FALSE;        //是否打开(否则为保存)
		CString defaultDir = L"G:\\PCL\\data\\B-002";    //默认打开的文件路径
		CString fileName = L"test.pcd";            //默认打开的文件名
		CString filter = L"文件 (*.pcd)|*.pcd||";    //文件过虑的类型
		CFileDialog openFileDlg(isOpen, defaultDir, fileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, filter, NULL);
		openFileDlg.GetOFN().lpstrInitialDir = L"G:\\PCL\\Data\\B-002\\test.pcd";
		INT_PTR result = openFileDlg.DoModal();
		CString filePath = defaultDir + "\\" + fileName;
		if (result ==IDOK) {
			filePath = openFileDlg.GetPathName();

			std::string fp(CW2A(filePath.GetString())); //Cstring转string
			
			pcl::io::savePCDFileASCII(fp, *cloud);

		}
		
	}

}

//框选点云
void CMFCApplication1Dlg::OnBnClickedButton6()
{
	// TODO: 在此添加控件通知处理程序代码
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_select(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	extract.setInputCloud(mapCloud.at(3213).cloud_ptr);
	extract.setIndices(indices_Ptr_global);
	extract.setNegative(false);
	extract.filter(*cloud_select);

	//m_viewer->removePointCloud("cloud_select");

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_select, 0, 0, 255);
	//m_viewer->addPointCloud<pcl::PointXYZ >(cloud_select, single_color, "cloud_select");

	//将边界点云分割为若干个直线点云
	double dis_max = para.threshold;//点到模型的距离
	int pointC =cloud_select->size() / para.point_size; //最少为点云数量的1/10
	std::list<Line_pointCloud_coefficients> lpc_list;
	lpc_list = segment_to_lineCloud(cloud_select, dis_max, pointC);

	//通过直线点云 获取 拟合线段 构成线段数组
	std::vector<line> lines = cloudline_to_lines(lpc_list);

	
	//将拟合的线段添加到显示窗口
	for (size_t i = 0; i < lines.size(); i++)
	{
		std::string id = "select_line_"  + std::to_string(i);
		m_viewer->removeShape(id);
		m_viewer->addLine(lines[i].pt1, lines[i].pt2, 0, 0, 1, id);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作

	}
}

//计时器，用于控制进度条，暂时停止使用
void CMFCApplication1Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	

	//m_progress.ShowWindow(SW_SHOW);
			//进度条走一个单位
			m_progress.StepIt();
			CRect rect;
			GetClientRect(&rect);//实时获取MFC窗口大小
			//CWnd* pWnd = GetDlgItem(IDC_PROGRESS1);//参数为控件ID
			//if (pWnd != NULL)
			//{
			//	pWnd->MoveWindow(rect.Width()/6, rect.Height()/2-25,rect.Width()*2/3, 25, FALSE);
			//}



	CDialogEx::OnTimer(nIDEvent);
}



//将平面点云投影到采样平面上
void CMFCApplication1Dlg::OnBnClickedprojectionplane()
{
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM item = m_Tree.GetSelectedItem();

	
	// 本点云key
	int key_self = 3000 + item_to_pXuhao(item) * 100 + (item_to_sXuhao(item) - 1) * 10 + (item_to_sXuhao(item) + 1);

	//向树控件中添加剩余点云项和分割点云项
	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);
		//由树控件项文本，获取点云编码key
		int key = name_to_key(text);
		//由key获取点云指针
		cloud_plane_coefficient cpc;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mapCloud.at(key).cloud_ptr;

		pcl::ModelCoefficients::Ptr coefficients=mapCloud.at(key).plane_coefficient;

		if (coefficients ==NULL)
		{
			//cloud_plane_coefficient cloud_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double dis)
			cloud_plane_coefficient cpc = cloud_to_plane(cloud, 0.5);
			coefficients= cpc.plane_coefficient;
		}


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

		// 具体投影计算就是着一小坨代码了
		pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影器
		proj.setModelType(pcl::SACMODEL_PLANE);//选择要投影的是平面
		proj.setInputCloud(cloud);//输入投影前的点云
		proj.setModelCoefficients(coefficients);//输入平面的参数
		proj.filter(*cloud_projected);//输出投影后的点云


		//将投影点云存储到图中		

		std::string text_s(CW2A(text.GetString()));

		ptrName ptrName_prj;
		ptrName_prj.cloud_name = text_s + "_prj_" + std::to_string( key_self);
		ptrName_prj.cloud_ptr = cloud_projected;
		ptrName_prj.plane_coefficient = coefficients;
		mapCloud.insert(mp::value_type(key_self,ptrName_prj));

		//将点云名称插入到树控件中
		HTREEITEM item_prj = m_Tree.InsertItem(CA2T(ptrName_prj.cloud_name.c_str()),item); //在剩余点云项后插入提取点云平面项
		m_Tree.SetCheck(item_prj, 1);
		m_Tree.Expand(item, TVE_EXPAND);

		//将投影点云显示在view控件中

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_projected, 255, 0, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(cloud_projected, single_color2, ptrName_prj.cloud_name);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作		
	}


}

//提取 投影后的点云 的边界
void CMFCApplication1Dlg::OnBnClickedboundary()
{
	// TODO: 在此添加控件通知处理程序代码

	HTREEITEM item = m_Tree.GetSelectedItem();

	//向树控件中添加剩余点云项和分割点云项
	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);
		//由树控件项文本，获取点云编码key
		int key = name_to_key(text);

		ptrName pnc = mapCloud.at(key);

		pcl::PointCloud<pcl::PointXYZ>::Ptr boundary;

		boundary = cloud_to_boundary(pnc.cloud_ptr);

		//将投影点云存储到图中		

		int f = key % 100 - 1;//父节点顺序号

		int key_s = item_to_sXuhao(m_Tree.GetParentItem(item));
		//本节点编号
		int key_prj = 3000 + item_to_pXuhao(m_Tree.GetParentItem(item)) * 100+ (key_s -1)*10+(key_s +1);//第三层，父节点的第一个子节点

		std::string text_s(CW2A(text.GetString()));

		ptrName ptrName_prj;
		ptrName_prj.cloud_name = text_s + "_bdy" + std::to_string(key_prj);
		ptrName_prj.cloud_ptr =boundary;
		ptrName_prj.plane_coefficient =pnc.plane_coefficient;
		mapCloud.insert(mp::value_type(key_prj, ptrName_prj));

		//将点云名称插入到树控件中
		HTREEITEM item_prj = m_Tree.InsertItem(CA2T(ptrName_prj.cloud_name.c_str()), m_Tree.GetParentItem(item)); //在剩余点云项后插入提取点云平面项
		m_Tree.SetCheck(item_prj, 1);
		m_Tree.Expand(item, TVE_EXPAND);

		//将投影点云显示在view控件中

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(boundary, 255, 0, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(boundary, single_color2, ptrName_prj.cloud_name);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作	
	}
	//cloud_to_boundary

}

//边界聚类分析，划分不同的点云线边界
void CMFCApplication1Dlg::OnBnClickedButton10()
{
	// TODO: 在此添加控件通知处理程序代码

	//std::vector < pcl::PointCloud <pcl::PointXYZ>::Ptr> euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered)
	HTREEITEM item = m_Tree.GetSelectedItem();
	//向树控件中添加剩余点云项和分割点云项
	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);
		//由树控件项文本，获取点云编码key
		int key = name_to_key(text);
		ptrName pnc = mapCloud.at(key);

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;

		cloud_vector = euclidean_cluster_extraction(pnc.cloud_ptr);

		std::string t_s(CW2A(text.GetString()));
		std::string cluster_name;
		cluster_name = t_s + "_cluster";
		HTREEITEM item_cl_z = m_Tree.InsertItem(CA2T(cluster_name.c_str()), m_Tree.GetParentItem(item));
		m_Tree.SetCheck(item_cl_z, 1);

		int key_cluster = 4102; //第一个直线点云的编码

		int cnt_color = 255 / cloud_vector.size();

		for (size_t i = 0; i < cloud_vector.size(); i++)
		{
			//将聚类后的边界点云，添加到map图中
			ptrName ptrName_prj;
		
			ptrName_prj.cloud_name = t_s + "_cluster_" + std::to_string(key_cluster);
			ptrName_prj.cloud_ptr = cloud_vector[i];
			ptrName_prj.plane_coefficient = pnc.plane_coefficient;
			mapCloud.insert(mp::value_type(key_cluster, ptrName_prj));

			key_cluster = key + 11 + i;//同层级的下一个节点

			//将点云名称插入到树控件中
			HTREEITEM item_clu = m_Tree.InsertItem(CA2T(ptrName_prj.cloud_name.c_str()), item_cl_z); //在剩余点云项后插入提取点云平面项
			m_Tree.SetCheck(item_clu, 1);
			m_Tree.Expand(item_clu, TVE_EXPAND);

			//将投影点云显示在view控件中

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_vector[i], 0, (i + 1) * cnt_color , 0);
			m_viewer->addPointCloud<pcl::PointXYZ >(cloud_vector[i], single_color2, ptrName_prj.cloud_name);
			m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作	

		}

		//std::string t_s(CW2A(text.GetString()));
		//std::string c_n = t_s + "_cluster";
		////将点云名称插入到树控件中
		//HTREEITEM item_clu = m_Tree.InsertItem(CA2T(c_n.c_str()), m_Tree.GetParentItem(item)); //在剩余点云项后插入提取点云平面项
		//m_Tree.SetCheck(item_clu, 1);
		//m_Tree.Expand(item_clu, TVE_EXPAND);
		
	}
}


void CMFCApplication1Dlg::OnBnClickedcluster()
{

}



//提取边界线段
void CMFCApplication1Dlg::OnBnClickedline()
{
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM item = m_Tree.GetSelectedItem();
	CString text_item = m_Tree.GetItemText(item);
	//由树控件项文本，获取点云编码key
	int key_item = name_to_key(text_item);

	//向树控件中添加剩余点云项和分割点云项
	if (item != NULL)
	{
		//将线段的父节点插入到树控件中
		HTREEITEM item_lines = m_Tree.InsertItem(_T("lines"),m_Tree.GetParentItem(item)); //在剩余点云项后插入提取点云平面项
		m_Tree.SetCheck(item_lines, 1);
		m_Tree.Expand(item_lines, TVE_EXPAND);

		

		HTREEITEM sub_item = m_Tree.GetChildItem(item);

		int key_lines = 4202;//确定第一个线段组的key
		
		
		while (sub_item != NULL)
		{
			CString text = m_Tree.GetItemText(sub_item);
			//由树控件项文本，获取点云编码key
			int key = name_to_key(text);
			ptrName pnc = mapCloud.at(key);

			//将边界点云分割为若干个直线点云
			double dis_max = para.threshold;//点到模型的距离
			int pointC = pnc.cloud_ptr->size() / para.point_size; //最少为点云数量的1/10
			std::list<Line_pointCloud_coefficients> lpc_list;
			lpc_list = segment_to_lineCloud(pnc.cloud_ptr, dis_max, pointC);

			//通过直线点云 获取 拟合线段 构成线段数组
			std::vector<line> lines = cloudline_to_lines(lpc_list);

			//将拟合的线段添加map

			ptrName ptrName_lines;

			std::string cloud_name = "line_" + std::to_string( key_lines);

			ptrName_lines.cloud_name = cloud_name;

			//ptrName_lines.cloud_name = "cswb";

			ptrName_lines.lines = lines;
			mapCloud.insert(mp::value_type(key_lines, ptrName_lines));

			//将拟合的线段添加到显示窗口和树控件
			for (size_t i = 0; i < lines.size(); i++)
			{
				std::string id = "line_" + std::to_string(key_lines) + "_" + std::to_string(i);
				m_viewer->addLine(lines[i].pt1, lines[i].pt2, 1, 0, 0, id);
				m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作


				HTREEITEM item_line = m_Tree.InsertItem(CA2T(id.c_str()), item_lines); //在剩余点云项后插入提取点云平面项
				m_Tree.SetCheck(item_line, 1);
				m_Tree.Expand(item_line, TVE_EXPAND);
			}

			//将线段组1插入到树控件中b


			sub_item = m_Tree.GetNextItem(sub_item, TVGN_NEXT);

			key_lines = key_lines + 11;

		}

	}

}

//生成多段下焊接线段
void CMFCApplication1Dlg::OnBnClickedButton7()
{
	// TODO: 在此添加控件通知处理程序代码
		
	HTREEITEM item = m_Tree.GetSelectedItem();

	HTREEITEM parentItem = m_Tree.GetParentItem(item);

	HTREEITEM subItem=m_Tree.GetChildItem(parentItem);
	
	//添加多断线虚节点
	int cnt=0; 
	while (subItem!=NULL)
	{
		cnt++;
		CString cs = m_Tree.GetItemText(subItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		subItem = m_Tree.GetNextItem(subItem,TVGN_NEXT);
	}
	
	// cnt为多断线虚节点节点序号

	//求多段线虚节点父节点序号
	HTREEITEM sub_parentItem;
	if (parentItem != m_Tree.GetRootItem())
	{
		sub_parentItem = m_Tree.GetChildItem(m_Tree.GetParentItem(parentItem));
	}
	else
	{
		sub_parentItem = m_Tree.GetChildItem(m_Tree.GetRootItem());
	}

	int cnt_p = 0; //cnt_p多段线虚节点父节点序号
	while (sub_parentItem != NULL)
	{
		cnt_p++;
		CString cs = m_Tree.GetItemText(sub_parentItem);
		//subItem = NULL;
		//subItem = m_Tree.GetNextSiblingItem(subItem);
		sub_parentItem = m_Tree.GetNextItem(sub_parentItem, TVGN_NEXT);
	}
	
	int key_p = 3000 + (cnt_p+1) * 100 + (cnt - 1) * 10 + (cnt + 1);


	//向树控件添加多线段父节点

	std::string item_p = "polyLine";

	HTREEITEM item_new_p_0 = m_Tree.InsertItem(CA2T(item_p.c_str()), parentItem);
	m_Tree.SetCheck(item_new_p_0, 1);
	m_Tree.Expand(item_new_p_0, TVE_EXPAND);

	//定义多线段节点
	HTREEITEM item_new_p;
	//定义第一个多线段key
	int key = 4000 + (cnt + 1) * 100 + 02;
		

	std::map<int, ptrName>::const_iterator mp_Iter;

	//遍历map，查找含有lines的记录
	for (mp_Iter = mapCloud.begin();mp_Iter!=mapCloud.end(); mp_Iter++) 
	{
		std::vector<line> lines = (&mp_Iter._Ptr->_Myval)->second.lines;
		
		if (lines.size() > 0)
		{

			//有提取的线段构造多断线
			std::vector<pcl::PointXYZ> polyline = lines_to_ployline(lines);


			//将多段线添加到点云显示窗口

			if (polyline.size()>0)
			{
				std::string name = "polyLine_" + std::to_string(key);

				for (int j = 0; j < polyline.size() - 1; j++)
				{
					m_viewer->addLine(polyline[j], polyline[j + 1], 1, 0, 0, name + std::to_string(j));
				}

				//将多段线添加到map
				ptrName ptrName_polyLine;

				ptrName_polyLine.cloud_name = name;

			    ptrName_polyLine.polyline = polyline;

				mapCloud.insert(mp::value_type(key, ptrName_polyLine));

				//将多段线添加到树控件

				item_new_p = m_Tree.InsertItem(CA2T(name.c_str()), item_new_p_0);
				m_Tree.SetCheck(item_new_p, 1);
				m_Tree.Expand(item_new_p, TVE_EXPAND);
				key = key + 11;

			}
			
		}
		
		m_Tree.Expand(parentItem, TVE_EXPAND);

	}

}

//导出多段线

void CMFCApplication1Dlg::OnBnClickedButton4()
{
	// TODO: 在此添加控件通知处理程序代码

	CString filter;
	CString defaultDir;
	CString defaultFileName;
	defaultDir = "D: / ";
	defaultFileName = ".txt";
	filter = "txt(.txt)";

	CString str;

	CFileDialog dlg(TRUE, defaultDir, defaultFileName, OFN_HIDEREADONLY, filter); //打开对话框
	if (dlg.DoModal() == IDOK)
	{
		str = dlg.GetPathName(); // 获取文件路径
	}
		
	std::string fileName(CW2A(str.GetString())); //Cstring 转string	
	
	
	//将多段线导出到记事本
	std::ofstream ofs;
		
	ofs.open(fileName, ios::out);

	//遍历map查找多段线导出

	std::map<int, ptrName>::const_iterator mp_Iter;

	int cnt = 0;
	//遍历map，查找含有polyline的记录
	for (mp_Iter = mapCloud.begin(); mp_Iter != mapCloud.end(); mp_Iter++)
	{
		std::vector<pcl::PointXYZ> polyLine= (&mp_Iter._Ptr->_Myval)->second.polyline;
		
		if (polyLine.size()>0)
		{
			
			//ofs << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << endl;

			ofs << cnt << "," ;
			
			std::string s;
			for (size_t i = 0; i < polyLine.size(); i++)
			{
 				s =float_3_string(polyLine[i].x) ;

				ofs << s << ",";

				s = float_3_string(polyLine[i].y);

				ofs << s << ",";

				s = float_3_string(polyLine[i].z);

				ofs << s << ",";
			}

			ofs << endl;

			cnt++;
		}
		
	}

	ofs.close();
}


//将选中的点坐标添加到文本框
void CMFCApplication1Dlg::OnBnClickedAddpoint()
{
	// TODO: 在此添加控件通知处理程序代码
	point_selcted;

	if (!(point_selcted.x==0&&point_selcted.y==0&&point_selcted.z==0))
	{
		CString cs;

		int cnt = ListBox_Point.GetCount();
		
		cs.Format(_T("%d"), cnt);

		CString temp;

		temp = float_3_string(point_selcted.x).c_str();

		cs = cs + "," + temp;

		temp = float_3_string(point_selcted.y).c_str();
		
		cs = cs + "," + temp;

		temp = float_3_string(point_selcted.z).c_str();

		cs = cs + "," + temp;
		
		ListBox_Point.AddString(cs);

		CString s;

		ListBox_Point.GetText(0, s);

	}

}

//菜单，格式转换-》PCD转XYZ 响应事件
void CAboutDlg::pcdToXYZ()
{
	// TODO: 在此添加命令处理程序代码
	Dlg_format_conversion dfc;
	dfc.DoModal();
	
	
}

//导出CAD绘制多段命令
void CMFCApplication1Dlg::OnBnClickedsavecadcommand()
{
	// TODO: 在此添加控件通知处理程序代码

	//line 2.182,0.002,0.450 1.571,0.287,18.644 -0.012,21.946,18.195 2.182,0.002,0.450  
	//最后是两个空格
	
	CString filter;
	CString defaultDir;
	CString defaultFileName;
	defaultDir = "D: / ";
	defaultFileName = ".txt";
	filter = "txt(.txt)";

	CString str;

	CFileDialog dlg(TRUE, defaultDir, defaultFileName, OFN_HIDEREADONLY, filter); //打开对话框
	if (dlg.DoModal() == IDOK)
	{
		str = dlg.GetPathName(); // 获取文件路径
	}

	std::string fileName(CW2A(str.GetString())); //Cstring 转string	


	//将多段线导出到记事本
	std::ofstream ofs;

	ofs.open(fileName, ios::out);

	//遍历map查找多段线导出

	std::map<int, ptrName>::const_iterator mp_Iter;

	int cnt = 0;
	//遍历map，查找含有polyline的记录
	for (mp_Iter = mapCloud.begin(); mp_Iter != mapCloud.end(); mp_Iter++)
	{
		std::vector<pcl::PointXYZ> polyLine = (&mp_Iter._Ptr->_Myval)->second.polyline;

		if (polyLine.size() > 0)
		{

			//ofs << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << endl;

			ofs << "line ";

			std::string s;
			for (size_t i = 0; i < polyLine.size(); i++)
			{
				s = float_3_string(polyLine[i].x);

				ofs << s << ",";

				s = float_3_string(polyLine[i].y);

				ofs << s << ",";

				s = float_3_string(polyLine[i].z);

				ofs << s << " ";
			}

			ofs << " ";

			ofs << endl;

			cnt++;
		}

	}

	ofs.close();


}



void CMFCApplication1Dlg::OnRadiusOutlierRemoval()
{
	// TODO: 在此添加命令处理程序代码

	HTREEITEM item = m_Tree.GetSelectedItem();
//
	int depth = item_to_depth(item);//选中节点的深度
	int pXuhao = item_to_pXuhao(item);//父节点序号
	int sXuhao = item_to_sXuhao(item);//本节点的序号


	if (item != NULL)
	{
		CString text = m_Tree.GetItemText(item);

		//获取点云的key
		int key_select = name_to_key(text);

		pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud = mapCloud.at(key_select).cloud_ptr;

		pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOR;

		radiusOR.setInputCloud(inCloud);

		radiusOR.setRadiusSearch(para.SearchRadius);

		radiusOR.setMinNeighborsInRadius(para.MinNeighborsInRadius);

		radiusOR.setKeepOrganized(true);

		radiusOR.filter(*filterCloud);


		int key = (depth + 1) * 1000 + pXuhao * 100 + (sXuhao - 1) * 10 + (sXuhao + 1);

		//将投影点云存储到图中		

		std::string text_s(CW2A(text.GetString()));

		ptrName ptrName_filter;
		ptrName_filter.cloud_name = text_s + "_radiusOR_" + std::to_string(key);
		ptrName_filter.cloud_ptr = filterCloud;
		
		mapCloud.insert(mp::value_type(key, ptrName_filter));

		//将点云名称插入到树控件中
		HTREEITEM item_prj = m_Tree.InsertItem(CA2T(ptrName_filter.cloud_name.c_str()), item); //在剩余点云项后插入提取点云平面项
		m_Tree.SetCheck(item_prj, 1);
		m_Tree.Expand(item, TVE_EXPAND);

		//将投影点云显示在view控件中

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(filterCloud, 255, 0, 0);
		m_viewer->addPointCloud<pcl::PointXYZ >(filterCloud, single_color2, ptrName_filter.cloud_name);
		m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作	

	}


}


void CAboutDlg::On32776() //点云分割菜单响应程序
{
	// TODO: 在此添加命令处理程序代码

	
	
	 //获取参数设置窗口中设置的区域增长参数
	CString temp;
	temp = dlg_csss.RegionGrowing_K;
	int RegionGrowing_K = _tstof(temp);

	temp = dlg_csss.RegionGrowing_Neighbours;
	int RegionGrowing_Neighbours = _tstof(temp);

	temp = dlg_csss.RegionGrowing_SmoothnessThreshold;
	double RegionGrowing_SmoothnessThreshold = _tstof(temp);

	temp = dlg_csss.RegionGrowing_CurvatureThreshold;
	double RegionGrowing_CurvatureThreshold = _tstof(temp);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("F:\\PCL\\DATA\\column\\SdjyColumnL2.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		//return(-1);
	}

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(RegionGrowing_K); //计算法线半径

	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::removeNaNFromPointCloud(*cloud, *indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(RegionGrowing_Neighbours); //邻域点数
	reg.setInputCloud(cloud);
	reg.setIndices(indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(RegionGrowing_SmoothnessThreshold / 180.0 * M_PI); //以弧度为单位设置角度，该角度将用作法线偏差的允许范围
	reg.setCurvatureThreshold(RegionGrowing_CurvatureThreshold); //曲率阈值

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

	//将投影点云显示在view控件中

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(filterCloud, 255, 0, 0);
	//m_viewer->addPointCloud<pcl::PointXYZ >(filterCloud, single_color2, ptrName_filter.cloud_name);
	//m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作
	m_viewer->removeAllPointClouds();
	m_viewer->addPointCloud(colored_cloud);
	m_viewer->resetCamera();


	//	//存储分割后的点云，待分割成功后使用
	std::size_t counter = 0;
	while (counter < clusters.size())
	{	
		pcl::ExtractIndices<pcl::PointXYZ> ext;
		PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new PointCloud<pcl::PointXYZ>);
		ext.setInputCloud(cloud);
		ext.setNegative(false);
		std::vector<int> inlier = clusters[counter].indices;
		pcl::IndicesPtr index_ptr(new std::vector<int>(inlier));/// 将自定义的pi数组进行智能指针的转换
		ext.setIndices(index_ptr);
		ext.filter(*rest_cloud);
		//保存点云
		std::string fp = "F:\\PCL\\DATA\\column\\" + std::to_string(counter) + ".pcd";
		pcl::io::savePCDFileASCII(fp, *rest_cloud);

		counter++;
	}
		
}



void CAboutDlg::OnFiltering_YZ() //圆柱提取--点云去噪统计滤波
{
	// TODO: 在此添加命令处理程序代码
	 
	//获取参数设置窗口中设置的统计异常值过滤参数
	CString temp;
	temp = dlg_csss.MeanK;
	double MeanK = _tstof(temp);
	temp = dlg_csss.StddevMulThresh;
	double StddevMulThresh = _tstof(temp);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zz_1.5.pcd", *cloud);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(MeanK);
	sor.setStddevMulThresh(StddevMulThresh);
	sor.filter(*cloud_filtered);


    //将点云添加到窗口
	m_viewer->addPointCloud(cloud);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_filtered, 255, 0, 0);
    m_viewer->addPointCloud<pcl::PointXYZ >(cloud_filtered, single_color2,"1");
	m_viewer->resetCamera();
	
	//m_viewer->setCameraPosition(0,0,3,0,0,-1,0,1,0);

	//导出过滤后的点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zzglh2.pcd", *cloud_filtered, false);

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zzzd2.pcd", *cloud_filtered, false);


	//获取圆柱点云的范围，即高程起始值
	pcl::PointCloud<pcl::PointXYZ> cloudYZ;
	pcl::PCDReader readerYZ;
	readerYZ.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zzglh2.pcd", cloudYZ);
	Eigen::Vector4f min_p, max_p;
	pcl::getMinMax3D(cloudYZ, min_p, max_p); 	//圆柱范围0.173700005,17.0091991,1.27880001     0.830799997,17.6690998,8.04030037

}


void CAboutDlg::OnProject() //圆柱提取--向XOY平面投影
{
	// TODO: 在此添加命令处理程序代码

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zzglh2.pcd", *cloud);

	
	// Create a set of planar coefficients with X=Y=0,Z=1；即XOY平面
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	//将点云添加到窗口
	m_viewer->addPointCloud(cloud);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_projected, 255, 0, 0);
	m_viewer->addPointCloud<pcl::PointXYZ >(cloud_projected, single_color2, "1");
	m_viewer->resetCamera();

	//导出投影后的点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zztyh3.pcd", *cloud_projected, false);

}


void CAboutDlg::OnRadiusOutlierRemovalFilter() //圆柱提取--半径滤波
{
	// TODO: 在此添加命令处理程序代码
	//获取参数设置窗口中设置的半径过滤参数
	CString temp;
	temp = dlg_csss.SearchRadius;
	double SearchRadius= _tstof(temp);//搜索半径
	temp = dlg_csss.MinNeighborsInRadius;
	double MinNeighborsInRadius= _tstof(temp); //半径内最小点云数量

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zztyh3.pcd", *cloud);


	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(SearchRadius);
	outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
	outrem.setKeepOrganized(true);
	// apply filter
	outrem.filter(*cloud_filtered);

	//将点云添加到窗口
	m_viewer->addPointCloud(cloud);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_filtered, 255, 0, 0);
	m_viewer->addPointCloud<pcl::PointXYZ >(cloud_filtered, single_color2, "1");
	m_viewer->resetCamera();

}

//点云最小二乘法拟合圆
void CAboutDlg::OncircleLeastFit()
{
	// TODO: 在此添加命令处理程序代码

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zztyh3.pcd", *cloud);

	cloud->size();

	double center_x = 0.0f;
	double center_y = 0.0f;
	double radius = 0.0f;
	if (cloud->size() < 3)
	{
		return ;
	}

	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

	int N = cloud->size();
	
	for (int i = 0; i < N; i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double x2 = x * x;
		double y2 = y * y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2 * x;
		sum_y3 += y2 * y;
		sum_xy += x * y;
		sum_x1y2 += x * y2;
		sum_x2y1 += x2 * y;
	}

	double C, D, E, G, H;
	double a, b, c;

	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

	center_x = a / (-2);
	center_y = b / (-2);
	radius = sqrt(a * a + b * b - 4 * c) / 2;


	//绘制圆

	pcl::ModelCoefficients circle_coeff;
	//circle_coeff.values.resize(3);    // We need 3 values
	//circle_coeff.values[0] = center_x;
	//circle_coeff.values[1] = center_y ;
	//circle_coeff.values[2] = radius;

	//m_viewer->addCircle(circle_coeff, "c");
	//m_viewer->addPointCloud(cloud);
	pcl::ModelCoefficients coeffs;
	//
	coeffs.values.push_back(center_x);
	coeffs.values.push_back(center_y);
	coeffs.values.push_back(5);
	m_viewer->addCircle(coeffs, "c");

	m_viewer->resetCamera();

}


void CAboutDlg::OnAccuracyAnalysis()
{
	// TODO: 在此添加命令处理程序代码

	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("F:\\PCL\\DATA\\column\\zztyh3.pcd", cloud);

	float radius = 0.3216;//上切面0.3237; //最小二乘拟合0.32306375016104849;

	pcl::PointXYZ center;
	center.x = 0.4999; //上切面0.5033; //最小二乘拟合0.50390042884982977;
	center.y = 17.3422; //下切面17.3404; //最小二乘拟合17.340817588905441;
	center.z = 0;

	std::vector<float> error;

	float temp,temp2;

	temp2 = 0;

	std::ofstream oFile;
	oFile.open("F:\\PCL\\DATA\\column\\下切面圆.csv", ios::out | ios::trunc);

	for (size_t i = 0; i < cloud.points.size(); i++)
	{
		temp = sqrt( (center.x - cloud.points[i].x)*(center.x - cloud.points[i].x)+(center.y-cloud.points[i].y)* (center.y - cloud.points[i].y) )-radius;

		error.push_back(temp);

		temp2 +=abs(temp);

		oFile << temp << endl;
	}

	


	float pjwc, bzc, temp3;

	temp3 = 0;

	pjwc = temp2 / cloud.points.size();

	oFile << "平均误差" << endl;

	oFile << pjwc << endl;

	for (size_t i = 0; i < cloud.points.size(); i++)
	{
		temp3 += (error[i] - pjwc) * (error[i] - pjwc);
	}

	bzc = sqrt(temp3 / error.size());

	oFile << "标准差" << endl;

	oFile << bzc << endl;

	oFile.close();

}


void CMFCApplication1Dlg::OnBnClickedNormal()
{
	// TODO: 在此添加控件通知处理程序代码
	/****************计算点云法向量********************/
	// 原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("F:/PCL/DATA/column/SdjyColumnL2.pcd", *cloud);							// 加载原始点云数据
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	// 计算点云法向量
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;										// OMP加速
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);																	// 设置openMP的线程数
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);																	// 设置KD树用于邻近搜索
	n.setKSearch(30);																			// k邻域搜索的点云点个数
	n.compute(*normals);

	
	/****************展示********************/
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(""));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 225, 0), "sample cloud");									// 添加点云
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.01, "normals");																					// 添加点云法向量

	//viewer->resetCamera();
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,255,255);
	m_viewer->addPointCloud<pcl::PointXYZ >(cloud, single_color, "fn");
	m_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.5, "normals");
	m_viewer->resetCamera();//使点云显示在屏幕中间，并绕中心操作

}


struct Point {
	double x, y, z;
};

struct Vector {
	double x, y, z;
};
//线段求交点  Point findIntersection(Point P1, Vector d1, Point P2, Vector d2) 
Point findIntersection(Point P1, Vector d1, Point P2, Vector d2){

	double A[2][2] = { {d1.x, -d2.x}, {d1.y, -d2.y} };
	double B[2] = { P2.x - P1.x, P2.y - P1.y };
	double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];

	if (det == 0) {
		std::cout << "直线平行或重合，没有唯一交点" << std::endl;
		return Point{ 0, 0, 0 }; // 返回一个无效点
	}

	double t = (B[0] * A[1][1] - B[1] * A[0][1]) / det;
	double s = (B[1] * A[0][0] - B[0] * A[1][0]) / det;

	Point intersection = { P1.x + t * d1.x, P1.y + t * d1.y, P1.z + t * d1.z };
	return intersection;
}

//单独提取线段
void CMFCApplication1Dlg::OnBnClickedButton5()
{
	// TODO: 在此添加控件通知处理程序代码
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\PCL\\DATA\\CharacteristicLines\\yuanSu2.pcd", *cloud);


	//将边界点云分割为若干个直线点云
	double dis_max = para.threshold;//点到模型的距离
	int pointC = cloud->size() / para.point_size; //最少为点云数量的1/10
	std::list<Line_pointCloud_coefficients> lpc_list;
	lpc_list = segment_to_lineCloud(cloud, dis_max, pointC);


	//通过直线点云 获取 拟合线段 构成线段数组
	std::vector<line> lines = cloudline_to_lines(lpc_list);

	Line_pointCloud_coefficients Lc1, Lc2, Lc3, Lc4;

	std::list<Line_pointCloud_coefficients>::iterator lpc = lpc_list.begin();	
	
	Lc1.coefficients = lpc->coefficients;

	//pcl::io::savePCDFileASCII("F:\\PCL\\DATA\\CharacteristicLines\\yuanSu2\\1.pcd", *lpc->cloud_line);

	lpc++;
	Lc2.coefficients = lpc->coefficients;
	//pcl::io::savePCDFileASCII("F:\\PCL\\DATA\\CharacteristicLines\\yuanSu2\\2.pcd", *lpc->cloud_line);

	lpc++;
	Lc3.coefficients = lpc->coefficients;
	//pcl::io::savePCDFileASCII("F:\\PCL\\DATA\\CharacteristicLines\\yuanSu2\\3.pcd", *lpc->cloud_line);

	lpc++;
	Lc4.coefficients = lpc->coefficients;
	//pcl::io::savePCDFileASCII("F:\\PCL\\DATA\\CharacteristicLines\\yuanSu2\\4.pcd", *lpc->cloud_line);


	Point P1; Vector d1; Point P2; Vector d2;
	P1.x=Lc1.coefficients->values[0]; P1.y = Lc1.coefficients->values[1]; P1.z = Lc1.coefficients->values[2];
	d1.x = Lc1.coefficients->values[3]; d1.y = Lc1.coefficients->values[4]; d1.z = Lc1.coefficients->values[5];

	P2.x = Lc3.coefficients->values[0]; P2.y = Lc3.coefficients->values[1]; P2.z = Lc3.coefficients->values[2];
	d2.x = Lc3.coefficients->values[3]; d2.y = Lc3.coefficients->values[4]; d2.z = Lc3.coefficients->values[5];


	Point p11 = findIntersection(P1,d1,P2,d2);

	P1.x = Lc1.coefficients->values[0]; P1.y = Lc1.coefficients->values[1]; P1.z = Lc1.coefficients->values[2];
	d1.x = Lc1.coefficients->values[3]; d1.y = Lc1.coefficients->values[4]; d1.z = Lc1.coefficients->values[5];

	P2.x = Lc4.coefficients->values[0]; P2.y = Lc4.coefficients->values[1]; P2.z = Lc4.coefficients->values[2];
	d2.x = Lc4.coefficients->values[3]; d2.y = Lc4.coefficients->values[4]; d2.z = Lc4.coefficients->values[5];


	Point p12 = findIntersection(P1, d1, P2, d2);
	

	P1.x = Lc2.coefficients->values[0]; P1.y = Lc2.coefficients->values[1]; P1.z = Lc2.coefficients->values[2];
	d1.x = Lc2.coefficients->values[3]; d1.y = Lc2.coefficients->values[4]; d1.z = Lc2.coefficients->values[5];

	P2.x = Lc4.coefficients->values[0]; P2.y = Lc4.coefficients->values[1]; P2.z = Lc4.coefficients->values[2];
	d2.x = Lc4.coefficients->values[3]; d2.y = Lc4.coefficients->values[4]; d2.z = Lc4.coefficients->values[5];


	Point p13 = findIntersection(P1, d1, P2, d2);

	P1.x = Lc2.coefficients->values[0]; P1.y = Lc2.coefficients->values[1]; P1.z = Lc2.coefficients->values[2];
	d1.x = Lc2.coefficients->values[3]; d1.y = Lc2.coefficients->values[4]; d1.z = Lc2.coefficients->values[5];

	P2.x = Lc3.coefficients->values[0]; P2.y = Lc3.coefficients->values[1]; P2.z = Lc3.coefficients->values[2];
	d2.x = Lc3.coefficients->values[3]; d2.y = Lc3.coefficients->values[4]; d2.z = Lc3.coefficients->values[5];


	Point p14 = findIntersection(P1, d1, P2, d2);


}








//三维直线求交点

void CMFCApplication1Dlg::OnBnClickedButton8()
{
	// TODO: 在此添加控件通知处理程序代码
}
