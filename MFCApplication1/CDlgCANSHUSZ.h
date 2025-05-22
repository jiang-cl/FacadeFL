#pragma once


// CDlgCANSHUSZ 对话框

class CDlgCANSHUSZ : public CDialogEx
{
	DECLARE_DYNAMIC(CDlgCANSHUSZ)

public:
	CDlgCANSHUSZ(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~CDlgCANSHUSZ();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_CANSHUSZ };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()

	// 实现
protected:
	//HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	//afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	//afx_msg void OnPaint();
	//afx_msg HCURSOR OnQueryDragIcon();
	//DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
public:
	//拟合平面距离
	CString dis_plane;
	//统计异常值删除过滤参数
	CString MeanK;
	//统计异常值删除乘常数
	CString StddevMulThresh;
	//半径异常值删除过滤器 过滤半径参数
	CString SearchRadius;
	//半径异常值删除过滤器 半径内最少点数量
	CString MinNeighborsInRadius;

	//边界提取参数
	CString Alpha;

	//定义拟合直线段参数
	CString threshold;
	CString max_iterations;
	CString point_size;

	//定义线段焊接间距参数
	CString dis_weld;

	//区域增长参数

	CString RegionGrowing_K;
	CString RegionGrowing_Neighbours;
	CString RegionGrowing_SmoothnessThreshold;
	CString RegionGrowing_CurvatureThreshold;
};
