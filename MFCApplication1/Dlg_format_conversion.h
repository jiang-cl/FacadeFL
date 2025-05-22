#pragma once


// Dlg_format_conversion 对话框

class Dlg_format_conversion : public CDialogEx
{
	DECLARE_DYNAMIC(Dlg_format_conversion)

public:
	Dlg_format_conversion(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~Dlg_format_conversion();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG1 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedCancel();
	CEdit sourcePath;
	CEdit resultpath;
};
