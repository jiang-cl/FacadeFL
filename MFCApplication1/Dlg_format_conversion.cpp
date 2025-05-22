// Dlg_format_conversion.cpp: 实现文件
//

#include "pch.h"
#include "MFCApplication1.h"
#include "Dlg_format_conversion.h"
#include "afxdialogex.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <isostream>
#include <boost/filesystem.hpp>

// Dlg_format_conversion 对话框

IMPLEMENT_DYNAMIC(Dlg_format_conversion, CDialogEx)

CString sourceFilePath;
CString resultFilePath;

Dlg_format_conversion::Dlg_format_conversion(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DIALOG1, pParent)
{

}

Dlg_format_conversion::~Dlg_format_conversion()
{
}

void Dlg_format_conversion::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, sourcePath);
	DDX_Control(pDX, IDC_EDIT2, resultpath);
}


BEGIN_MESSAGE_MAP(Dlg_format_conversion, CDialogEx)
	ON_BN_CLICKED(IDOK, &Dlg_format_conversion::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON1, &Dlg_format_conversion::OnBnClickedButton1)
	ON_BN_CLICKED(IDCANCEL, &Dlg_format_conversion::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON2, &Dlg_format_conversion::OnBnClickedButton2)
END_MESSAGE_MAP()


// Dlg_format_conversion 消息处理程序


//打开pcd文件路径
void Dlg_format_conversion::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码

	CString filter = L"文件 (*.pcd; *.las)|*.pcd;*.las||";

	//true为打开文件，false为保存文件
	CFileDialog openFileDlg(true, TEXT("*.pcd"),NULL, OFN_HIDEREADONLY | OFN_READONLY, filter,NULL);

	if (openFileDlg.DoModal() == IDOK)
	{
		sourceFilePath = openFileDlg.GetPathName(); // 获取文件路径

		sourcePath.SetWindowTextW(sourceFilePath);
	}
}

//保存xyz文件路径
void Dlg_format_conversion::OnBnClickedButton2()
{
	CString filter = L"文件 (*.txt; *.las)|*.txt;*.las||";

	//true为打开文件，false为保存文件
	CFileDialog openFileDlg(false, TEXT("*.pcd"), NULL, OFN_HIDEREADONLY | OFN_READONLY, filter, NULL);

	if (openFileDlg.DoModal() == IDOK)
	{
		resultFilePath = openFileDlg.GetPathName(); // 获取文件路径

		resultpath.SetWindowTextW(resultFilePath);
	}

}

//转换文件格式
void Dlg_format_conversion::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	//CDialogEx::OnOK();

	if (sourceFilePath!="" && resultFilePath !="")
	{
		//std::string id(CW2A(text.GetString()))
		std::string str(CW2A(sourceFilePath.GetString()));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(str,*cloud)!=-1)
		{
			//将点云导出到记事本
			std::ofstream ofs;

			ofs.open(resultFilePath, std::ios_base::out);
			
			for (size_t i = 0; i <cloud->size(); i++)
			{
				ofs << i<<",";

				ofs << cloud->points[i].x<<",";

				ofs << cloud->points[i].y<<",";

				ofs << cloud->points[i].z;

				ofs << std::endl;
				
			}

			ofs.close();
			MessageBox(_T("转换完成"));
		}
		
	}
}

void Dlg_format_conversion::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}
