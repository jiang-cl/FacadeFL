#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
//vtk
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>


#include <vtkAutoInit.h>

#include <pcl/sample_consensus/ransac.h> //ransac法提取平面
#include <pcl/sample_consensus/sac_model_plane.h> //ransac法提取平面
#include <pcl/filters/extract_indices.h> //索引点
#include<pcl/filters/statistical_outlier_removal.h> //统计法（疑似高斯法）过滤点云

#include <pcl/surface/concave_hull.h>  //Alpha Shapes 提取投影边界
#include <pcl/filters/voxel_grid.h>  //Alpha Shapes 提取投影边界
#include <pcl/console/time.h>
#include <pcl/segmentation/extract_clusters.h> //用于欧氏空间聚类提取
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/impl/intersections.hpp>  //用于lineWithLineIntersection

#include<map>



VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2

VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);





using namespace pcl;
// MFCApplication1Dlg.h: 头文件
//

#pragma once


// CMFCApplication1Dlg 对话框
class CMFCApplication1Dlg : public CDialogEx
{
// 构造
public:
	CMFCApplication1Dlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MFCAPPLICATION1_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

//定义点云窗口
private:
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;//要共享指针类型的，要不然，显示窗口会跳出MFC界面
	vtkRenderWindow* m_win;   //vtk渲染的窗口句柄
	vtkRenderWindowInteractor* m_iren;//vtk交互的对象



private:
	
	double dis_plane;//设置距离阈值，与平面距离小于0.01的点作为内点,用于提取墙立面

	//定义参数结构，用于存储程序中所有的计算参数
	struct parameters
	{
		double MeanK;  //用于统计值过滤，稀疏异常值去除
		double StddevMulThresh;//用于统计值过滤，稀疏异常值去除

		//半径异常值过滤 参数
		double SearchRadius;//搜索半径
		double MinNeighborsInRadius; //半径内最小点云数量

		double Alpha; //用于边界计算

		//由边界点云拟合线段的参数
		double threshold;//点到直线模型的距离阈值，默认值0.03
		int max_iterations;//最大迭代此时，默认值1000
		int point_size;//设置拟合直线点云的最小数量，取点云的比例，默认值为1/20

		//线段焊接距离
		float dis_weld;

		//定义区域增长参数
		int RegionGrowing_K;
		int RegionGrowing_Neighbours;
		double RegionGrowing_SmoothnessThreshold;
		double RegionGrowing_CurvatureThreshold;

	};

	


	

public:
	parameters para;

	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnBnClickedButton1();

	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedDnoise();

	//CMenu m_Menu;
	afx_msg void Onmenusssz();
	//CTreeCtrl m_Tree;

	afx_msg void OnNMClickTree1(NMHDR* pNMHDR, LRESULT* pResult);

	struct cloud_plane_coefficient
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr remain_cloud;

		pcl::ModelCoefficients::Ptr plane_coefficient;
	};

	//定义方法，使用RANSAC随机采样一致性算法提取平面;参数1：点云，参数2：距离，设置距离阈值，与平面距离小于0.01的点作为内点
	cloud_plane_coefficient cloud_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,double dis)
	{
	
		cloud_plane_coefficient cpc;

		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));	//选择拟合点云与几何模型
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	//创建随机采样一致性对象
		ransac.setDistanceThreshold(dis);	//设置距离阈值，与平面距离小于0.01的点作为内点
		ransac.computeModel();				//执行模型估计

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>); //定义拟合的平面点云

		// 根据索引提取内点 

		std::vector<int> inliers;				//存储内点索引的向量
		ransac.getInliers(inliers);			//提取内点对应的索引
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane); //根据内点索引复制平面点云

		/// 输出模型参数Ax+By+Cz+D=0
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);

		cpc.cloud_plane = cloud_plane;
		//cpc.plane_coefficient = coefficient;

		pcl::ModelCoefficients::Ptr coefficient_prj(new pcl::ModelCoefficients());
		coefficient_prj->values.resize(4);
		coefficient_prj->values[0] =coefficient[0]; //coefficient[0];
		coefficient_prj->values[1] =coefficient[1]; //coefficient[1];
		coefficient_prj->values[2] =coefficient[2];// coefficient[2];
		coefficient_prj->values[3] =coefficient[3];// coefficient[3];
		cpc.plane_coefficient = coefficient_prj;

		
		
		// extract remain pointcloud 提取分离后剩余的点云
		pcl::IndicesPtr index_ptr(new std::vector<int>(inliers));/// 将自定义的pi数组进行智能指针的转换
				
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud(cloud);
	    extract.setIndices(index_ptr);
	   
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
	    extract.setNegative(true);
	    extract.filter(*cloud_remain);

	   // extract segmentation part  提取分离的点云
    
       //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line1(new pcl::PointCloud<pcl::PointXYZ>); 
       //extract.setNegative(false);
       //extract.filter(*cloud_line1);

	   cpc.remain_cloud = cloud_remain;
	   
		return cpc;
	}

	// 定义线结构，有两点构成
	struct  line
	{
		pcl::PointXYZ pt1;
		pcl::PointXYZ pt2;
	};
	//定义一个结构，包含点云的名称和智能指针
	struct ptrName
	{
		std::string cloud_name;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
		pcl::ModelCoefficients::Ptr plane_coefficient;
		std::vector<line> lines;
		std::vector<pcl::PointXYZ> polyline;
	};
	//定义一个map（图）结构，用于存储中的处理数据
	typedef std::map<int, ptrName> mp;

	//定义图类型
	 mp mapCloud;
	//定义方法，由点云名称查询点云key
	 int name_to_key(CString text)
	{
		std::string text_str = CT2A(text.GetString());
		std::map<int, ptrName>::iterator it;
		it = mapCloud.begin();
		while (it != mapCloud.end())
		{
			if (text_str == it._Ptr->_Myval.second.cloud_name)
			{
				//m_viewer->addPointCloud(it._Ptr->_Myval.second.cloud_ptr, it._Ptr->_Myval.second.cloud_name);

				return it._Ptr->_Myval.first;
			}

			it++;
		}

		return -1;
	}



	//过滤点云
	//稀疏异常值去除是基于输入数据集中点到邻居距离分布的计算。
	//对于每个点，我们计算从它到它所有邻居的平均距离。
	//通过假设结果分布是具有平均值和标准差的高斯分布，
	//其平均距离在由全局距离平均值和基准差定义的区间之外的所有点都可以被视为异常值，并从数据集中进行修剪。
	pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(para.MeanK);    //设置用于平均距离估计的最近邻居的数量
		sor.setStddevMulThresh(para.StddevMulThresh);   //设置距离阈值计算的标准偏差乘数。距离阈值将等于：mean+stddev_mult*stddev。
		                               //  如果点的平均邻居距离分别低于或高于该阈值，则将其分类为内点或外点。
		sor.filter(*cloud_filtered);

		//提取噪点
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ZaoDian(new pcl::PointCloud<pcl::PointXYZ>);
		sor.setNegative(true);
		sor.filter(*cloud_ZaoDian);

		////提取噪点
		//pcl::IndicesConstPtr ZaodianIndices = sor.getRemovedIndices();
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ZaoDian(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::ExtractIndices<pcl::PointXYZ> extract;
		//extract.setInputCloud(cloud);
		//extract.setIndices(ZaodianIndices);
		//extract.setNegative(false); // 如果想提取不在索引列表中的点，可以设置为true
		//extract.filter(*cloud_ZaoDian);

		pcl::io::savePCDFileASCII("F:\\PCL\\DATA\\CharacteristicLines\\ZaoDian.pcd", *cloud_ZaoDian);

		return cloud_filtered;
	}

	//提取点云边界，（提取投影后的平面点云的边界）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_projected)
	{

		//-------------------------- 投影边界下采样 --------------------------
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);	//投影下采样点云
		//pcl::VoxelGrid<pcl::PointXYZ> vg;			//创建体素下采样对象
		//vg.setInputCloud(cloud_plane_projected);			//设置下采样输入点云
		//vg.setLeafSize(0.05f, 0.05f, 0.05f);//设置体素栅格边长
		//vg.filter(*cloud_sub);				//执行体素下采样
		//==================================================================

		//-------------------- Alpha Shapes 提取投影边界 --------------------
		pcl::console::TicToc time;
		time.tic();
		cout << "->正在提取边界..." << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConcaveHull < pcl::PointXYZ > ch;
		ch.setInputCloud(cloud_plane_projected);
		ch.setAlpha(0.1); //原文0.15 //设置alpha值，该值限制生成的外壳分段的大小（外壳越小，越详细）
		ch.reconstruct(*cloud_boundary);
		cout << "->边界点提取用时：" << time.toc() / 1000 << " s" << endl;
		//==================================================================

		return cloud_boundary;
	}

	//欧氏距离聚类提取，用于提取不同的多边形的边界----测试成功
	std::vector < pcl::PointCloud <pcl::PointXYZ>::Ptr> euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered)
	{
		//利用输入点云创建kd树对象tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_filtered);

		//创建点云索引向量，用于存储实际的点云信息
		std::vector<pcl::PointIndices> cluster_indices;
		//创建欧式聚类分割对象
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.3); //设置近邻搜索的搜索半径
		ec.setMinClusterSize(20); //设置最小聚类尺寸
		ec.setMaxClusterSize(10000);//设置最大聚类尺寸
		ec.setSearchMethod(tree);//设置点云的搜索机制
		ec.setInputCloud(cloud_filtered);
		ec.extract(cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

		//迭代访问点云索引cluster_indices,直至分割出所有聚类
		std::vector < pcl::PointCloud <pcl::PointXYZ>::Ptr> euclu_extra; //创建点云数据集，用于存放分割后的所有点云

		int i = 0;

		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
		while (it != cluster_indices.end())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);//定义点云，用于存储分割后的单个点云

			std::vector<int>::const_iterator pit = it->indices.begin();

			while (pit != it->indices.end())
			{
				cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

				pit++;
			}

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			euclu_extra.push_back(cloud_cluster);

			it++;
		}

		return euclu_extra;
	}

	//定义结构，包含拟合直线的点云和拟合直线常数两个变量. -----用于存储拟合直线的分离点云和直线常数
	struct Line_pointCloud_coefficients
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line;
		pcl::ModelCoefficients::Ptr coefficients;
				
	};
	//分割点云，将点云分割为若干个直线点云.参数1：要分割的点云；参数2：点到模型的距离；参数3：拟合点云的最少数量（即：小于某数时不再拟合）
	std::list<Line_pointCloud_coefficients> segment_to_lineCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold, int point_size)
	{
		std::list<Line_pointCloud_coefficients> list_lpc;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);

		cloud_remain = cloud;
		while (cloud_remain->points.size() > point_size)
		{
			Line_pointCloud_coefficients lpc;

			//定义拟合分割
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //定义直线参数
			pcl::PointIndices::Ptr indices(new pcl::PointIndices); //定义点索引
			pcl::SACSegmentation<pcl::PointXYZ> seg_SAC;  //定义拟合分割
			seg_SAC.setOptimizeCoefficients(true);
			seg_SAC.setModelType(pcl::SACMODEL_LINE);
			seg_SAC.setMethodType(pcl::SAC_RANSAC);
			seg_SAC.setMaxIterations(para.max_iterations); //设置最大迭代次数，默认值1000
			seg_SAC.setDistanceThreshold(distance_threshold); //设置到模型的距离阈值，默认值0.03

			
			//定义提取分割
			pcl::ExtractIndices<pcl::PointXYZ> extract; //定义提取索引
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud < pcl::PointXYZ>); //定义提取的线点云

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain_0(new pcl::PointCloud<pcl::PointXYZ>); //定义点云，用于存储分割后剩余的点云

			//点云分割
			seg_SAC.setInputCloud(cloud_remain);
			seg_SAC.segment(*indices, *coefficients);

			//点云提取
			extract.setInputCloud(cloud_remain);
			extract.setIndices(indices);
			extract.setNegative(false);
			extract.filter(*cloud_line);

			extract.setNegative(true);
			extract.filter(*cloud_remain_0);

			if (cloud_line->size() > cloud_line->size() / 5) //大于9个点，存储提取点云
			{
				lpc.cloud_line = cloud_line;
				lpc.coefficients = coefficients;
				list_lpc.push_back(lpc);

			}


			cloud_remain = cloud_remain_0;

		}

		return list_lpc;
	}
	//求线点云中距离最远的两个点（即线点云的两个端点）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line_to_2point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line) //, pcl::ModelCoefficients::Ptr coefficients1
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		//------------------------计算拟合直线点云中距离最远的两个点-----------------------------------------

		pcl::PointXYZ pt1, pt2, pt, pt0;
		double dis = 0;

		pt = cloud_line->points[0];

		for (int i = 0; i < cloud_line->size(); i++)
		{
			pt0 = cloud_line->points[i];


			double dis0 = sqrt(pow((pt.x - pt0.x), 2) + pow((pt.y - pt0.y), 2) + pow((pt.z - pt0.z), 2));

			if (dis0 > dis)
			{
				pt1 = pt;
				dis = dis0;
			}
		}
		dis = 0;
		for (int i = 0; i < cloud_line->size(); i++)
		{
			pt0 = cloud_line->points[i];

			double dis0 = sqrt(pow((pt1.x - pt0.x), 2) + pow((pt1.y - pt0.y), 2) + pow((pt1.z - pt0.z), 2));

			if (dis0 > dis)
			{
				pt2 = pt0;
				dis = dis0;
			}

		}

	/*	cloud->push_back(pt1);
		cloud->push_back(pt2);*/

		pcl::PointXYZ pt3, pt4;
		pcl::getMinMax3D(*cloud_line, pt3, pt4);

		cloud->push_back(pt3);
		cloud->push_back(pt4);

		return cloud;
		//=================================================================================================
	}

	//求点在直线上的投影点（垂足）
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_projection_to_line(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::ModelCoefficients::Ptr coefficients)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proj(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ pt, pt_proj;

		double a = coefficients->values[0];
		double b = coefficients->values[1];
		double c = coefficients->values[2];
		double d = coefficients->values[3];
		double e = coefficients->values[4];
		double f = coefficients->values[5];

		double t;

		for (int i = 0; i < cloud->points.size(); i++)
		{
			pt.x = cloud->points[i].x;
			pt.y = cloud->points[i].y;
			pt.z = cloud->points[i].z;

			t = (d * (pt.x - a) + e * (pt.y - b) + f * (pt.z - c)) / (d * d + e * e + f * f);

			pt_proj.x = d * t + a;
			pt_proj.y = e * t + b;
			pt_proj.z = f * t + c;

			cloud_proj->push_back(pt_proj);
		}

		return cloud_proj;
	}


	//由存储拟合直线点云和直线常数的 链 计算 拟合的线段数组
	std::vector<line> cloudline_to_lines(std::list<Line_pointCloud_coefficients> list_lpc)
	{
		std::vector<line> lines;

		//定义第一条拟合直线的 点云和直线常数
		std::list<Line_pointCloud_coefficients>::iterator lpc = list_lpc.begin();

		//遍历分割的直线点云，求取所有的线段

		int i = 1;
		while (lpc != list_lpc.end())
		{
			//定义并获取拟合直线点云的 两端点的 点
			pcl::PointCloud<pcl::PointXYZ>::Ptr line_point(new 	pcl::PointCloud<pcl::PointXYZ>);
			line_point = cloud_line_to_2point(lpc->cloud_line);

			//定义并获取拟合直线点云的两端点 在拟合直线上的投影，即拟合线段的两端点
			pcl::PointCloud<pcl::PointXYZ>::Ptr line_point_proj(new pcl::PointCloud<pcl::PointXYZ>);

			line_point_proj = point_projection_to_line(line_point, lpc->coefficients);

			line line;

			line.pt1 = line_point_proj->points[0];
			line.pt2 = line_point_proj->points[1];

			lines.push_back(line);

			lpc++;
			i++;

		}

		return lines;
	}


	//求取两点间的距离
	double dis_2point(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
	{
		double dis2, dis;

		dis2 = pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2) + pow((pt1.z - pt2.z), 2);

		dis = pow(dis2, 0.5);

		return dis;
	}

	//求两条直线的交点
	pcl::PointXYZ jiaodian_2_line(line l1, line l2)
	{
		Eigen::VectorXf line_a(6);
		line_a(0) = l1.pt1.x;
		line_a(1) = l1.pt1.y;
		line_a(2) = l1.pt1.z;
		line_a(3) = l1.pt2.x - l1.pt1.x;
		line_a(4) = l1.pt2.y - l1.pt1.y;
		line_a(5) = l1.pt2.z - l1.pt1.z;

		Eigen::VectorXf line_b(6);
		line_b(0) = l2.pt1.x;
		line_b(1) = l2.pt1.y;;
		line_b(2) = l2.pt1.z;;
		line_b(3) = l2.pt2.x - l2.pt1.x;;
		line_b(4) = l2.pt2.y - l2.pt1.y;
		line_b(5) = l2.pt2.z - l2.pt1.z;

		Eigen::Vector4f point;

		bool b = lineWithLineIntersection(line_a, line_b, point, 1e-4);

		pcl::PointXYZ pt;

		pt.x = point(0);
		pt.y = point(1);
		pt.z = point(2);


		//char f[10];

		//int temp = round((point(0) * 1000));
		////char temp_char[10];
		////_itoa(temp, temp_char, 10);

		//std::string s = std::to_string(temp);

		//std::string ss = s.substr(0, s.size() - 3) + "." + s.substr(s.size() - 3, 3);

		//char c[10];

		//for (size_t i = 0; i < ss.size(); i++)
		//{
		//	c[i] = ss[i];
		//}
		//for (size_t i = ss.size(); i < 10; i++)
		//{
		//	c[i] = 48;

		//}

		//float ff = std::stof(c);

		//pt.y = ((float)((int)((point(1) + 0.005) * 100))) / 100;
		//pt.z = ((float)((int)((point(0) + 0.005) * 100))) / 100;		

		return pt;
	}

	//焊接线段，将线段数组的端点进行焊接，
	std::vector<line> lines_to_weld(std::vector<line> lines)
	{

		double dis_weld =para.dis_weld; //焊接距离，即：当距离小于该值时焊接两点

		std::vector<line> lines_weld;

		for (size_t i = 0; i < lines.size(); i++)
		{
			for (size_t j = 0; j < lines.size(); j++)
			{
				if (dis_2point(lines[i].pt1, lines[j].pt1) < dis_weld)
				{
					if (i != j)
					{
						pcl::PointXYZ pt = jiaodian_2_line(lines[i], lines[j]);
						lines[i].pt1 = pt;
						
					}
				}
				if (dis_2point(lines[i].pt1, lines[j].pt2) < dis_weld)
				{
					pcl::PointXYZ pt = jiaodian_2_line(lines[i], lines[j]);
					lines[i].pt1 = pt;
					//lines[j].pt2 = pt;
				}

			}

		}

		for (size_t i = 0; i < lines.size(); i++)
		{
			for (size_t j = 0; j < lines.size(); j++)
			{
				if (dis_2point(lines[i].pt2, lines[j].pt1) < dis_weld)
				{


					pcl::PointXYZ pt = jiaodian_2_line(lines[i], lines[j]);
					lines[i].pt2 = pt;
					//lines[j].pt1 = pt;

				}
				if (dis_2point(lines[i].pt2, lines[j].pt2) < dis_weld)
				{
					if (i != j)
					{
						pcl::PointXYZ pt = jiaodian_2_line(lines[i], lines[j]);
						lines[i].pt2 = pt;
						//lines[j].pt2 = pt;
					}

				}

			}

		}

		return lines;
	}

	//判断是否闭合
	struct lines_close
	{
		bool close;
		int line;
		int pt;
	};
	//判断是否闭合
	lines_close close_bool(std::vector<line> line_weld)
	{
		lines_close close;
		close.close = true;

		bool lianxu;
		lianxu = false;

		for (size_t i = 0; i <line_weld.size(); i++)
		{
			lianxu = false;
			for (size_t j = 0; j <line_weld.size(); j++)
			{
				if (j!=i)
				{
					if (dis_2point(line_weld[i].pt1, line_weld[j].pt1) < 0.3)
					{
						lianxu = true;
						break;
					}

					if (dis_2point(line_weld[i].pt1, line_weld[j].pt2) < 0.3)
					{
						lianxu = true;
						break;
					}

				}
			}

			if (!lianxu)
			{
				close.close = false;
				close.line = i;
				close.pt = 1;
				break;
				return close;
			}			
		}

		for (size_t i = 0; i < line_weld.size(); i++)
		{
			lianxu = false;
			for (size_t j = 0; j < line_weld.size(); j++)
			{
				if (j != i)
				{
					if (dis_2point(line_weld[i].pt2, line_weld[j].pt1) < 0.3)
					{
						lianxu = true;
						break;
					}

					if (dis_2point(line_weld[i].pt2, line_weld[j].pt2) < 0.3)
					{
						lianxu = true;
						break;
					}
				}
			}

			if (!lianxu)
			{
				close.close = false;
				close.line = i;
				close.pt = 2;
				break;
				return close;
			}
		}

		return close;
	}

	//将焊接后的线段转换多段线--闭合式
	std::vector<pcl::PointXYZ> line_to_ployline(std::vector<line> line_weld)
	{

		double dis_weld = 0.3; //焊接距离，即：当距离小于该值时焊接两点

		std::vector<pcl::PointXYZ> polyLine;

		polyLine.push_back(line_weld[0].pt1);

		int index = 0; //线的索引号

		int size = 1;//已存储的线段


		while (polyLine.size() < line_weld.size() + 1)
		{
			for (size_t i = 0; i < line_weld.size(); i++)
			{
				if (i != index)
				{
					if (dis_2point(polyLine[polyLine.size() - 1], line_weld[i].pt1) < dis_weld)
					{
						polyLine.push_back(line_weld[i].pt2);

						index = i;

						break;
					}
					if (dis_2point(polyLine[polyLine.size() - 1], line_weld[i].pt2) < dis_weld)
					{
						polyLine.push_back(line_weld[i].pt1);

						index = i;

						break;
					}

				}

			}

		}

		return polyLine;
	}


	//将焊接后的线段转换多段线--非闭合式
	std::vector<pcl::PointXYZ> line_to_ployline_duankai(std::vector<line> line_weld)
	{

		double dis_weld = 0.3; //焊接距离，即：当距离小于该值时焊接两点

		std::vector<pcl::PointXYZ> polyLine;

		polyLine.push_back(line_weld[0].pt1);

		polyLine.push_back(line_weld[0].pt2);

		int index = 0; //线的索引号

	
		bool lianxu = true;

		while (lianxu)
		{
			lianxu = false;

			for (size_t i = 0; i < line_weld.size(); i++)
			{
				if (i != index)
				{
					if (dis_2point(polyLine[polyLine.size() - 1], line_weld[i].pt1) < dis_weld)
					{
						polyLine.push_back(line_weld[i].pt2);

						index = i;

						lianxu = true;

						break;
					}
					if (dis_2point(polyLine[polyLine.size() - 1], line_weld[i].pt2) < dis_weld)
					{
						polyLine.push_back(line_weld[i].pt1);

						index = i;

						lianxu = true;

						break;
					}

				}

			}

		}

		return polyLine;
	}


	//由提取线段 转 多段线

	std::vector<pcl::PointXYZ> lines_to_ployline(std::vector<line> lines)
	{
		std::vector<line> lines_weld = lines_to_weld(lines);
		
		//判断是否闭合

		lines_close close = close_bool(lines_weld);
		
		std::vector<pcl::PointXYZ> polyline;

		if (!close.close) //不闭合
		{
			//先调整线段的顺序

			if (close.pt!=1)
			{
				pcl::PointXYZ pt = lines_weld[close.line].pt1;

				lines_weld[close.line].pt1 = lines_weld[close.line].pt2;

				lines_weld[close.line].pt2 = pt;
			}

			if (close.line!=0)
			{
				line l = lines_weld[close.line];

				lines_weld[close.line] = lines_weld[0];

				lines_weld[0] = l;
			}
			
			polyline = line_to_ployline_duankai(lines_weld);

		}
		else //闭合
		{
			polyline = line_to_ployline(lines_weld);

		}
				
		return polyline;

	}
	//float取小数点后三位数字转换为string
	std::string float_3_string(float f)
	{

		float ff = f * 1000;
		int c = round(ff);

		std::string str = std::to_string(c);

		std::string s;
		char c0 = 0; char c1 =46;
		if (str[0]==45) //45为“-”负号 的ASCII码
		{
			if (str.size()==2)
			{
				//s[0] = str[0];
				//s[1] = (char)"0";//0的ASCII码
				//s[2] = (char)".";//.的ASCII码
				//s[3] = (char)"0";
				//s[4] = (char)"0";
				//s[5] = str[1];
				s += str[0];
				s +="0";
				s +=".";
				s += "0";
				s += "0";
				s = s + str[1];
			}
			else if (str.size() == 3)
			{
				s = str[0];
				s += "0";;
				s+= ".";
				s+= "0";
				s+= str[1];
				s+= str[2];
			}
			else if (str.size() == 4)
			{
				s= str[0];
				s+= "0";
				s+= ".";
				s += str[1];
				s += str[2];
				s += str[3];
			}
			else if (str.size() >4)
			{
				std::string str1 = str.substr(0, str.size() - 3);

				std::string str2 = str.substr(str.size() - 3, 3);

				s = str1 + "." + str2;
			}

		}
		else if (str[0]!= 45)//45为“-”负号
		{
			if (str.size() == 1)
			{
				s = "0";
				s += ".";
				s += "0";
				s += "0";
				s += str[0];
			}
			else if(str.size() == 2)
			{
				s = "0";
				s += ".";
				s += "0";
				s += str[0];
				s += str[1];
			}
			else if (str.size() == 3)
			{
				s = "0";
				s += ".";
				s += str[0];
				s += str[1];
				s += str[2];
			}
			else if (str.size() > 3)
			{
				std::string str1 = str.substr(0, str.size() - 3);

				std::string str2 = str.substr(str.size() - 3, 3);

				s = str1 + "." + str2;
			}			

		}
	
		return s;

		
	}




	afx_msg void OnNMRClickTree1(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnBnClickedsavepc();
	CProgressCtrl m_progress;
	afx_msg void OnBnClickedButton6();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedprojectionplane();
	afx_msg void OnBnClickedboundary();
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedcluster();
	afx_msg void OnBnClickedline();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedAddpoint();
	CListBox ListBox_Point;
	afx_msg void OnBnClickedsavecadcommand();
	
	
	
	afx_msg void OnRadiusOutlierRemoval();

	

	afx_msg void OnBnClickedNormal();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton8();
};
