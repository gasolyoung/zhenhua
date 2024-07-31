/*
 * www.pclcn.org
 *
 *
 */

#include <fstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/time_cpu.h>
//#include <boost/shared_ptr.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include<vector>

#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>


#include <pcl/filters/passthrough.h>


using pcl::cuda::PointCloudAOS;
using pcl::cuda::Device;

class KinectViewerCuda
{
  public:
     KinectViewerCuda (bool downsample) : viewer ("KinectGrabber"), downsample_(downsample),global_(false) {}

    void cloud_cb_ (const boost::shared_ptr<openni_wrapper::Image>& image, 
	const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float constant)
    {
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
      // ����һ��ָ��PointCloudAOS<Device>���͵�ָ��
      PointCloudAOS<Device>::Ptr data;
    	{
        // ��ʱ�������ڼ���d2c.compute<Device>������ִ��ʱ��
        pcl::cuda::ScopeTimeCPU t ("time:");    
        // ����d2c.compute<Device>��������depth_image��image��constant��data��downsample_��Ϊ��������
        d2c.compute<Device> (depth_image, image, constant, data, downsample_);
        }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::cuda::toPCL (*data, *output);
	  cloud_=output;

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered0 (new pcl::PointCloud<pcl::PointXYZRGB>),
	  	cloud_filteredz (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_filteredx (new pcl::PointCloud<pcl::PointXYZRGB>);
//******************************************* �˳�����Զ��2���׵ĵ��ƣ����ں�������Ч�ʺ�����
	  // ����һ��PassThrough�˲���
pcl::PassThrough<pcl::PointXYZRGB> pass;
// �����������
pass.setInputCloud (output);
// �����˲��ֶ�Ϊz��
pass.setFilterFieldName ("z");
// �����˲���ΧΪ0.0��1.7
pass.setFilterLimits (0.0, 1.7);
// �Ե��ƽ����˲�
pass.filter (*cloud_filteredz);
// �����������Ϊ��һ���˲���ĵ���
pass.setInputCloud(cloud_filteredz);
// �����˲��ֶ�Ϊx��
 pass.setFilterFieldName ("x");
// �����˲���ΧΪ-0.3��0.3
 pass.setFilterLimits (-0.3, 0.3);
// �Ե��ƽ����˲�
 pass.filter (*cloud_filteredx);
// �����������Ϊ��һ���˲���ĵ���
 pass.setInputCloud(cloud_filteredx);
// �����˲��ֶ�Ϊy��
 pass.setFilterFieldName ("y");
// �����˲���ΧΪ-0.3��0.3
 pass.setFilterLimits (-0.3, 0.3);
// �Ե��ƽ����˲�
 pass.filter (*cloud_filtered0);
//*********************************************����ǰ�Ե��ƽ����²���
	  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	  vg.setInputCloud (cloud_filtered0);
	  vg.setLeafSize (0.001f, 0.001f, 0.001f);
	  vg.filter (*cloud_filtered);
	  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  //cloud_filtered�����²�������ĵ���
//******************************************����޳�������ƽ���ϵĵ���*/
	  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	  pcl::PointIndices::Ptr tmpinliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());

	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.05);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
	  int i=0, nr_points = (int) cloud_filtered->points.size ();
	  while (cloud_filtered->points.size () > 0.3 * nr_points)
	  {
		  // Segment the largest planar component from the remaining cloud
		  seg.setInputCloud (cloud_filtered);
		  seg.segment (*tmpinliers, *coefficients);
		  //std::cout << *coefficients << std::endl;��ӡƽ����ĸ�����
		  if (tmpinliers->indices.size () == 0)
		  {
			  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			  break;
		  }

		  // Extract the planar inliers from the input cloud
		  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		  extract.setInputCloud (cloud_filtered);
		  extract.setIndices (tmpinliers);
		  extract.setNegative (false);

		  // Write the planar inliers to disk
		  extract.filter (*cloud_plane);
		  std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		  // Remove the planar inliers, extract the rest
		  extract.setNegative (true);
		  extract.filter (*cloud_f);
		  cloud_filtered = cloud_f;
	  }

  output=cloud_filtered;
  //******************************************cloud_filteredΪȥ��������ƽ�漯�ϵĵ���
  //******************************************��ʼ������� 
  //�����
  pcl::SACSegmentation<pcl::PointXYZRGB> seg2;
  pcl::PointIndices::Ptr tmpinliers2 (new pcl::PointIndices);
  std::vector <pcl::ModelCoefficients> coefficients_all;
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere (new pcl::PointCloud<pcl::PointXYZRGB> ());

  seg2.setOptimizeCoefficients (true);
  seg2.setModelType (pcl::SACMODEL_SPHERE);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (100);
  seg2.setDistanceThreshold (0.01);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_f2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  //**************************************����ȫ������
    for(i=0;i<3;i++)
	{
	  if(global_==false&&i==0)
	  {
		  std::cout<<"\n���������������������Ķ�Ӧ��ȫ������λ��x y z֮���ÿո��س�\n"<<std::endl;
		  std::cin>>xyz.x;
		  std::cin>>xyz.y;
		  std::cin>>xyz.z;
		  std::cout<<"			x="<<xyz.x<<"			y="<<xyz.y<<"			z="<<xyz.z<<std::endl;
		  std::cin.clear();
	  }//�������
	  if(global_==false&&i==1)
	  {
		  std::cout<<"\n�����������Զ��������Ķ�Ӧ��ȫ������λ��x y z֮���ÿո��س�\n"<<std::endl;
		  std::cin>>xyz.x;
		  std::cin>>xyz.y;
		  std::cin>>xyz.z;
		  std::cout<<"			x="<<xyz.x<<"			y="<<xyz.y<<"			z="<<xyz.z<<std::endl;
		  std::cin.clear();
	  }//�ڶ�������
	  if(global_==false&&i==2)
	  {
		  std::cout<<"\n������������Զ��������Ķ�Ӧ��ȫ������λ��x y z֮���ÿո��س�\n"<<std::endl;
		  std::cin>>xyz.x;
		  std::cin>>xyz.y;
		  std::cin>>xyz.z;
		  std::cout<<"			x="<<xyz.x<<"			y="<<xyz.y<<"			z="<<xyz.z<<std::endl;
		  std::cin.clear();
		  global_=true;
	  }//��Զ����
	   xyz_all.push_back(xyz);
	}

  for(i=0;i<3;i++)//ѭ���������������
  {
	  // Segment the largest planar component from the remaining cloud
	  pcl::ModelCoefficients coefficients2;
	 
	  seg2.setInputCloud (cloud_filtered);
	  seg2.segment (*tmpinliers2, coefficients2);
	  coefficients_all.push_back(coefficients2);
	
	 
	  if (tmpinliers2->indices.size () == 0)
	  {
		  std::cout << "Could not estimate a sphere model for the given dataset." << std::endl;
		  break;
	  }

	  // Extract the planar inliers from the input cloud
	  pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
	  extract2.setInputCloud (cloud_filtered);
	  extract2.setIndices (tmpinliers2);
	  extract2.setNegative (false);

	  // Write the planar inliers to disk
	  extract2.filter (*cloud_sphere);
	  viewer.showCloud(cloud_sphere);//���ӻ���ʾ���һ��С��
	  std::cout << "PointCloud representing the sphere component: " << cloud_sphere->points.size () << " data points." << std::endl;
	  std::cout << coefficients2 << std::endl;//��ӡ��������ĸ�����
	  // Remove the planar inliers, extract the rest
	  extract2.setNegative (true);
	  extract2.filter (*cloud_f2);
	  cloud_filtered = cloud_f2;

  }
	
 // �������������ļ�⣬���ھֲ�����ϵ�ڵ�����洢��coefficients_all,�����ȫ��������xyz_all
//****************************************************��ʼ���б任�������
		Eigen::Matrix4f transformationCorrespondence; 
		pcl::TransformationFromCorrespondences transformationFromCorr; 
         for ( i =0;i<coefficients_all.size();i++) 
        { 
			Eigen::Vector3f from(coefficients_all.at(i).values[0], 
				coefficients_all.at(i).values[1],
				coefficients_all.at(i).values[2]); 

			Eigen::Vector3f  to (xyz_all.at(i).x, 
				xyz_all.at(i).y,
				xyz_all.at(i).z); 

			transformationFromCorr.add(from, to, 1.0);//all the same weight 

        } 

        transformationCorrespondence= transformationFromCorr.getTransformation().matrix(); 
        std::cout<< "\ntransformation from corresponding points is \n"<<transformationCorrespondence<<std::endl; 
		
		std::cout<< "\n�������Ϊ�궨������ȷ��������y�������ʾ�������ݣ���������n��\n"<<std::endl; 
		char userin;
		std::cin.clear();

		std::cin.get(userin);
		if(userin=='y'||userin=='Y'){

			std::string filename;
			std::cin.clear();
			std::cout<< "\n�������ļ�����\n"<<std::endl;
			std::cin>>filename;
			std::ofstream myfile( filename, std::ios::binary);
			myfile.write((char *)transformationCorrespondence.data(),transformationCorrespondence.size()*sizeof(float));
			myfile.close();
		}
		else
		{
			std::cout<< "\n���������\n"<<std::endl; 
		}


    }
    
    void run (const std::string& device_id)
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id);

      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)>
        f = boost::bind (&KinectViewerCuda::cloud_cb_, this, _1, _2, _3);

      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      
      while (true)
      {
        pcl_sleep (1);
      }

      interface->stop ();
    }

    pcl::cuda::DisparityToCloud d2c;
	pcl::visualization::CloudViewer viewer;
    boost::mutex mutex_;
    bool downsample_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
	bool global_;
    std::vector <pcl::PointXYZ> xyz_all;
    pcl::PointXYZ xyz;
};

int main (int argc, char** argv)
{
	std::string device_id = "#1";
	
  int downsample = false;
	if (argc >= 2)
	{
		device_id = argv[1];
	}
	if (argc >= 3)
	{
		downsample = atoi (argv[2]);
	}
  KinectViewerCuda v (downsample);
  v.run (device_id);
   std::cout<< "/n�����˳�/n"<<std::endl; 
  return 0;
}
