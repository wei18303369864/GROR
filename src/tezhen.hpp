#include"tezhen.h"

void tez(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointIndicesPtr tez_Idx)
{
	//(new pcl::PointCloud<pcl::PointXYZ>);


	// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Wedge/W0.2S45T45.pcd", *cloud);
	// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/TwoPlane22.pcd", *cloud);
	 //  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CubeSharpEdge.pcd", *cloud);
	 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/OnePlane.pcd", *cloud);
	  // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/bunny.pcd",*cloud);
	  // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Statue.pcd", *cloud);
		// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/dragon.pcd", *cloud);
	 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/IntersectionThreePlanes.pcd", *cloud);


	 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AddingNoise/Bunny03Noise50.pcd", *cloud);
	//pcl::io::loadPCDFile("0_cam_cloudTemp_r1(4).pcd", *cloud);
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
	//pcl::VoxelGrid<pcl::PointXYZ> sor;  //创建滤波对象
	//sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	//sor.setLeafSize(5.8f,5.8f, 5.8f);  //设置滤波时创建的体素体积为1cm的立方体
	//sor.filter(*cloud);

	//pcl::io::savePCDFile("111.txt", *cloud);

	/*pcl::UniformSampling<pcl::PointXYZ> US;
	US.setInputCloud(cloud);
	US.setRadiusSearch(1.51f);
	US.filter(*cloud);*/

	// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/SpherMultiple.pcd", *cloud);

	  // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/trim-starC.pcd", *cloud);
	  //	pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/VaseC.pcd", *cloud);
	   //  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/twirlC.pcd", *cloud);
		//  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/fandiskC.pcd", *cloud);
	  //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/sharp_sphereC.pcd", *cloud);

	std::cout << "点数 is:" << cloud->points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr Normals(new pcl::PointCloud<pcl::PointXYZ>);
	Normals->resize(cloud->size());

	// K nearest neighbor search
	int KNumbersNeighbor = 120; // numbers of neighbors 7 , 120
	std::vector<int> NeighborsKNSearch(KNumbersNeighbor);
	std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighbor);

	int* NumbersNeighbor = new  int[cloud->points.size()];
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;



	double* SmallestEigen = new  double[cloud->points.size()];
	double* MiddleEigen = new  double[cloud->points.size()];
	double* LargestEigen = new  double[cloud->points.size()];

	double* DLS = new  double[cloud->points.size()];
	double* DLM = new  double[cloud->points.size()];
	double* DMS = new  double[cloud->points.size()];
	double* Sigma = new  double[cloud->points.size()];

	//		std::vector<double> SmallestEigen;
	//		std::vector<double> MiddleEigen;
	//		std::vector<double> LargestEigen;
	//
	//		std::vector<double> DLS;
	//		std::vector<double> DML;
	//		std::vector<double> DMS;

			//  ************ All the Points of the cloud *******************
	std::vector<float> pointRadiusSquaredDistance;
	for (size_t i = 0; i < cloud->points.size(); ++i) {

		searchPoint.x = cloud->points[i].x;
		searchPoint.y = cloud->points[i].y;
		searchPoint.z = cloud->points[i].z;

		if (kdtree.nearestKSearch(searchPoint, KNumbersNeighbor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0) {
			NumbersNeighbor[i] = NeighborsKNSearch.size();
		}
		else { NumbersNeighbor[i] = 0; }
		//if ( kdtree.radiusSearch(searchPoint,5, NeighborsKNSearch, pointRadiusSquaredDistance/*, sizeof(searchPoint)*/)> 0)
		//{
		//	NumbersNeighbor[i] = NeighborsKNSearch.size();
		//}
		//else { NumbersNeighbor[i] = 0; }
		float Xmean; float Ymean; float Zmean;
		float sum = 0.00;
		// 计算特征向量
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += cloud->points[NeighborsKNSearch[ii]].x;
		}
		Xmean = sum / NumbersNeighbor[i];
		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += cloud->points[NeighborsKNSearch[ii]].y;
		}
		Ymean = sum / NumbersNeighbor[i];
		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += cloud->points[NeighborsKNSearch[ii]].z;
		}
		Zmean = sum / NumbersNeighbor[i];

		float	CovXX;  float CovXY; float CovXZ; float CovYX; float CovYY; float CovYZ; float CovZX; float CovZY; float CovZZ;

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].x - Xmean) * (cloud->points[NeighborsKNSearch[ii]].x - Xmean));
		}
		CovXX = sum / (NumbersNeighbor[i] - 1);

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].x - Xmean) * (cloud->points[NeighborsKNSearch[ii]].y - Ymean));
		}
		CovXY = sum / (NumbersNeighbor[i] - 1);

		CovYX = CovXY;

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].x - Xmean) * (cloud->points[NeighborsKNSearch[ii]].z - Zmean));
		}
		CovXZ = sum / (NumbersNeighbor[i] - 1);

		CovZX = CovXZ;

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].y - Ymean) * (cloud->points[NeighborsKNSearch[ii]].y - Ymean));
		}
		CovYY = sum / (NumbersNeighbor[i] - 1);

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].y - Ymean) * (cloud->points[NeighborsKNSearch[ii]].z - Zmean));
		}
		CovYZ = sum / (NumbersNeighbor[i] - 1);

		CovZY = CovYZ;

		sum = 0.00;
		for (size_t ii = 0; ii < NeighborsKNSearch.size(); ++ii) {
			sum += ((cloud->points[NeighborsKNSearch[ii]].z - Zmean) * (cloud->points[NeighborsKNSearch[ii]].z - Zmean));
		}
		CovZZ = sum / (NumbersNeighbor[i] - 1);

		// Computing Eigenvalue and EigenVector
		Matrix3f Cov;
		Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

		SelfAdjointEigenSolver<Matrix3f> eigensolver(Cov);
		if (eigensolver.info() != Success) abort();

		double EigenValue1 = eigensolver.eigenvalues()[0];
		double EigenValue2 = eigensolver.eigenvalues()[1];
		double EigenValue3 = eigensolver.eigenvalues()[2];

		double Smallest = 0.00; double Middle = 0.00; double Largest = 0.00;
		if (EigenValue1 < EigenValue2) { Smallest = EigenValue1; }
		else { Smallest = EigenValue2; }
		if (EigenValue3 < Smallest) { Smallest = EigenValue3; }


		if (EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
			Smallest = EigenValue1;
			if (EigenValue2 <= EigenValue3) { Middle = EigenValue2; Largest = EigenValue3; }
			else { Middle = EigenValue3; Largest = EigenValue2; }
		}

		if (EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
		{
			Largest = EigenValue1;
			if (EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
			else { Smallest = EigenValue3; Middle = EigenValue2; }
		}

		if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
		{
			Middle = EigenValue1;
			if (EigenValue2 >= EigenValue3) { Largest = EigenValue2; Smallest = EigenValue3; }
			else { Largest = EigenValue3; Smallest = EigenValue2; }
		}

		SmallestEigen[i] = Smallest;
		MiddleEigen[i] = Middle;
		LargestEigen[i] = Largest;

		//int v =sizeof(eigensolver.eigenvectors());
		//cout << v << endl;
		//Eigen::Matrix<double, Dynamic, Dynamic> matrix_name(cloud->points.size(), 3);
		//for (int i = 0; i < 3; i++) {
		//	Eigen::Vector3f faxian;
		//	if (eigensolver.eigenvalues()[i] == Smallest)
		//		matrix_name <<eigensolver.eigenvectors().col(2)[0];  //取出最大的特诊向量
		//	matrix_name << eigensolver.eigenvectors().col(2)[1];
		//	matrix_name << eigensolver.eigenvectors().col(2)[2];
		//		/* = eigensolver.eigenvectors();*/
		//}


		//cout << "特征向量eigenvectors = \n" << eigensolver.eigenvectors() << endl;
		DLS[i] = std::abs(SmallestEigen[i] / LargestEigen[i]);          // std::abs ( LargestEigen[i] -  SmallestEigen[i] ) ;
		DLM[i] = std::abs(MiddleEigen[i] / LargestEigen[i]);             // std::abs (  LargestEigen[i] - MiddleEigen[i] ) ;
		DMS[i] = std::abs(SmallestEigen[i] / MiddleEigen[i]);       // std::abs ( MiddleEigen[i] -  SmallestEigen[i] ) ;
		Sigma[i] = (SmallestEigen[i]) / (SmallestEigen[i] + MiddleEigen[i] + LargestEigen[i]);
	} // For each point of the cloud

	std::cout << " 计算sigama! " << std::endl;
	// 特征值

	double MaxD = 0.00;
	double MinD = cloud->points.size();
	int Ncolors = 15;

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (Sigma[i] < MinD) MinD = Sigma[i];
		if (Sigma[i] > MaxD) MaxD = Sigma[i];
	}

	std::cout << " Minimum is :" << MinD << std::endl;
	std::cout << " Maximum  is :" << MaxD << std::endl;

	//   *****************************************
		/*

		 double ss = 0.00 ;
		  for (size_t i = 0; i < cloud ->points.size (); ++i) {
			  ss += Sigma [i] ;}
		  double avg = ss / cloud ->points.size () ;
		  ss = 0.00 ;
		  for (size_t i = 0; i < cloud ->points.size (); ++i) {
			  ss += (Sigma [i] -  avg ) * (  Sigma [i] -  avg ) ;}
		  double stddvtion =   sqrt (  ss  /  ( cloud ->points.size () - 1 )  ) ;

		  std::cout<< " Standard Deviation is :" << stddvtion << std::endl;

		   MaxD = ( 2 )* stddvtion;
		  //MaxD = 10* stddvtion;

			double line;
			double code[Ncolors][3];
		   ifstream colorcode ( "/Path/TO/ArtificialPointClouds/JetColorDensity/ColorCodes256.txt" );
		   //store color codes in array
			int i=0,j=0;
			while( colorcode>> line ) {
			code[i][j]=line;
			j++;
			if (j == 3)
			i++;
		}
			code[1][0] = 0;
			code[1][1] = 0;
			code[1][2] = 135.468;



			// jet color map

		int level = 0;
		float step = ( ( MaxD -  MinD) / Ncolors ) ;
			for (size_t i = 0; i < cloud ->points.size (); ++i) {
	if (  SmallestEigen [i] <= MaxD ) {
			level = floor( (SmallestEigen [i] - MinD ) /  step ) ;

			cloud->points[i].r = code[ level ][0];
			cloud->points[i].g =  code[ level ][1];
			cloud->points[i].b =  code[ level ][2];
	} // if sigma less than Max
			}
	*/
	//    *****************************************


	//   *****************************************
			//int Edgepoints = 0;
			// Red and white (khaki)
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	cloud1->width = cloud->points.size();
	cloud1->height = 1;
	cloud1->points.resize(cloud1->height*cloud1->width);

	//for (size_t i = 0; i < cloud->points.size(); ++i) {
	//	cloud->points[i].r = 240;
	//	cloud->points[i].g = 230;
	//	cloud->points[i].b = 140;
	//}

	int level = 0;
	float step = ((MaxD - MinD) / Ncolors);
	//  level = floor( (Sigma [i] - MinD ) /  step ) ;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (Sigma[i] > (MinD + (6 * step))) {  //6*step
			/*cloud->points[i].r = 255;
			cloud->points[i].g = 0;
			cloud->points[i].b = 0;*/

			tez_Idx->indices.push_back(i);

			cloud1->points[i].x = cloud->points[i].x;
			cloud1->points[i].y = cloud->points[i].y;
			cloud1->points[i].z = cloud->points[i].z;
			/*cloud1->points[i].r = 255;
			cloud1->points[i].g = 0;
			cloud1->points[i].b = 0;*/

			//    Dim gray....
	//		    cloud->points[i].r = 105;
	//		    cloud->points[i].g = 105 ;
	//		    cloud->points[i].b = 105;
			//Edgepoints ++;
		}
	}

	//   *****************************************

	pcl::io::savePCDFile("tez.pcd", *cloud1);
	//std::cout<< " Number of Edge points  is :" << Edgepoints << std::endl;

			// writing the Sigma on the disk
	//	    	   		std::ofstream ofsSigma;
	//	    	   		ofsSigma.open("/Path/TO/SigmaDragon.txt");
	//	    	            for (size_t i = 0; i < cloud ->points.size (); ++i) {
	//	    	            	ofsSigma << Sigma [i]<< ","<< std::endl ;
	//	    	                    }




		//pcl::PLYWriter writePLY;
	 //  writePLY.write ("/Path/TO/RatioSmallestEigen22.ply", *cloud,  false);
		 //   writePLY.write ("/Path/TO/CloudEigeJnetTwirl.ply", *cloud,  false);
		// writePLY.write ("/Path/TO/CloudEigeJnetDragon.ply", *cloud,  false);
	  // writePLY.write ("/Path/TO/EigenTwoPlane90N10.ply", *cloud,  false);
		 // writePLY.write ("/Path/TO/DragonRedWhite.ply", *cloud,  false);
		// writePLY.write ("/Path/TO/IntersectionThreePlanes.ply", *cloud,  false);
		// writePLY.write ("/Path/TO/BunnyNoise50.ply", *cloud,  false);
		//writePLY.write ("/Path/TO/BunnyEdges.ply", *cloud,  false);
		 //writePLY.write ("1.ply", *cloud,  false);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud1);
	while (!viewer.wasStopped())
	{
	}

	//return 0;
}
