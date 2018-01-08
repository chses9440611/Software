//Bad Result
tf::Transform icpWithKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{
	clock_t begin = clock();

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_test_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_target_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_source_normals_keypoint(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_target_normals_keypoint(new pcl::PointCloud<pcl::PointXYZINormal>);

	pcl::PointCloud<pcl::Normal>::Ptr source_normals_keypoint(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normals_keypoint (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoint(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoint(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);

	// keypoint
	double resolution_source = computeCloudResolution(cloud_source);
	double resolution_target = computeCloudResolution(cloud_target);

	//ISS keypoint
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
	iss.setInputCloud(cloud_source);
	iss.setSearchMethod(tree_xyz);
	iss.setSalientRadius(5 * resolution_source);
	iss.setNonMaxRadius(2 * resolution_source);
	iss.setMinNeighbors(5);
	iss.setThreshold21(0.975);
	iss.setThreshold32(0.975);
	iss.compute(*source_keypoint);

	iss.setInputCloud(cloud_target);
	iss.setSearchMethod(tree_xyz);
	iss.setSalientRadius(5 * resolution_target);
	iss.setNonMaxRadius(2 * resolution_target);
	iss.setMinNeighbors(5);
	iss.setThreshold21(0.975);
	iss.setThreshold32(0.975);
	iss.compute(*target_keypoint);

	// normal of keypoint PCL
	
	pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(source_keypoint);
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(0.6);
	ne.compute(*source_normals_keypoint);
	pcl::concatenateFields( *source_keypoint, *source_normals_keypoint, *cloud_with_source_normals_keypoint );

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_target_keypoint;
	ne.setInputCloud(target_keypoint);
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(0.6);
	ne.compute(*target_normals_keypoint);
	pcl::concatenateFields( *target_keypoint, *target_normals_keypoint, *cloud_with_target_normals_keypoint );

	//normal of PCL
	ne.setInputCloud(cloud_source);
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(0.6);
	ne.compute(*source_normals);
	pcl::concatenateFields( *cloud_source, *source_normals, *cloud_with_source_normals );

	ne.setInputCloud(cloud_target);
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(0.6);
	ne.compute(*target_normals);
	pcl::concatenateFields( *cloud_target, *target_normals, *cloud_with_target_normals );

	//Point Feature Histogram
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setRadiusSearch(0.2);
	fpfh.setInputCloud(source_keypoint);
	fpfh.setInputNormals (source_normals_keypoint);
	fpfh.setSearchMethod (tree_xyz);
	fpfh.compute (*source_fpfh);

	fpfh.setRadiusSearch(0.2);
	fpfh.setInputCloud(target_keypoint);
	fpfh.setInputNormals (target_normals_keypoint);
	fpfh.setSearchMethod (tree_xyz);
	fpfh.compute (*target_fpfh);

 	// SampleConsensusInitialAlignment
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sac_ia(new pcl::PointCloud<pcl::PointXYZ>);
 	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia; 
 	sac_ia.setInputSource(source_keypoint);
 	sac_ia.setSourceFeatures(source_fpfh);
 	sac_ia.setInputTarget(target_keypoint);
 	sac_ia.setTargetFeatures(target_fpfh);
 	sac_ia.setCorrespondenceRandomness(6);
 	sac_ia.align(*cloud_sac_ia);
 	Eigen::Matrix4f trans_initial = sac_ia.getFinalTransformation();
 	

 	//ICP
 	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZINormal>);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZINormal>);
	tree_1->setInputCloud(cloud_with_source_normals_keypoint);
	tree_2->setInputCloud(cloud_with_target_normals_keypoint);
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
	icp.setInputCloud(cloud_with_source_normals_keypoint);
	icp.setInputTarget(cloud_with_target_normals_keypoint);
	icp.setMaxCorrespondenceDistance(1500); 
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(500);
	icp.setRANSACOutlierRejectionThreshold(0.001);
	icp.align(*cloud_icp, trans_initial);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"icpWithKeyPoint compute time = " << elapsed_secs<<", score =  "<<icp.getFitnessScore() <<endl;
	
	
	Eigen::Matrix4f trans_eigen = icp.getFinalTransformation();
	//cout << trans_eigen << endl;
	pcl::transformPointCloud(*cloud_with_source_normals, *cloud_test_with_normals, trans_initial);
	matrix_record = matrix_record * trans_initial;
	tf::Transform trans = transformEigenToTF(matrix_record);

	pub_cloud_test.publish(*cloud_test_with_normals);
	pub_cloud_target.publish(*cloud_with_target_normals);
	
	
	//Visualization 
	if (first_Set_Viewer)
	{
		first_Set_Viewer = false;
		//viewer->addPointCloud<pcl::PointXYZ> (cloud_sac_ia, "cloud");
		viewer->addPointCloud<pcl::PointXYZ> (source_keypoint, "cloud_2");
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_2");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_2");
	}
	//viewer->updatePointCloud(cloud_sac_ia, "cloud");
	viewer->updatePointCloud(source_keypoint, "cloud_2");
	/*
	plotter.clearPlots();
	plotter.addFeatureHistogram(*source_fpfh, 300);
	plotter.plot();
	*/

	/*
	// CorrespondenceEstimation
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_est;
	corr_est.setInputCloud(source_fpfh); 
	corr_est.setInputTarget(target_fpfh); 
	boost::shared_ptr<pcl::Correspondences> cru_correspondences (new pcl::Correspondences);
	corr_est.determineReciprocalCorrespondences(*cru_correspondences);
	*/
	return trans;
}

/*
void test3(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_target)
{
		//ICP
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp
		 (new pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal>);
	icp->setMaxCorrespondenceDistance(1500); 
	icp->setTransformationEpsilon(1e-10);
	icp->setEuclideanFitnessEpsilon(0.1);
	icp->setMaximumIterations(500);
	icp->setRANSACOutlierRejectionThreshold(0.001);
	size++;
	elch.addPointCloud(cloud_target);
	elch.setReg (icp);
	elch.setLoopStart (0);
	elch.setLoopEnd (size-1 );
	elch.compute();
	pcl::registration::ELCH::LoopGraph loopGraph= elch.getLoopGraph(); 
	grid_map::GridMap& gridMap ;
  	for (int i = 0; i < boost::num_vertices (*loopGraph); i++) 
  	{ 
        gridMap+=*(*loopGraph)[i].cloud; 
  	} 
  	viewer->removeAllPointClouds();
  	viewer->addPointCloud(gridMap.makeShared()); 
  	viewer->resetCamera();
}*/

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
 
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;
 
		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;
 
	return resolution;
}
tf::Transform test2(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{
	clock_t begin = clock();

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_test_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_target_normals(new pcl::PointCloud<pcl::PointXYZINormal>);

	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);

	// normal of PCL and combine to pointxyz
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_source;
	ne_source.setInputCloud(cloud_source);
	ne_source.setSearchMethod(tree_xyz);
	ne_source.setRadiusSearch(0.6);
	ne_source.compute(*source_normals);
	pcl::concatenateFields( *cloud_source, *source_normals, *cloud_with_source_normals );

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_target;
	ne_source.setInputCloud(cloud_target);
	ne_source.setSearchMethod(tree_xyz);
	ne_source.setRadiusSearch(0.6);
	ne_source.compute(*target_normals);
	pcl::concatenateFields( *cloud_target, *target_normals, *cloud_with_target_normals );

	if (first_Set_Viewer){
		size++;
		first_Set_Viewer = false;
		elch.addPointCloud(cloud_with_source_normals);
	}
	//test3(cloud_with_target_normals);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	//cout<<"icpWithNormal compute time = " << elapsed_secs<<", score =  "<<icp->getFitnessScore() <<endl;
	cout << elch.getLoopTransform() <<endl;
	Eigen::Matrix4f trans_eigen = elch.getLoopTransform() ;
	cout << trans_eigen << endl;
	pcl::transformPointCloud(*cloud_with_source_normals, *cloud_test_with_normals, trans_eigen);
	//matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(trans_eigen);

	pub_cloud_test.publish(*cloud_test_with_normals);
	pub_cloud_target.publish(*cloud_with_target_normals);
	return trans;

}
tf::Transform test(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_seg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_seg(new pcl::PointCloud<pcl::PointXYZ>);

	clock_t begin = clock();
	pcl::ModelCoefficients::Ptr source_coefficients (new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr target_coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr source_inliers (new pcl::PointIndices);
	pcl::PointIndices::Ptr target_inliers (new pcl::PointIndices);
	  
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (1);
	seg.setInputCloud (cloud_source);
	seg.segment (*source_inliers, *source_coefficients);

	seg.setInputCloud (cloud_target);
	seg.segment (*target_inliers, *target_coefficients);

	//Extract
	pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_source);
    extract.setIndices (source_inliers);
    extract.setNegative (false);
    extract.filter (*source_seg);

    extract.setInputCloud (cloud_target);
    extract.setIndices (target_inliers);
    extract.setNegative (false);
    extract.filter (*target_seg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_1->setInputCloud(cloud_source);
	tree_2->setInputCloud(cloud_target);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(source_seg);
	icp.setInputTarget(target_seg);
	icp.setMaxCorrespondenceDistance(1500); 
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(500);
	icp.setRANSACOutlierRejectionThreshold(0.01);
	icp.align(*cloud_icp);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"icpWithSeg compute time = " << elapsed_secs<<", score =  "<<icp.getFitnessScore() <<endl;

	Eigen::Matrix4f trans_eigen = icp.getFinalTransformation() ;
	//cout << trans_eigen << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_test, trans_eigen);
	matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(matrix_record);
 
	pub_cloud_test.publish(*cloud_test);
	pub_cloud_target.publish(*cloud_target);

	return trans;
}