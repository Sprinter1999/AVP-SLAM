//TODO: this repo use this ODOAVP

// parse through a rosbag and form vertexes and edges to save in g2o format.
// Vertexes are from AVP-SLAM-PLUS
// Edges are added between each vertex using transformations between the closest odometry
//TODO: 需要明确这里是如何进行回环检测的，vertexes overlap by a defined threshold是什么意思？
// Loop closure edges are added when vertexes overlap by a defined threshold

#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "parser");

    // parameter variables
    std::string fileName;
    double tooFar = 0.01;
    int ignoredVertexes = 300;
    std::string dataFile = "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/data/";
    double odomCovXX = 1;
    double odomCovYY = 1;
    double odomCovTT = 1;
    double maxCovXX = 1;
    double maxCovYY = 1;
    double maxCovTT = 1;

    ros::NodeHandle nh;
    
    // get parameter from config file
    nh.param<std::string>("dataFile", dataFile, "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/data/");
    nh.param<std::string>("fileName", fileName, "");
    nh.param<double>("tooFar", tooFar, 0.01);
    nh.param<int>("ignoredVertexes",ignoredVertexes,300);
    nh.param<double>("odomCovXX",odomCovXX,1);
    nh.param<double>("odomCovYY",odomCovYY,1);
    nh.param<double>("odomCovTT",odomCovTT,1);
    nh.param<double>("maxCovXX",maxCovXX,1);
    nh.param<double>("maxCovYY",maxCovYY,1);
    nh.param<double>("maxCovTT",maxCovTT,1);

    // directories to save each type of .g2o file into
    std::string dataDir = dataFile+"rosbag/";
    std::string outDir = dataFile+"g2o/";
    std::string GTDir = dataFile+"GroundTruth/";
    std::string OdomDir = dataFile+"odometry/";

    // source of rosbag file recorded while running AVP-SLAM-PLUS
    rosbag::Bag bag;
    bag.open(dataDir+fileName+".bag", rosbag::bagmode::Read);

    // file to save the vertex and edges in g2o file format
    std::ofstream myfile;
    std::ofstream GTfile;
    std::ofstream Odomfile;
    myfile.open(outDir+fileName+".g2o");
    GTfile.open(GTDir+fileName+".g2o");
    Odomfile.open(OdomDir+fileName+".g2o");

    // initializing private variables
    Eigen::Affine3f transform_0 = Eigen::Affine3f::Identity();
    int initialized = 0;
    Eigen::Affine3f transform_t = Eigen::Affine3f::Identity();

    std::vector<double> slamX;
    std::vector<double> slamY;
    std::vector<double> slamTheta;

    int vertexCount = 0;
    bool GT_first = false; // store GT before SLAM pose so the id would start at 0 

    // iterate through the rosbag and handle each type of message appropriately
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        std::string topic = m.getTopic();
        //ros::Time time = m.getTime();
        //std::cout<<"topic"<<topic<<std::endl;
        //std::cout<<"Time "<<time<<std::endl;

        // This will catch all string rostopics
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL){
            // read the odometry vertexs that are saved in the rosbag and save the pose in SE2
            std::string text = s->data;
            
            size_t pos = text.find(" ");
            std::string name = text.substr(0, pos);
            if(name == "VERTEX_SE2"){
                text = text.erase(0,pos+1);
                pos = text.find(" ");
                //ignore time
                text = text.erase(0,pos+1);

                //trim x
                pos = text.find(" ");
                double currentX = std::stod(text.substr(0,pos));
                text = text.erase(0,pos+1);
                //trim y
                pos = text.find(" ");
                double currentY = std::stod(text.substr(0,pos));
                text = text.erase(0,pos+1);
                //save yaw
                double currentYaw = std::stod(text);

                // pose in SE2 is either saved in transform_0 or transform_t
                if(initialized==0){
                    // transform_0 is located at the previous vertex
                    transform_0.translation() << currentX, currentY, 0.0;
                    transform_0.rotate (Eigen::AngleAxisf (currentYaw, Eigen::Vector3f::UnitZ()));
                    initialized = 1;
                }
                else{
                    // transform_t is located at the current pose
                    transform_t = Eigen::Affine3f::Identity();
                    transform_t.translation() << currentX, currentY, 0.0;
                    transform_t.rotate (Eigen::AngleAxisf (currentYaw, Eigen::Vector3f::UnitZ()));
                }

                // only save vertexes because it will just be used for graphing and comparison purposes
                Odomfile << "VERTEX_SE2 " << vertexCount << " " << currentX << " " << currentY << " " << currentYaw << std::endl;
                
            }
            
        }

        // This will catch all Odometry rostopics
        nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
        //TODO: to understand the "/currentPose" topic is generated by which node; handle currentPose topic
        if (odom != nullptr && topic=="/currentPose" && GT_first == true){
            //Transform quaternion to rotation
            double roll, pitch, yaw;
            tf::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // save the AVP-SLAM current pose as a vertex in our pose graph
            myfile << "VERTEX_SE2 " << vertexCount << " " << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << yaw << std::endl;
            slamX.push_back(odom->pose.pose.position.x);
            slamY.push_back(odom->pose.pose.position.y);
            slamTheta.push_back(yaw);

            // save the odomety transformation between last pose and current pose as edge in our pose graph
            if(vertexCount>0){
                // transform_0 is the last pose from odomety
                // transform_t is the current pose from odomety
                // transform_0t is the transformation from last pose to current pose
                Eigen::Affine3f transform_0t = transform_0.inverse()*transform_t;
                transform_0 = transform_t;  // reset transform_0 to current pose for the next edge
                double theta = atan2(-transform_0t(0, 1), transform_0t(0, 0));
                // write edge to g2o file
                myfile << "EDGE_SE2 " << vertexCount-1 << " " << vertexCount << " " << transform_0t(0, 3) << " " << transform_0t(1, 3) << " " << theta << " " << odomCovXX << " 0 0 " << odomCovYY << " 0 " << odomCovTT << std::endl;
            }
            
            vertexCount++;
        }
        // handle ground truth published as odom by Gazebo (bad name)
        //TODO: to understand the "/odom" topic
        else if (odom != nullptr && topic=="/odom"){
            //Transform quaternion to rotation
            double roll, pitch, yaw;
            tf::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // only save vertexes because it will just be used for graphing and comparison purposes
            GTfile << "VERTEX_SE2 " << vertexCount << " " << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << yaw << std::endl;
            GT_first = true;
        }
    }

    // finished reading rosbag
    bag.close();
    std::cout<<"done reading "<<slamX.size()<<" odometry vertexes"<<std::endl;

    // begin looking for loop closures
    int newEdgeCount = 0;
    // iterate through each vertex from AVP-SLAM-PLUS
    for(size_t j=ignoredVertexes;j<slamX.size();j++){
        // find the clostest pose in x, y position that occurred ignoredVertexes number of vertexes before j
        int closestMatch = -1;
        double closestDist = 1000;
        for(size_t i=0;i<j-ignoredVertexes;i++){
            double dist = pow((slamX[i]-slamX[j]),2)+pow((slamY[i]-slamY[j]),2);
            // if you find a vertex that is close enough (below tooFar) and closer than closestDist
            if(dist<pow(tooFar,2)){
                if(dist<closestDist){
                    closestMatch=i;
                    closestDist = dist;
                }
            }
        }
        // if a match was found then add the edge assuming the pose transformation is 0,0 with theta calculated as the difference between j and i
        if(closestMatch!=-1){
            size_t i = closestMatch;
            double covXX = maxCovXX*abs(slamX[i]-slamX[j])/tooFar;
            double covYY = maxCovYY*abs(slamY[i]-slamY[j])/tooFar;
            myfile << "EDGE_SE2 " << i << " " << j << " " << 0 << " " << 0 << " " << slamTheta[j]-slamTheta[i] << " "<<covXX<<" 0 0 "<<covYY<<" 0 "<<maxCovTT<< std::endl;
            //std::cout<<"i "<<i<<" "<<slamX[i]<<","<<slamY[i]<<" j "<<j<<" "<<slamX[j]<<","<<slamY[j]<<std::endl;
            
            newEdgeCount++;
        }
    }

    // finished finding loop closures

    myfile.close();
    GTfile.close();
    Odomfile.close();
    
    std::cout << "added " << newEdgeCount << " edges" << std::endl;

    return 0;
}


