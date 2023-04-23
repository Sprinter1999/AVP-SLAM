#! /usr/bin/env python3

# Reads .g2o file formats from data/g2o, data/GT, and data/odometry in order to initialize
# a factor graph and optimize it, plotting results, and printing errors.

import matplotlib.pyplot as plt
import numpy as np
import yaml
from gtsam import NonlinearFactorGraph,PriorFactorPose2,noiseModel_Gaussian,noiseModel_Diagonal,Values,Pose2,GaussNewtonParams,GaussNewtonOptimizer,BetweenFactorPose2,ISAM2,ISAM2Params

# read from .g2o assuming that it is in the correct format and only containing lines that start with
# VERTEX_SE2 or EDGE_SE2. The first return is a list of vertexes (i,Pose2) and the second retur is
# a list of edges (BetweenFactorPose2)
#TODO:
# 读取.g2o，假设它是正确格式的，并且只包含以VERTEX_SE2或EDGE_SE2开头的行。
# vertex_SE2_list包含表示图中顶点的元组。每个元组包含一个整数ID i和一个Pose2对象，表示该顶点的位置和方向。
# edge_SE2_list包含表示图中边的BetweenFactorPose2对象。
# 每个BetweenFactorPose2对象包含四个参数：i和j（连接边缘的两个顶点的ID），一个Pose2对象，表示两个顶点之间的相对姿态，以及一个noiseModel_Gaussian.Covariance对象，表示相对姿态测量的不确定性。
def readSE2(file):
    vertex_SE2_list = []
    edge_SE2_list = []
    
    
    with open(file) as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(' ')
            if(line[0]=='VERTEX_SE2'):
                i = int(line[1])

                x = float(line[2])
                y = float(line[3])
                theta = float(line[4])

                vertex = (i,Pose2(x,y,theta))
                vertex_SE2_list.append(vertex)
            elif(line[0]=='EDGE_SE2'):
                i = int(line[1])
                j = int(line[2])

                x = float(line[3])
                y = float(line[4])
                theta = float(line[5])
                
                info = [float(x) for x in line[6:]]
                omega = np.zeros((3,3))
                for row in range(3):
                    for col in range(3):
                        if(col>=row):
                            omega[row,col] = info[0]
                            info = info[1:]
                        else:
                            omega[row,col] = omega[col,row]
                #model = NoiseModelFactor.Gaussian.Information(omega)
                model = noiseModel_Gaussian.Covariance(omega)
                
                # sigma = np.linalg.inv(omega)
                # model = NoiseModelFactor.Gaussian.Covariance(sigma)

                edge = BetweenFactorPose2(i, j, Pose2(x,y,theta), model)
                edge_SE2_list.append(edge)
            else:
                print('error',line)

    return vertex_SE2_list, edge_SE2_list

# batch solution using vertexes and edges from parse_rosbag.cpp
# GT and Odom are only used for plotting as a comparion
#TODO:
# 首先，该函数将读取的顶点和边列表分别赋值给 vertexes 和 edges 变量。
# 然后，创建一个 NonlinearFactorGraph 对象 graph，并在第一个姿态上添加一个先验因子，将其设置为原点。
# 接下来，将所有的边作为 odometry 因子添加到图中。然后，创建一个 Values 对象 initialEstimate，用于存储初始估计值。将每个顶点插入到 initialEstimate 中。
# 接着，使用 Gauss-Newton 非线性优化器对初始值进行优化，产生一个优化结果 result。最后，将优化后的位姿结果存储在一个列表 resultPoses 中，并将结果绘制为图形。
def batchSolution(output,GT,Odom):
    vertexes = output[0]
    edges = output[1]

    graph = NonlinearFactorGraph()
    # Add a prior on the first pose, setting it to the origin
    priorNoise = noiseModel_Gaussian.Covariance(initialCov)
    graph.add(PriorFactorPose2(0, Pose2(initialPose), priorNoise))

    # Add odometry factors
    for edge in edges:
        graph.add(edge)

    # Create the data structure to hold the initialEstimate estimate to the solution
    initialEstimate = Values()
    for vertex in vertexes:
        initialEstimate.insert(*vertex)

    # Optimize the initial values using a Gauss-Newton nonlinear optimizer
    parameters = GaussNewtonParams()
    parameters.setRelativeErrorTol(relativeErrorTol)
    parameters.setMaxIterations(maxIterations)
    optimizer = GaussNewtonOptimizer(graph, initialEstimate, parameters)
    result = optimizer.optimize()
    #result.print("Final Result:\n")
    
    #store resulting poses
    resultPoses = []
    for key in range(result.keys().size()):
        resultPoses.append((result.keys().at(key),result.atPose2(key)))

    #plot results
    initialX = np.array([pose[1].x() for pose in vertexes])
    initialY = np.array([pose[1].y() for pose in vertexes])
    resultX = np.array([pose[1].x() for pose in resultPoses])
    resultY = np.array([pose[1].y() for pose in resultPoses])
    GTX = np.array([pose[1].x() for pose in GT[0]])
    GTY = np.array([pose[1].y() for pose in GT[0]])
    OdomX = np.array([pose[1].x() for pose in Odom[0]])
    OdomY = np.array([pose[1].y() for pose in Odom[0]])
    
    plt.figure()
    plt.plot(initialX,initialY,"r",label='SLAM')
    plt.plot(resultX,resultY,"g",label='Loop Closure')
    plt.plot(GTX,GTY,"b",label='GT')
    plt.plot(OdomX,OdomY,"black",label='Odometry')
    plt.title('Factor Graph batch')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.draw()

    return resultPoses



#TODO:

# incremental solution using ISAM2 using vertexes and edges from parse_rosbag.cpp
# GT and Odom are only used for plotting as a comparion
def incrementalSolution(output,GT,Odom):
    # 从输出参数output中分别获取节点和边的信息；
    vertexes = output[0]
    edges = output[1]

    # 初始化iSAM2算法，并设置一些算法参数，如重线性化阈值和重线性化间隔；
    parameters = ISAM2Params()
    parameters.setRelinearizeThreshold(relinearizeThreshold)
    parameters.setRelinearizeSkip(relinearizeSkip)
    isam = ISAM2(parameters)
    isam.update()
    isam.calculateEstimate()

    # Loop over the poses, adding the observations to iSAM incrementally
    # # 对于每个节点，根据其前一个节点的位姿信息和当前节点与其它节点之间的运动约束，构建一个非线性因子图，并将初始位姿估计值插入到图中；
    for vertex in vertexes:
        graph = NonlinearFactorGraph()
        initialEstimate = Values()
        
        startIdx = 0
        if(vertex[0] == startIdx):
            priorNoise = noiseModel_Gaussian.Covariance(initialCov)
            graph.add(PriorFactorPose2(0, Pose2(initialPose), priorNoise))
            initialEstimate.insert(*vertex)
        else:
            prevPose = result.atPose2(vertex[0]-1)
            initialEstimate.insert(vertex[0],prevPose)
            for edge in edges:
                if(edge.keys().at(1)==vertex[0]):
                    graph.add(edge)

        # 调用iSAM2的update()函数，将当前图和位姿估计值传入算法中进行优化；
        # 调用iSAM2的calculateEstimate()函数，计算出优化后的位姿估计值；
        isam.update(graph,initialEstimate)
        result = isam.calculateEstimate()

    #store resulting poses
    # 将优化后的位姿信息存储在resultPoses中。
    resultPoses = []
    for key in range(result.keys().size()):
        resultPoses.append((result.keys().at(key),result.atPose2(key)))

    #plot results
    initialX = [pose[1].x() for pose in vertexes]
    initialY = [pose[1].y() for pose in vertexes]
    resultX = [pose[1].x() for pose in resultPoses]
    resultY = [pose[1].y() for pose in resultPoses]
    GTX = [pose[1].x() for pose in GT[0]]
    GTY = [pose[1].y() for pose in GT[0]]
    OdomX = [pose[1].x() for pose in Odom[0]]
    OdomY = [pose[1].y() for pose in Odom[0]]

    plt.figure()
    plt.plot(initialX,initialY,"r",label='SLAM')
    plt.plot(resultX,resultY,"g",label='Loop Closure')
    plt.plot(GTX,GTY,"b",label='GT')
    plt.plot(OdomX,OdomY,"black",label='Odometry')
    plt.title('ISAM2 incremental')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.draw()

    return resultPoses

#TODO: not used
def downSample(GT, Odom):

    downSample_GT = []
    downSample_Odom = []

    for i in range(len(output[0])):
        for j in range(len(GT[0])):
            if output[0][i][0] == GT[0][j][0]:
                downSample_GT.append(GT[0][j])
                break

        for k in range(len(Odom[0])):
            if output[0][i][0] == Odom[0][k][0]:
                downSample_Odom.append(Odom[0][k])
                break

    return downSample_GT, downSample_Odom

#
def calculateRMSE(x, y, GT_x, GT_y):
    return np.sqrt(np.mean((x - GT_x)**2 + (y - GT_y)**2))

def calculateError(batch_result, isam_result, GT, Odom, SLAM):
    # pick out GT and Odom that matches the result in timestamp
    GT_x = []
    GT_y = []
    Odom_x = []
    Odom_y = []

    for i in range(len(output[0])):
        for j in range(len(GT[0])):
            if output[0][i][0] == GT[0][j][0]:
                GT_x.append(GT[0][j][1].x())
                GT_y.append(GT[0][j][1].y())
                break

        for k in range(len(Odom[0])):
            if output[0][i][0] == Odom[0][k][0]:
                Odom_x.append(Odom[0][k][1].x())
                Odom_y.append(Odom[0][k][1].y())
                break

    GT_x = np.asarray(GT_x)
    GT_y = np.asarray(GT_y)
    Odom_x = np.asarray(Odom_x)
    Odom_y = np.asarray(Odom_y)
    
    GT_x = np.insert(GT_x, 0, 0)
    GT_y = np.insert(GT_y, 0, 0)
    GT_x = np.delete(GT_x, -1)
    GT_y = np.delete(GT_y, -1)

    Odom_x = np.insert(Odom_x, 0, 0)
    Odom_y = np.insert(Odom_y, 0, 0)
    Odom_x = np.delete(Odom_x, -1)
    Odom_y = np.delete(Odom_y, -1)

    batch_x = []
    batch_y = []
    for i in range(len(batch_result)):
        batch_x.append(batch_result[i][1].x())
        batch_y.append(batch_result[i][1].y())
    batch_x = np.asarray(batch_x)
    batch_y = np.asarray(batch_y)

    isam_x = []
    isam_y = []
    for i in range(len(isam_result)):
        isam_x.append(isam_result[i][1].x())
        isam_y.append(isam_result[i][1].y())
    isam_x = np.asarray(isam_x)
    isam_y = np.asarray(isam_y)
    
    SLAM_x = []
    SLAM_y = []
    for i in range(len(SLAM)):
        SLAM_x.append(SLAM[i][1].x())
        SLAM_y.append(SLAM[i][1].y())
    SLAM_x = np.asarray(SLAM_x)
    SLAM_y = np.asarray(SLAM_y)

    GT_Odom_error = calculateRMSE(Odom_x, Odom_y, GT_x[:len(Odom_x)], GT_y[:len(Odom_y)])
    GT_batch_error = calculateRMSE(batch_x, batch_y, GT_x, GT_y)
    GT_isam_error = calculateRMSE(isam_x, isam_y, GT_x, GT_y)
    GT_SLAM_error = calculateRMSE(SLAM_x, SLAM_y, GT_x, GT_y)
    print("SLAM: ", GT_SLAM_error)
    print("Odom: ", GT_Odom_error)
    print("batch: ", GT_batch_error)
    print("isam: ", GT_isam_error)



if __name__ == "__main__":
    #Open the config file and read in all parameters
    with open('src/AVP-SLAM-PLUS/parse_rosbag/config/configFile.yaml','r') as stream:
        try:
            config = yaml.safe_load(stream)
            dataDir = config['dataFile']+'g2o/'
            GTDir = config['dataFile']+'GroundTruth/'
            OdomDir = config['dataFile']+'odometry/'
            fileName = config['fileName']
            relativeErrorTol = config['relativeErrorTol']
            maxIterations = config['maxIterations']
            relinearizeThreshold = config['relinearizeThreshold']
            relinearizeSkip = config['relinearizeSkip']
            initialPose = np.asarray(config['initialPose']).reshape(3,)
            initialCov = np.asarray(config['initialCov']).reshape(3,3)
        except yaml.YAMLError as exc:
            print(exc)

    #the g2o file has the vertexes and poses to be used for optimization
    output = readSE2(dataDir+fileName+'.g2o')
    print('read',len(output[0]),'Vertexs and',len(output[1]),'edges')

    #GT and odometry folders have vertexes only and are just used for comparison
    GT = readSE2(GTDir+fileName+'.g2o')
    Odom = readSE2(OdomDir+fileName+'.g2o')
    
    # downSampled_GT, downSampled_Odom = downSample(GT, Odom)

    #find the optimized batch result
    batch_result = batchSolution(output,GT,Odom)
    
    #find the optimized incremental result
    isam_result = incrementalSolution(output,GT,Odom)

    plt.show()

    calculateError(batch_result, isam_result, GT, Odom, output[0])

    
    
    
    
    
    
    
    
    
    
    
