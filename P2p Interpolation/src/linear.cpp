#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <fstream>
#include<chrono>

#include "interpolator.hpp"



int main(int argc, char** argv) {


    static const std::string wcPath = "/home/student/Desktop/P2P/WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcPath);
    if ( wc.isNull() )
    {
        RW_THROW("Error loading workcell");
        return -1;
    }
    // Loading tool frame
    rw::kinematics::Frame *toolFrame = wc->findFrame<rw::kinematics::Frame>("GraspTCP");
    if ( toolFrame == nullptr ){
        RW_THROW("Error finding frame: Tool");
        return -1;
    }
    //Load Cylinder Frame
    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    if ( cylinderFrame == nullptr ){
        RW_THROW("Error finding frame: Cylinder");
        return -1;
    }
    // Loading target
    rw::kinematics::MovableFrame *targetFrame = wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");
    if ( targetFrame == nullptr ){
        RW_THROW("Error finding frame: GraspTarget");
        return -1;
    }
    // Loading UR
    const std::string deviceName = "UR-6-85-5-A";
    rw::models::SerialDevice::Ptr UR5 =wc->findDevice<rw::models::SerialDevice>(deviceName);
    if ( UR5 == nullptr ){
        RW_THROW("Device UR5 not found.");
        return -1;
    }
    // Get state
    rw::kinematics::State state = wc->getDefaultState();

    rw::math::Vector3D<> placePos(0.30, -0.50, 0.150);

    rw::math::Transform3D<> placeFrame(placePos, cylinderFrame->getTransform(state).R());



    // Move Cylinder Frame for different pick locations
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.000, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);

    auto PointTimes = interpolator::util::getPointTimes(cylinderFrame->getTransform(state), UR5->baseTend(state), placeFrame);
    std::vector<rw::math::Transform3D<>> Points = std::get<0>(PointTimes);
    std::vector<float> Times = std::get<1>(PointTimes);



    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::math::Transform3D<> T_World_Table = rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.1));

    std::vector<rw::math::Q> configurationPath;
    std::vector<rw::math::Transform3D<>> Tfs = Points;
    for ( unsigned int i = 0; i < Tfs.size(); i++ )
        Tfs[i] = T_World_Table*Tfs[i];

    for ( rw::math::Transform3D<> Tf : Tfs )
    {

        targetFrame->moveTo(Tf, state);
        std::vector<rw::math::Q> solutions = interpolator::util::getConfigurations("GraspTarget", "GraspTCP", UR5, wc, state);
        rw::math::Q configuration = interpolator::util::getCollisionFreeSolution(UR5, state, detector, solutions);
        configurationPath.push_back(configuration);
        //std::cout << configuration << std::endl;
    }

    std::vector<int> LItime; //Linear interpolate time
    std::vector<int> PBtime; //Parabolic blend time
    std::ofstream LIfile, PBfile;
    for (unsigned int i = 0; i < 60; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();

        interpolator::linearInterpolate(Tfs, Times);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        int dur_ms = duration.count();
        LItime.push_back(dur_ms);
        std::cout << "(" << i <<  ") Linear interpolation execution time: " << dur_ms << " [micros]" << std::endl;
    }
    LIfile.open("../LIexetime_micros.txt");
    for ( int msTime : LItime )
        LIfile << msTime << std::endl;

    for (unsigned int i = 0; i < 60; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();

        interpolator::parabolicBlend(Tfs, Times);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        int dur_ms = duration.count();
        PBtime.push_back(dur_ms);
        std::cout << "(" << i <<  ") Parabolic blend interpolation execution time: " << dur_ms << " [micros]" << std::endl;
    }
    PBfile.open("../PBexetime_micros.txt");
    for ( int msTime : PBtime )
        PBfile << msTime << std::endl;




    //  Interpolate in Cartesian space      linearInterpolate
    //  Interpolate in joint space          linearInterpolateQ
    //  Interpolate using Parabolic blend   parabolicBlend
    auto paths = interpolator::linearInterpolate(Tfs, Times, targetFrame, UR5, wc, state, detector);
    std::vector<rw::math::Transform3D<>> blendPath = interpolator::parabolicBlend(Tfs, Times);
    std::vector<rw::math::Q> jointPath = std::get<0>(paths);
    std::vector<rw::math::Transform3D<>> cartPath = std::get<1>(paths);
    std::cout << "Linear interpolate: " << jointPath.size() << std::endl;
    std::cout << "Linear interpolate: " << cartPath.size() << std::endl;
    // Write to file

    std::ofstream blendQFile;
    blendQFile.open("blendQ.txt");
    std::vector<rw::math::Q> BlendQPath = interpolator::util::mapCartesianToJoint(blendPath, targetFrame, UR5, wc, state, detector);
    for ( rw::math::Q jointQ : BlendQPath )
        blendQFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    blendQFile.close();

    std::ofstream tfFile, qFile, blendFile;
    qFile.open("../LinIntQ.txt");

    for ( rw::math::Q jointQ : jointPath )
    {
        qFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    }
    qFile.close();
    tfFile.open("../LinIntTF.txt");
    for ( rw::math::Transform3D<> tf : cartPath )
    {
        tfFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        tfFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        tfFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        tfFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    tfFile.close();
    blendFile.open("../blend_tau25.txt");
    for ( rw::math::Transform3D<> tf : blendPath )
    {
        blendFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        blendFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        blendFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        blendFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    blendFile.close();

    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 0.1;
    for ( rw::math::Q configuration : jointPath ) // configurationPath
    {
        UR5->setQ(configuration, state);
        statePath.push_back(rw::trajectory::TimedState(time, state));
        time += dur/double(1);
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, "/home/student/Desktop/P2P/WorkCell/visu.rwplay");

	return 0;
}
