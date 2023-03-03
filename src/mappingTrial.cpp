#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>


#define FIND_MAP_ID(width, x, y) (int) ((width) * (x) + (y))
#define LIDAR_HEIGHT (double) 0.6
#define RESOLUTION (double) 0.1

using namespace std;


typedef struct euler{
    double yaw;
    double pitch;
    double roll;
}Euler;


class Orientation{
    
    private:
        Euler rotation;
        tf::Vector3 transformation;
    
    public:
        Orientation(Euler rotation, tf::Vector3 transformation){
            this->rotation = rotation;
            this->transformation = transformation;
        }

        Orientation(){};

        void setRotation(Euler rotation){
            this->rotation = rotation;
        }

        void setRotation(tf::Quaternion rotation){
            this->rotation = convertRotation(rotation);
        }

        Euler convertRotation(tf::Quaternion rotation){
            Euler eulerRotation;
            tf::Matrix3x3 m(rotation);
            m.getRPY(eulerRotation.roll,eulerRotation.pitch,eulerRotation.yaw);
            return eulerRotation;
        }

        void setTransformation(tf::Point transformation){
            this->transformation = transformation;
        }

        Euler getRotation(){
            return this->rotation;
        }

        tf::Vector3 getTransformation(){
            return this->transformation;
        }
};

class Map{

    private:
        nav_msgs::OccupancyGrid *currentMap;
        double resolution;
        double lidarHeight;
        double min_x,min_y,max_x,max_y;

    public:

        Map(nav_msgs::OccupancyGrid* map, double lidarHeight){
            currentMap = map;
            this->lidarHeight = lidarHeight;
            this->resolution = map->info.resolution;
            min_x = map->info.origin.position.x;
            min_y = map->info.origin.position.y;
            max_x = (map->info.width / map->info.resolution) + min_x;
            max_y = (map->info.height / map->info.resolution) + min_y;
        }

        Map(double lidarHeight,double resolution){
            this->lidarHeight = lidarHeight;
            this->resolution = resolution;
            currentMap = NULL;
        }

        void printMapInfo(){
            ROS_INFO("minX: %f, minY: %f, max-x: %f, max_y: %f",min_x,min_y,max_x,max_y);
            ROS_INFO("origin_x: %f, origin_y:%f",currentMap->info.origin.position.x,currentMap->info.origin.position.y);
        }

        nav_msgs::OccupancyGrid *getMap(){
            return currentMap;
        }

        void setMap(nav_msgs::OccupancyGrid* map){
            currentMap = map;
            min_x = map->info.origin.position.x;
            min_y = map->info.origin.position.y;
            max_x = (map->info.width / map->info.resolution) + min_x;
            max_y = (map->info.height / map->info.resolution) + min_y;
        }

        double getLidarHeight(){
            return lidarHeight;
        }

        void setLidarHeight(double lidarHeight){
            this->lidarHeight = lidarHeight;
        }

        void updateMap(Orientation *lidarOrientation, const sensor_msgs::LaserScan::ConstPtr& laserScan){
            int scanSize;
            int mapPlace;
            double r2,yaw;
            // Create or Update map

            if(currentMap == NULL){                
                currentMap = new nav_msgs::OccupancyGrid;
                calculateMapSize(lidarOrientation,laserScan);
                int dataSize = currentMap->info.width * currentMap->info.height;

                for(int i=0; i<dataSize;i++){
                    currentMap->data[i] = 50;
                }
            }

            calculateMapSize(lidarOrientation,laserScan);
            // currentMap->data[0] = 100;
            // currentMap->data[30] = 100;
 
        

            // Calculate from laser to map

            scanSize = laserScan->ranges.size();
            double beta = laserScan->angle_min;

            ROS_INFO("Scan Size: %d",scanSize);

            for(int i=0; i<scanSize;i++){

                ROS_INFO("Yaw: %f",lidarOrientation->getRotation().yaw);

                if(laserScan->ranges[i] < laserScan->range_max && laserScan->ranges[i] > laserScan->range_min){


                    tf::Vector3 mapPoint = beamToMapPoint90(lidarOrientation,laserScan->ranges[i], beta);
                    tf::Vector3 realPoint = beamToRealWorld90(lidarOrientation,laserScan->ranges[i], beta);

                    mapPlace = (currentMap->info.width * (int)mapPoint.getY() + mapPoint.getX());
            

                    if(mapPoint.getZ() < lidarHeight){
                        
                        currentMap->data[mapPlace] -= 20;
                        currentMap->data[mapPlace] = max((int)currentMap->data[mapPlace],0);
                        // ROS_INFO("Free Real: x: %f, y:%f, z:%f, width: %d, height: %d",realPoint.x(),realPoint.y(),realPoint.z(),currentMap->info.width,currentMap->info.height);
                        // ROS_INFO("Free Map: x: %f, y:%f, z:%f map: %d\n",mapPoint.x(),mapPoint.y(),mapPoint.z(),mapPlace);
                    }
                    else{

                        if(currentMap->data[mapPlace]< 60){
                            currentMap->data[mapPlace] += 1;
                        }
                        else{
                            currentMap->data[mapPlace] += 5;
                        }
                        currentMap->data[mapPlace] = min((int)currentMap->data[mapPlace],100);
                        // ROS_INFO("Obs Map: x: %f, y:%f, z:%f",realPoint.x(),realPoint.y(),realPoint.z());
                    }
                }
                else{
                    ROS_INFO("Too Far or Close");
                }
                


                beta += laserScan->angle_increment;

            }
            
        }

        void calculateMapSize(Orientation *lidarOrientation, const sensor_msgs::LaserScan::ConstPtr& laserScan){
            int threshold = 0, dataSize;

            min_x =  -10;
            min_y =  -20;
            max_x =  10;
            max_y =  20;

            geometry_msgs::Pose pose = geometry_msgs::Pose();
            pose.position.x = min_x;
            pose.position.y = min_y;

            currentMap->info.width = (max_x-min_x+threshold) / resolution ;
            currentMap->info.height = (max_y-min_y+threshold) / resolution;
            currentMap->info.resolution = resolution;
            currentMap->info.origin = pose;
            currentMap->info.map_load_time = ros::Time(0);

            dataSize = currentMap->info.width * currentMap->info.height;
            currentMap->data.resize(dataSize);

            // min_x =  min(lidarOrientation->getTransformation().x() - laserScan->range_max,min_x);
            // min_y =  min(lidarOrientation->getTransformation().y() - laserScan->range_max,min_y);
            // max_x =  max(lidarOrientation->getTransformation().x() + laserScan->range_max,max_x);
            // max_y =  max(lidarOrientation->getTransformation().y() + laserScan->range_max,max_y);

            // geometry_msgs::Pose pose = geometry_msgs::Pose();
            // pose.position.x = min_x;
            // pose.position.y = min_y;

            // currentMap->info.width = (max_x-min_x+threshold) / resolution ;
            // currentMap->info.height = (max_y-min_y+threshold) / resolution;
            // currentMap->info.resolution = resolution;
            // currentMap->info.origin = pose;
            // currentMap->info.map_load_time = ros::Time(0);

            // dataSize = currentMap->info.width * currentMap->info.height;
            // currentMap->data.resize(dataSize);

            // for(int i=0; i<dataSize;i++){
            //     currentMap->data[i] = 50;
            // }
        }


        tf::Vector3 beamToMapPoint(Orientation *lidarOrientation, double beam, double angle){
            tf::Vector3 point;
            int r2;
            angle = lidarOrientation->getRotation().yaw + angle;
            point.setZ(beam * cos(lidarOrientation->getRotation().pitch));
            r2 =  beam * sin(lidarOrientation->getRotation().pitch);
            point.setX(cos(angle) * r2 + lidarOrientation->getTransformation().getX());
            point.setY(sin(angle) * r2 + lidarOrientation->getTransformation().getY()); 

            point.setX((point.getX() - min_x) / resolution);
            point.setY((point.getY() - min_y) / resolution);


            return point;
        }

        tf::Vector3 beamToMapPoint90(Orientation *lidarOrientation, double beam, double angle){
            tf::Vector3 point;
            double r2;
            double yaw = lidarOrientation->getRotation().yaw;
            int sign = -1;

            if(yaw < 0){
                yaw += 6.2831;
                // yaw += 3.1415;
            }

            
            point.setZ(beam * cos(angle));

            r2 =  beam * sin(angle);

            point.setX(sin(yaw) * r2 * sign + lidarOrientation->getTransformation().getX());
            point.setY(cos(yaw) * r2  + lidarOrientation->getTransformation().getY()); 

            point.setX((point.getX() - min_x) / resolution);
            point.setY((point.getY() - min_y) / resolution);

            

            // if(point.z() < lidarHeight){

            //     r2 =  beam * sin(angle);

            //     point.setX(sin(yaw) * r2 * sign + lidarOrientation->getTransformation().getX());
            //     point.setY(cos(yaw) * r2  + lidarOrientation->getTransformation().getY()); 

            //     point.setX((point.getX() - min_x) / resolution);
            //     point.setY((point.getY() - min_y) / resolution);
            // }
            // else{
            //     r2 =  point.getZ() * tan(angle);
            //     yaw = lidarOrientation->getRotation().yaw;

            //     point.setX(sin(yaw) * r2 + lidarOrientation->getTransformation().getX());
            //     point.setY(cos(yaw) * r2  + lidarOrientation->getTransformation().getY()); 

            //     point.setX((point.getX() - min_x) / resolution);
            //     point.setY((point.getY() - min_y) / resolution);
            // }
            
        
        
            

            // ROS_INFO("r2: %f",r2);

            return point;
        }


        tf::Vector3 beamToRealWorld(Orientation *lidarOrientation, double beam, double angle) {
            tf::Vector3 point;
            int r2;
            point.setZ(beam * cos(lidarOrientation->getRotation().pitch));
            r2 =  beam * sin(lidarOrientation->getRotation().pitch);
            point.setX(cos(angle) * r2 + lidarOrientation->getTransformation().getX());
            point.setY(sin(angle) * r2 + lidarOrientation->getTransformation().getY()); 

            return point;

        }

        tf::Vector3 beamToRealWorld90(Orientation *lidarOrientation, double beam, double angle) {
            tf::Vector3 point;
            double r2;
            double yaw = lidarOrientation->getRotation().yaw;
        
            point.setZ(beam * cos(angle));
            r2 =  beam * sin(angle);

            point.setX(sin(yaw) * r2 + lidarOrientation->getTransformation().getX());
            point.setY(cos(yaw) * r2  + lidarOrientation->getTransformation().getY()); 


            return point;

        }

};


class Mapper{

    private:

        std::string scanTopicName = "/scan";
        tf::TransformListener listener;
        tf::Transform robotToLaser;
        tf::StampedTransform robotToOdom;
        ros::Rate *rate;
        ros::NodeHandle *nh;
        ros::Publisher mapPublisher;
    
        nav_msgs::OccupancyGrid *currentMap;
        Map *map;
    

    public:
        Mapper(ros::NodeHandle *nh){    

        // initialize ros
            this->nh = nh;
            ros::Subscriber scanSub = nh->subscribe(scanTopicName,1,
            &Mapper::laserScanCallback,this);
            rate =  new ros::Rate(20);     

            mapPublisher = nh->advertise<nav_msgs::OccupancyGrid>("/map",1000);

            // Lidar Height + threshold
            map = new Map(LIDAR_HEIGHT,RESOLUTION);
            ros::spin();
        }

       

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan){
            
            ROS_INFO("Got Scan Message\n");
            // Get transform between lidar and the odom 
            try{

                ROS_INFO("Got Transform\n");

                listener.lookupTransform("odom","hokuyo_link",ros::Time(0),robotToOdom);

                Orientation *lidarOrientation = new Orientation();

                lidarOrientation->setTransformation(robotToOdom.getOrigin());
                lidarOrientation->setRotation(robotToOdom.getRotation());

                // Convert to euler


                // cout << "x: " << lidarOrientation->getTransformation().x()  << " y: " << lidarOrientation->getTransformation().y() << endl;
                // cout << "r: " << lidarOrientation->getRotation().roll << " p: " << lidarOrientation->getRotation().pitch << " y: " << lidarOrientation->getRotation().yaw <<endl;



                // Create or Update Map
                map->updateMap(lidarOrientation,laserScan);
                // map->printMapInfo();
                
                // Publish Map
                mapPublisher.publish(*map->getMap());
                
            }
            catch(tf::TransformException e){
                ROS_ERROR("%s",e.what());
            }
            

        }          

        void loop(){
            while(nh->ok()){
                ros::spinOnce();
                rate->sleep();
            }
        }

        void getTransform(){
            Euler euler = Euler();

            
            while(nh->ok()){

                try{
                    listener.lookupTransform("odom","hokuyo_link",ros::Time(0),robotToOdom);
                }
                catch(tf::TransformException e){
                    ROS_ERROR("%s",e.what());
                }

                tf::Vector3 point;
                point = robotToOdom.getOrigin();

                tf::Quaternion q;
                q = robotToOdom.getRotation();

                // Convert to euler
                tf::Matrix3x3 m(q);
                m.getRPY(euler.roll,euler.pitch,euler.yaw);




                cout << "x: " << point.x()  << " y: " << point.y() << endl;
                cout << "r: " << euler.roll << " p: " << euler.pitch << " y: " << euler.yaw <<endl;

                ros::spinOnce();
                rate->sleep();
            }

        }


};

int main(int argc, char** argv){
    ros::init(argc,argv,"mapper");
    ros::NodeHandle nh;
    Mapper *mapper = new Mapper(&nh); 
}


