#include <iostream>
#include <string>
#include <map>
#include <set>

#include <ros/ros.h>

#include <opt_msgs/TrackArray.h>

#include <SerialStream.h>
#include <stdlib.h>

struct people_data
{
	double x;
	double y;
	double z;
    int id;
    int time_count;
};

people_data laste_people;
people_data pre_people;
int count_time = 0;



using namespace LibSerial;
SerialStream serial_port;

//std::map<int, people_data> people_data_map;
//std::set<int> people_have;
//int min_id = -1;

const int MAX_TIME = 1;




/*void stm32f4_cb(const opt_msgs::TrackArray::ConstPtr& msg)
{
	laste_people.z = 100.0;

    laste_people.x = 0.0;

	bool is_people = false;
    
	
	if(is_people){
        back_count = 0;
        std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~`"<<std::endl;
		
	}else{
        back_count++;
        if (back_count < 10)
        {
            //serial_port.write( "b300b300$", 9) ;
        }else{
            serial_port.write( "f000b000$", 9) ;
        }
			
	}
	
}*/

/*void stm32f4_cb(const opt_msgs::TrackArray::ConstPtr& msg)
{
    std::set current_set;
    double min_z = 100.0;
    int current_min_id = -1;
    if (people_have.empty()){
        
        for(std::vector<opt_msgs::Track>::const_iterator it = msg->tracks.begin(); it != msg->tracks.end(); it++){
            people_have.insert(it->id);
            people_data tem;
            tem.x = it->x;
            tem.y = it->y;
            tem.z = it->distance;
            tem.time_count = 1;
            tem.id = it->id;
            people_data_map[tem.id] = tem;

            if(min_z > it->distance){
                min_z = it->distance;
                min_id = it->id;
            }

        }
    }else{
        
        for(std::vector<opt_msgs::Track>::const_iterator it = msg->tracks.begin(); it != msg->tracks.end(); it++){
            current_set.insert(it->id);
            if(people_have.count(it->id) == 0){
                people_have.insert(it->id);
                people_data tem;
                tem.x = it->x;
                tem.y = it->y;
                tem.z = it->distance;
                tem.time_count = 1;
                tem.id = it->id;
                people_data_map[tem.id] = tem;
            }else{
                people_data_map[it->id].x = it->x;
                people_data_map[it->id].y = it->y;
                people_data_map[it->id].z = it->distance;
                //people_data_map[it->id].time_count += 1;
            }

            if(min_z > it->distance and (it->visibility)){
                min_z = it->distance;
                current_min_id = it->id;
            }

        }
        if (people_data_map.count(min_id) == 0){
            min_id = -1;
        }else if (people_data_map[min_id].time_count < MAX_TIME){
            min_id = current_min_id;
        }

    }

    std::cout<<min_id<<std::endl;
    if(min_id != -1 ){
        if (people_data_map[min_id].x > 0.3){
            serial_port.write( "b500f500$", 9) ;
        }else if(people_data_map[min_id].x < -0.3){
            serial_port.write( "f500b500$", 9) ;
        }else{
            if(people_data_map[min_id].z > 2.5 and people_data_map[min_id].z < 10.0){
                serial_port.write( "f300f300$", 9) ;
            }else{
                serial_port.write( "f000b000$", 9) ;
            }
        }
    }else{
        serial_port.write( "f000b000$", 9) ;
    }

    //std::map<int, people_data>::iterator pos;
    //for (pos = people_data_map.begin(); pos != people_data_map.end(); ++pos){
        //if (pos->second.time_count > MAX_TIME){
            
        //}
    //}
    for(auto pos = people_data_map.begin(); pos != people_data_map.end();){
        if(pos->second.time_count > MAX_TIME){
            people_have.erase(pos->second.id);
            pos = people_data_map.erase(pos);
        }else{
            ++pos;
        }

    }

}*/

void stm32f4_cb(const opt_msgs::TrackArray::ConstPtr& msg)
{
    laste_people.z = 100.0;

    laste_people.x = 0.0;
    bool is_active = false;

    if (pre_people.id == -1){
        for(auto it = msg->tracks.begin(); it != msg->tracks.end(); it++){
            if((laste_people.z > it->distance) and (!it->visibility)){
                laste_people.x = it->x;
                laste_people.y = it->y;
                laste_people.z = it->distance;
                pre_people.id = it->id;
                pre_people.x = it->x;
                pre_people.y = it->y;
                pre_people.z = it->distance;
            }
        }
        is_active = true;
    }else{
        for(auto it = msg->tracks.begin(); it != msg->tracks.end(); it++){
            if((laste_people.z > it->distance) and (!it->visibility)){
                laste_people.x = it->x;
                laste_people.y = it->y;
                laste_people.z = it->distance;
                //pre_people.id = it->id;
            }
            if ((pre_people.id == it->id) and (!it->visibility)){
                pre_people.x = it->x;
                pre_people.y = it->y;
                pre_people.z = it->distance;
                is_active = true;
                count_time = 0; 
            }
        }

    }
    if((is_active or (count_time < MAX_TIME)) and (pre_people.id != -1)){
        laste_people.x = pre_people.x;
        laste_people.y = pre_people.y;
        laste_people.z = pre_people.z;

    }
    if(not is_active){
         count_time++;

    }
    if(count_time >= MAX_TIME){
        pre_people.id = -1;
        count_time = 0;
    }

    
    std::cout<<"~~~~~~~~~~"<<laste_people.x<<"~~~~~~~~~~~~~~~"<<laste_people.z<<std::endl;
    if (laste_people.x > 0.4){
        serial_port.write( "b450f450$", 9) ;
    }else if(laste_people.x < -0.4){
        serial_port.write( "f450b450$", 9) ;
    }else{
        if(laste_people.z > 2.6 and laste_people.z < 10.0){
            serial_port.write( "f300f300$", 9) ;
        }else if(laste_people.z < 2.3){
            serial_port.write( "b300b300$", 9) ;
            
        }else{
            serial_port.write( "f000f000$", 9) ;
        }
    }

}



void init_serial(const char SERIAL_PORT_DEVICE[]);


int main(int argc, char** argv)
{
	pre_people.id = -1;

    ros::init(argc, argv, "stm32f4_control");
  	ros::NodeHandle nh("~");
  	ros::Subscriber input_sub = nh.subscribe("/tracker/tracks", 1, stm32f4_cb);

  	std::string usb_port;

  	nh.param("dever_name", usb_port, std::string("_"));
  	init_serial(usb_port.c_str());
  	//serial_port.write( "f500b500$", 9) ;
    //sleep(5);
    //serial_port.write( "f300f300$", 9) ;
    //sleep(5);
    //serial_port.write( "b500f500$", 9) ;
  	//std::cout<<usb_port<<std::endl;

  	ros::spin();

  	return 0;
}

void init_serial(const char SERIAL_PORT_DEVICE[])
{
	serial_port.Open( SERIAL_PORT_DEVICE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not open serial port " 
                  << SERIAL_PORT_DEVICE 
                  << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_9600 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Turn on hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() ) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }
}
