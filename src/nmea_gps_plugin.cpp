/**
 * @file nmea_gps_plugin.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of NMEA GPS Plugin
 * @version 0.1
 * @date 2019-06-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <nmea_gps_plugin/nmea_gps_plugin.h>

namespace gazebo
{
    NmeaGpsPlugin::NmeaGpsPlugin()
    {

    }

    NmeaGpsPlugin::~NmeaGpsPlugin()
    {

    }

    void NmeaGpsPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        model_ptr_ = model;
        world_ptr_ = model_ptr_->GetWorld();
        if (!sdf->HasElement("robotNamespace"))
        {
            namespace_.clear();
        }
        else
        {
            namespace_ = sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
        }
        if (!sdf->HasElement("bodyName"))
        {
            link_ptr_ = model_ptr_->GetLink();
            link_name_ = link_ptr_->GetName();
        }
        else
        {
            link_name_ = sdf->GetElement("bodyName")->GetValue()->GetAsString();
            link_ptr_ = model_ptr_->GetLink(link_name_);
        }
        reference_longitude_ = nmea_gps_plugin::default_param::reference_longitude;
        reference_latitude_ = nmea_gps_plugin::default_param::reference_latitude;
        reference_altitude_ = nmea_gps_plugin::default_param::reference_altitude;
        reference_heading_ = nmea_gps_plugin::default_param::reference_heading/180*M_PI;
        nmea_topic_ = nmea_gps_plugin::default_param::nmea_topic;
        publish_rate_ = nmea_gps_plugin::default_param::publish_rate;
        position_gaussiaa_noise_ = nmea_gps_plugin::default_param::position_gaussiaa_noise;
        orientation_gaussian_noise_ = nmea_gps_plugin::default_param::orientation_gaussian_noise;
        velocity_gaussian_noise_ = nmea_gps_plugin::default_param::velocity_gaussian_noise;
        hdt_use_internalcompass_ = nmea_gps_plugin::default_param::hdt_use_compass;
        if (sdf->HasElement("frameId"))
        {
            frame_id_ = sdf->GetElement("frameId")->GetValue()->GetAsString();
        }
        if (sdf->HasElement("topicName"))
        {
            nmea_topic_ = sdf->GetElement("topicName")->GetValue()->GetAsString();
        }
        if (sdf->HasElement("publishRate"))
        {
            sdf->GetElement("publishRate")->GetValue()->Get(publish_rate_);
        }
        if (sdf->HasElement("referenceLatitude"))
        {
            sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);
        }
        if (sdf->HasElement("referenceLongitude"))
        {
            sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
        }
        if (sdf->HasElement("referenceHeading"))
        {
            if (sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
            {
                reference_heading_ = reference_heading_*M_PI/180.0;
            }
        }
        if (sdf->HasElement("referenceAltitude"))
        {
            sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
        }
        if (sdf->HasElement("positionGaussiaNoise"))
        {
            sdf->GetElement("positionGaussiaNoise")->GetValue()->Get(position_gaussiaa_noise_);
        }
        if (sdf->HasElement("orientationGaussiaNoise"))
        {
            sdf->GetElement("orientationGaussiaNoise")->GetValue()->Get(orientation_gaussian_noise_);
        }
        if (sdf->HasElement("velocityGaussiaNoise"))
        {
            sdf->GetElement("velocityGaussiaNoise")->GetValue()->Get(velocity_gaussian_noise_);
        }
        if (sdf->HasElement("HDTcompass"))
        {
            sdf->GetElement("HDTcompass")->GetValue()->Get(hdt_use_internalcompass_);
        }
        std::unique_ptr<GpsSensorModel> sensor_model_ptr(new GpsSensorModel(position_gaussiaa_noise_,orientation_gaussian_noise_,velocity_gaussian_noise_));
        sensor_model_ptr_ = std::move(sensor_model_ptr);
        node_handle_ = ros::NodeHandle(namespace_);
        nmea_pub_ = node_handle_.advertise<nmea_msgs::Sentence>(nmea_topic_,10);
        initial_pose_.position.longitude = reference_longitude_;
        initial_pose_.position.latitude = reference_latitude_;
        initial_pose_.position.altitude = reference_altitude_;
        geometry_msgs::Vector3 vec;
        vec.x = 0.0;
        vec.y = 0.0;
        vec.z = reference_heading_;
        initial_pose_.orientation = quaternion_operation::convertEulerAngleToQuaternion(vec);
        initial_utm_pose_ = geodesy::UTMPose(initial_pose_);
        update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&NmeaGpsPlugin::Update, this));
        return;
    }

    std::string NmeaGpsPlugin::getCheckSum(const std::string sentence)
    {
      uint checksum = 0;
      for(auto s : sentence)
        checksum ^= static_cast<uint>(s);
      std::stringstream ss;
      ss << std::uppercase << std::setfill ('0') << std::setw(2) << std::hex << checksum;
      return "*" + ss.str();
    }

    void NmeaGpsPlugin::Reset()
    {
        return;
    }


    double NmeaGpsPlugin::correctPISegment(double in)
    {
        if (in > 360)
        {
            return correctPISegment(in - 360);
        }
        if (in < 0)
        {
            return correctPISegment(in + 360);
        }
        return in;
    }


    nmea_msgs::Sentence NmeaGpsPlugin::getGPGST(ros::Time stamp)
    {
        nmea_msgs::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = stamp;
        sentence.sentence = "GPGST," + getUnixTime(stamp) + ",35,0.061,0.025,150,0.022,0.015,0.030";
        sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
        sentence.sentence = "$" + sentence.sentence;
        return sentence;
    }


    nmea_msgs::Sentence NmeaGpsPlugin::getGPGGA(ros::Time stamp)
    {
        nmea_msgs::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = stamp;
        sentence.sentence = "GPGGA," + getUnixTime(stamp) + ",";
        double lat = current_geo_pose_.position.latitude;
        std::string north_or_south;
        if(lat >= 0.0)
        {
            north_or_south = "N";
        }
        else
        {
            north_or_south = "S";
        }
        sentence.sentence = sentence.sentence + convertToDmm(lat,2) + "," + north_or_south + ",";
        double lon = current_geo_pose_.position.longitude;
        std::string east_or_west;
        if(lon >= 0.0)
        {
            east_or_west = "E";
        }
        else
        {
            east_or_west = "W";
        }
        sentence.sentence = sentence.sentence + convertToDmm(lon,3) + "," + east_or_west + ",4,08,1.0,";
        sentence.sentence = sentence.sentence + std::to_string(current_geo_pose_.position.altitude) + ",M,";
        sentence.sentence = sentence.sentence + std::to_string(current_geo_pose_.position.altitude) + ",M,,0000";
        sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
        sentence.sentence = "$" + sentence.sentence;
        return sentence;
    }

    nmea_msgs::Sentence NmeaGpsPlugin::getGPRMC(ros::Time stamp)
    {
        nmea_msgs::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = stamp;
        sentence.sentence = "GPRMC," + getUnixTime(stamp) + ",A,";
        double lat = current_geo_pose_.position.latitude;
        std::string north_or_south;
        if(lat >= 0.0)
        {
            north_or_south = "N";
        }
        else
        {
            north_or_south = "S";
        }
        sentence.sentence = sentence.sentence + convertToDmm(lat,2) + "," + north_or_south + ",";
        double lon = current_geo_pose_.position.longitude;
        std::string east_or_west;
        if(lon >= 0.0)
        {
            east_or_west = "E";
        }
        else
        {
            east_or_west = "W";
        }
        sentence.sentence = sentence.sentence + convertToDmm(lon,3) + "," + east_or_west + ",";
        double vel = std::sqrt(std::pow(current_twist_.linear.x,2)+std::pow(current_twist_.linear.y,2)) * 1.94384; //[knot]
        if (vel > 0.1*1.94384)
        {
            sentence.sentence = sentence.sentence + std::to_string(vel) + ",";
            double angle = -1*std::atan2(current_twist_.linear.y,current_twist_.linear.x)*(360/(2*M_PI));
            angle = correctPISegment(angle);
            sentence.sentence = sentence.sentence + std::to_string(angle) + ",";
        }
        else
        {
            sentence.sentence = sentence.sentence + "0.0,"; // Speed
            sentence.sentence = sentence.sentence + ","; // Angle
        }
        sentence.sentence = sentence.sentence + getUnixDay(stamp) + ",,,";
        sentence.sentence = sentence.sentence + "D";
        sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
        sentence.sentence = "$" + sentence.sentence;
        return sentence;
    }

    nmea_msgs::Sentence NmeaGpsPlugin::getGPVTG(ros::Time stamp)
    {
        nmea_msgs::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = stamp;
        sentence.sentence = "GPVTG,";
        double vel_kmph = std::sqrt(std::pow(current_twist_.linear.x,2)+std::pow(current_twist_.linear.y,2)) * 3.6; //[km/h]
        if (vel_kmph < 0.36)
        {
            sentence.sentence = sentence.sentence + ",T,,M,0.00,N,0.00,K,D";
            sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
            sentence.sentence = "$" + sentence.sentence;
            return sentence;
        }
        double angle = -1*std::atan2(current_twist_.linear.y,current_twist_.linear.x)*(360/(2*M_PI));
        angle = correctPISegment(angle);
        std::stringstream anglestream;
        anglestream << std::fixed << std::setprecision(2) << angle;
        sentence.sentence = sentence.sentence + anglestream.str() + ",T,,M,";
        double vel_knot = std::sqrt(std::pow(current_twist_.linear.x,2)+std::pow(current_twist_.linear.y,2)) * 1.94384; //[knot]
        sentence.sentence = sentence.sentence + std::to_string(vel_knot) + ",N,";
        sentence.sentence = sentence.sentence + std::to_string(vel_kmph) + ",K,";
        sentence.sentence = sentence.sentence + ",D";
        sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
        sentence.sentence = "$" + sentence.sentence;
        return sentence;
    }

    nmea_msgs::Sentence NmeaGpsPlugin::getGPHDT(ros::Time stamp)
    {
        nmea_msgs::Sentence sentence;
        sentence.header.frame_id = frame_id_;
        sentence.header.stamp = stamp;
        sentence.sentence = "GPHDT,";
        geometry_msgs::Vector3 vec = quaternion_operation::convertQuaternionToEulerAngle(current_geo_pose_.orientation);
        double angle = 90 + -1*vec.z/M_PI*180;
        if (!hdt_use_internalcompass_) // No internal compass, emulate just from movement
        {
            double vel = std::sqrt(std::pow(current_twist_.linear.x,2)+std::pow(current_twist_.linear.y,2));
            if (vel < 0.1)
            {
                sentence.sentence = sentence.sentence + ",T";
                sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
                sentence.sentence = "$" + sentence.sentence;
                return sentence;
            }
            angle = -1*std::atan2(current_twist_.linear.y,current_twist_.linear.x); // Radians
            angle = angle*(360/(2*M_PI));
        }
        angle = correctPISegment(angle);
        sentence.sentence = sentence.sentence + std::to_string(angle) + ",T";
        sentence.sentence = sentence.sentence + getCheckSum(sentence.sentence);
        sentence.sentence = "$" + sentence.sentence;
        return sentence;
    }

    std::string NmeaGpsPlugin::getUnixDay(ros::Time stamp)
    {
        std::string ret;
        time_t t = stamp.sec;
        struct tm *utc_time;
        utc_time = gmtime(&t);
        int day = utc_time->tm_mday;
        int month = utc_time->tm_mon;
        int year = 1900 + utc_time->tm_year;
        std::string year_str = std::to_string(year);
        ret = std::to_string(day) + std::to_string(month) + year_str[2] + year_str[3];
        return ret;
    }

    std::string NmeaGpsPlugin::getUnixTime(ros::Time stamp)
    {
        std::string ret;
        time_t t = stamp.sec;
        struct tm *utc_time;
        utc_time = gmtime(&t);
        int hour = utc_time->tm_hour;
        int min = utc_time->tm_min;
        int sec = utc_time->tm_sec;
        uint32_t nsec = stamp.nsec;
        int csec = round((double)nsec/std::pow(10,7));
        std::string hour_str;
        if(hour<9)
        {
            hour_str = "0" + std::to_string(hour);
        }
        else
        {
            hour_str = std::to_string(hour);
        }
        std::string min_str;
        if(min<=9)
        {
            min_str = "0" + std::to_string(min);
        }
        else
        {
            min_str = std::to_string(min);
        }
        std::string sec_str;
        if(sec<=9)
        {
            sec_str = "0" + std::to_string(sec);
        }
        else
        {
            sec_str = std::to_string(sec);
        }
        ret = hour_str + min_str + sec_str + "." + std::to_string(csec);
        return ret;
    }

    void NmeaGpsPlugin::Update()
    {
#if (GAZEBO_MAJOR_VERSION >= 8)
        common::Time sim_time = world_ptr_->SimTime();
#else
        common::Time sim_time = world_ptr_->GetSimTime();
#endif
        bool publish;
        double step = world_ptr_->Physics()->GetMaxStepSize(); double fraction = fmod(sim_time.Double() + (step / 2.0), 1.0/publish_rate_); if (!  ((fraction >= 0.0) && (fraction < step)) )  {  return; }
if(!last_publish_timestamp_ || sim_time-(*last_publish_timestamp_) > common::Time(1.0/publish_rate_))
        {
            last_publish_timestamp_ = sim_time;
            publish = true;
        }
        if(!publish)
        {
            return;
        }
#if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Pose3d pose = link_ptr_->WorldPose();
        ignition::math::Vector3d linear_velocity = link_ptr_->WorldLinearVel();
        current_twist_.linear.x = linear_velocity.X();
        current_twist_.linear.y = linear_velocity.Y();
        current_twist_.linear.z = linear_velocity.Z();
#else
        gazebo::math::Pose pose = link_ptr_->GetWorldPose();
        gazebo::math::Vector3 linear_velocity = link_ptr_->GetWorldLinearVel();
        current_twist_.linear.x = linear_velocity.x;
        current_twist_.linear.y = linear_velocity.y;
        current_twist_.linear.z = linear_velocity.z;
#endif
        if(!initial_gazebo_pose_)
        {
            initial_gazebo_pose_ = pose;
        }
        current_twist_ = sensor_model_ptr_->addGaussianNoise(current_twist_);
        ros::Time stamp;
        stamp.sec = sim_time.sec;
        stamp.nsec = sim_time.nsec;
        geodesy::UTMPoint current_utm_point;
        geometry_msgs::Quaternion current_utm_quat;
#if (GAZEBO_MAJOR_VERSION >= 8)
        current_utm_quat.x = pose.Rot().X();
        current_utm_quat.y = pose.Rot().Y();
        current_utm_quat.z = pose.Rot().Z();
        current_utm_quat.w = pose.Rot().W();
        geometry_msgs::Vector3 current_utm_orientation 
            = quaternion_operation::convertQuaternionToEulerAngle(current_utm_quat);
        current_utm_orientation.z = current_utm_orientation.z + reference_heading_;
        current_utm_quat = quaternion_operation::convertEulerAngleToQuaternion(current_utm_orientation);
        double diff_x = pose.Pos().X() - initial_gazebo_pose_->Pos().X();
        double diff_y = pose.Pos().Y() - initial_gazebo_pose_->Pos().Y();
        double r = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        double theta = std::atan2(diff_y,diff_x) + reference_heading_;
        current_utm_point.northing = initial_utm_pose_.position.northing + r*std::cos(theta);
        current_utm_point.easting = initial_utm_pose_.position.easting - r*std::sin(theta);
        current_utm_point.altitude = pose.Pos().Z() + initial_utm_pose_.position.altitude;
#else
        current_utm_quat.x = pose.rot.x;
        current_utm_quat.y = pose.rot.y;
        current_utm_quat.z = pose.rot.z;
        current_utm_quat.w = pose.rot.w;
        geometry_msgs::Vector3 current_utm_orientation = quaternion_operation::convertQuaternionToEulerAngle(current_utm_quat);
        current_utm_orientation.z = current_utm_orientation.z + reference_heading_;
        current_utm_quat = quaternion_operation::convertEulerAngleToQuaternion(current_utm_orientation);
        double diff_x = pose.pos.x - initial_gazebo_pose_->pos.x;
        double diff_y = pose.pos.y - initial_gazebo_pose_->pos.y;
        double r = std::sqrt(diff_x*diff_x + diff_y*diff_y);
        double theta = std::atan2(diff_y,diff_x) + reference_heading_;
        current_utm_point.northing = initial_utm_pose_.position.northing + r*std::cos(theta);
        current_utm_point.easting = initial_utm_pose_.position.easting - r*std::sin(theta);
        current_utm_point.altitude = pose.pos.z + initial_utm_pose_.position.altitude;
#endif
        current_utm_point.zone = initial_utm_pose_.position.zone;
        current_utm_point.band = initial_utm_pose_.position.band;
        current_utm_point = sensor_model_ptr_->addGaussianNoise(current_utm_point);
        current_utm_quat = sensor_model_ptr_->addGaussianNoise(current_utm_quat);
        current_geo_pose_.position = geodesy::toMsg(current_utm_point);
        current_geo_pose_.orientation = current_utm_quat;
        nmea_pub_.publish(getGPRMC(stamp));
        nmea_pub_.publish(getGPGGA(stamp));
        nmea_pub_.publish(getGPVTG(stamp));
        nmea_pub_.publish(getGPHDT(stamp));
        nmea_pub_.publish(getGPGST(stamp));
        return;
    }

    std::string NmeaGpsPlugin::convertToDmm(double value, int width)
    {
        std::string ret;
        value = std::fabs(value);
        int deg = std::floor(value);
        std::stringstream ss, ss2;
        ss2 <<  std::setfill ('0') << std::setw(width) << deg;
        ss << std::setprecision(7) << (value-(double)deg)*60.0;
        ret = ss2.str() + ss.str();
        return ret;
    }

    GZ_REGISTER_MODEL_PLUGIN(NmeaGpsPlugin)
}
