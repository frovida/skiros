#include "skiros_common/utility.h"
#include <pwd.h>
#include <dirent.h>
#include <ros/package.h>

namespace skiros_common
{
namespace utility
{
    std::vector<skiros_common::any> deserializeAnyVector(SerialData any_ser)
    {
      std::vector<skiros_common::any> to_ret;
      boost::shared_array<uint8_t> buffer(new uint8_t[any_ser.length]);
      for(int i=0;i<any_ser.length;i++)buffer[i] = any_ser.data[i];
      ros::serialization::IStream streamIN(buffer.get(), any_ser.length);
      ros::serialization::Serializer<std::vector<skiros_common::any> >::read(streamIN, to_ret);
      return to_ret;
    }

    SerialData serializeAnyVector(std::vector<skiros_common::any> any_v)
    {
      SerialData to_ret;
      to_ret.length = ros::serialization::serializationLength(any_v);
      boost::shared_array<uint8_t> buffer(new uint8_t[to_ret.length]);
      ros::serialization::OStream stream(buffer.get(), to_ret.length);
      ros::serialization::serialize(stream, any_v);
      for(int i=0;i<to_ret.length;i++)
      {
          //ROS_INFO("%i %i", to_ret.length, i);
          to_ret.data.push_back(buffer[i]);
      }
      return to_ret;
    }

    skiros_common::ParamMap deserializeParamMap(skiros_msgs::ParamMapSerialized ph_ser)
    {
      skiros_common::ParamMap map;
      if(ph_ser.length)
      {
          boost::shared_array<uint8_t> buffer(new uint8_t[ph_ser.length]);
          for(int i=0;i<ph_ser.length;i++)buffer[i] = ph_ser.data[i];
          ros::serialization::IStream streamIN(buffer.get(), ph_ser.length);
          ros::serialization::Serializer<skiros_common::ParamMap>::read(streamIN, map);
      }
      return map;
    }

    skiros_msgs::ParamMapSerialized serializeParamMap(skiros_common::ParamMap map)
    {
      skiros_msgs::ParamMapSerialized to_ret;
      to_ret.length = ros::serialization::serializationLength(map);
      boost::shared_array<uint8_t> buffer(new uint8_t[to_ret.length]);
      ros::serialization::OStream stream(buffer.get(), to_ret.length);
      ros::serialization::serialize(stream, map);
      for(int i=0;i<to_ret.length;i++)
      {
          //ROS_INFO("%i %i", to_ret.length, i);
          to_ret.data.push_back(buffer[i]);
      }
      return to_ret;
    }

    std::string getSkirosSaveDirectory()
    {
            std::string path = ros::package::getPath("skiros");
            if(path=="") throw std::runtime_error("Package not found.");
            path = path + "/";
            /*
            //Home folder dir
            struct passwd *pw = getpwuid(getuid());
            const char *homedir = pw->pw_dir;
            std::string path(homedir);
            path = path + "/skiros/";*/

            /*Find proper place to create skiros workspace folders
            int rc = mkdir(path.c_str(), 0777);
            if (rc != 0)
            {
                if(errno != 17)
                  {
                    FERROR("Problem while creating the skiros workspace in folder: " << path);
                  }
            }*/

            //get the full path of the executable
            /*char buff[1024];
            ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
            std::string path = std::string(buff);

            //Chop down to path to "MobileManipulator" and add the supplied location
            size_t found;
            //TODO: make the log folder configurable
            std::string key = "ROS_WS";

            found=path.rfind(key);
            std::string location = "/logs/";

            if (found!=std::string::npos) path.replace (path.begin()+found+key.size(), path.end(),location);
            */
            return path;
    }

    std::vector<std::string> getFilesInFolder(std::string folder_name, std::string filter)
    {
        struct dirent *dirpent;
        DIR * dirp;
        dirp = opendir(folder_name.c_str());
        std::vector<std::string> v;
        if(dirp)
        {
            int i = 0;
            //List the files found in the directory
            while((dirpent=readdir(dirp)) != NULL)
            {
              std::string temp(dirpent->d_name);
              if(temp.find(filter.c_str()) != std::string::npos)
              {
                  v.push_back(temp);
              }
            }
            closedir(dirp);
        }
        return v;
    }

    std::string browseFilesInFolder(std::string folder_name, std::string filter)
    {
        struct dirent *dirpent;
        DIR * dirp;
        dirp = opendir(folder_name.c_str());
        std::cout << folder_name << ": " << std::endl;
        if(dirp)
        {
            std::vector<std::string> v;int i = 0;
            //List the files found in the directory
            while((dirpent=readdir(dirp)) != NULL)
            {
              std::string temp(dirpent->d_name);
              if(temp.find(filter.c_str()) != std::string::npos)
              {
              std::cout << i++ << ": " << dirpent->d_name << std::endl;
              v.push_back(temp);
              }
            }
            closedir(dirp);
            //Wait for a selection
            while(true)
            {
              if(v.size()>1)
              {
                std::cout << "Select the file to load. (number)" << std::endl;
                std::cin >> i;
                if(i>=0 && i < v.size())
                {
                    return v[i];
                }
                else
                {
                  std::cout << "Not valid input. Try again." << std::endl;
                  std::cin.clear();
                  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
              }
              else
              {
                if(v.size()==1) return v[0];
                std::cout << "No files to load." << std::endl;
                return std::string("");
              }
            }
        }
    }

}
}
