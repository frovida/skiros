#ifndef NODE_SPAWNER_H
#define NODE_SPAWNER_H
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Francesco Rovida
 *	Robotics, Vision and Machine Intelligence Laboratory
 *  Aalborg University, Denmark
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Aalborg Universitet nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/***************************************************************************
 *  Original code: rosspawn.cpp - ROSspawn main application
 *
 *  Created: Tue Aug  3 17:06:44 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Carnegie Mellon University
 *             2010  Intel Labs Pittsburgh, Intel Research
 *
 ****************************************************************************/

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <regex.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/filesystem.hpp>

#include <ros/package.h>
#include <map>
#include <string>
#include <utility>
#include <fstream>

#include "skiros_common/logger_sys.h"

namespace skiros_common
{
    /*!
     * \brief The RosNodeSpawner class allows to execute ROS nodes and handle their state
     */

    struct SpawnerProcessData
    {
        int id;
        std::string exe_name;
        /*!
          Possible status: Running, Terminated, Paging, Zombie, Sleeping, Waiting, Stopped
          */
        std::string status;
    };

    class RosNodeSpawner
    {
    public:
        RosNodeSpawner()
        {
          childrens_wait_thread_ = boost::thread(boost::bind(&RosNodeSpawner::childsMonitor, this));
        }

        ~RosNodeSpawner()
        {
            // We use native pthread calls here since Boost does not allow to cancel a thread.
            void *dont_care;
            pthread_cancel(childrens_wait_thread_.native_handle());
            pthread_join(childrens_wait_thread_.native_handle(), &dont_care);
        }

        int execute(std::string pkg_name, std::string executable, std::string params)
        {
            //If its a launch file I use a special policy
            if(executable.find(".launch")!=std::string::npos)
            {
                return launch(pkg_name, executable, params);
            }
            std::string path;
            try
            {
              path = getRosPkgExecutableFolder(pkg_name, executable);
            }
            catch (ros::Exception &e)
            {
              FERROR("[RosNodeSpawner::execute]Error starting " << pkg_name.c_str() << ": "<< e.what());
              return -1;
            }

            boost::filesystem::path temp(path.c_str());

            //check the presence of executable
            /*struct dirent *dirpent;
            DIR * dirp;
            dirp = opendir(path.c_str());
            //std::cout << path << ": " << std::endl;
            bool found = false;
            if(dirp) //TODO: extend dirpent with subdirectories
            {
                //List the files found in the directory
                while((dirpent=readdir(dirp)) != NULL)
                {
                      //std::cout << dirpent->d_name << std::endl;
                      if(std::string(dirpent->d_name) == executable)found = true;
                }
                closedir(dirp);
            }*/

            if(!this->findExecutable(temp, executable))
            {
                FERROR("[RosNodeSpawner::execute]Executable not found at: " << temp << ".");
                return -1;
            }
            pid_t pid = fork();
            if (pid == -1)
            {
                //Fork failed
                return -1;
            }
            else if (pid == 0)
            {
                // child
                path = temp.c_str();
                path += "/"+executable;
                setsid();
                signal(SIGINT, SIG_IGN);
                FINFO("[RosNodeSpawner::execute]Running " << pkg_name.c_str() << " from path " << path.c_str());
                std::stringstream ss(params);
                std::vector<char*> v;
                v.push_back(const_cast<char*>(executable.c_str()));
                while(!ss.eof())
                {
                    std::string temp;
                    ss >> temp;
                    if(temp!="")
                    {
                        FINFO(temp);
                        v.push_back(const_cast<char*>(temp.c_str()));
                    }
                }
                v.push_back(0);
                //execute
                //char * cmd[] = { executable.c_str() , NULL };
                //fclose(stdout);
                //fclose(stdin);
                //fclose(stderr);
                execv (path.c_str(), &v[0]);
                FERROR("[RosNodeSpawner::execute] If you read this, something went wrong.");
                exit(-1);
            }
            else
            {
                // parent
                FDEBUG("[RosNodeSpawner::execute] Child PID " << pid);
                boost::mutex::scoped_lock lock(childrens_mutex_);
                //Note: if the pid is not present the function automatically creates it
                childrens_[pid] = std::pair<std::string, std::string> (executable, path);
                //FINDO("[RosNodeSpawner::execute] Notify all.");
                childrens_cond_.notify_all();
                //Notify to eventual external monitors
                SpawnerProcessData temp;
                temp.id = pid;
                temp.exe_name = executable;
                temp.status = "Running";
                boost::mutex::scoped_lock lock2(event_mutex_);
                event_list_.push_back(temp);
                return pid;
            }
        }

        bool shutdown(int process_pid)
        {
            pid_t pid = (pid_t)process_pid;
            if (childrens_.find(pid) != childrens_.end())
            {
              std::string state = getProcessState(pid);
              ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(SIGINT), SIGINT,
                   childrens_[pid].first.c_str(), pid);
              ::kill(pid, SIGINT);

              sleep(0.5);
              state = getProcessState(pid);

              if (state == "T")
              {
                ROS_INFO("Process %s (PID %i) was stopped, sending %s (%i)",
                     childrens_[pid].first.c_str(), pid, strsignal(SIGCONT), SIGCONT);
                ::kill(pid, SIGCONT);
              }
              /*if (state == "Z")
              {
                  FINFO("[RosNodeSpawner::shutdown] Process " << pid << " was already closed. Removing.");
                  childrens_.erase(pid);
              }*/
              return true;
            }
            else return false;
        }

        void childsMonitor()
        {
            SpawnerProcessData pd;
            while (ros::ok())
            {
              boost::unique_lock<boost::mutex> lock(childrens_mutex_);
              boost::unique_lock<boost::mutex> lock2(event_mutex_);
              lock2.unlock();
              while (childrens_.empty()) {
                    childrens_cond_.wait(lock);
              }

              int status = 0;
              lock.unlock();
              pid_t pid = waitpid(-1, &status,  WUNTRACED | WCONTINUED); //This function blocks until a child process change state
              if (pid == -1)  continue;
              lock.lock();

              //rosspawn::NodeEvent msg;
              //msg.event_type = rosspawn::NodeEvent::NODE_DIED;
              std::string node_name = childrens_[pid].first;
              std::string message;

              if (WIFEXITED(status))
              {
                //ROS_WARN("%i/%s exited, status=%d", pid, childrens_[pid].first.c_str(), WEXITSTATUS(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) exited, status=%d",
                         childrens_[pid].first.c_str(), pid, WEXITSTATUS(status)) != -1)
                {
                  message = tmp;
                  free(tmp);
                }
              }
              else if (WIFSIGNALED(status))
              {
                //ROS_WARN("%i/%s killed by signal %d", pid, childrens_[pid].first.c_str(), WTERMSIG(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) killed by signal %d", childrens_[pid].first.c_str(), pid, WTERMSIG(status)) != -1)
                {
                    message = tmp;
                    free(tmp);
                }
              }
              else if (WIFSTOPPED(status))
              {
                //ROS_WARN("%i/%s stopped by signal %d", pid, childrens_[pid].first.c_str(), WSTOPSIG(status));
                char *tmp;
                //msg.event_type = rosspawn::NodeEvent::NODE_PAUSED;
                if (asprintf(&tmp, "%s (PID %i) stopped by signal %d", childrens_[pid].first.c_str(), pid, WSTOPSIG(status)) != -1)
                {
                    message = tmp;
                    free(tmp);
                }
              } else if (WIFCONTINUED(status))
              {
                //ROS_WARN("%i/%s continued", pid, childrens_[pid].first.c_str());
                char *tmp;
                //msg.event_type = rosspawn::NodeEvent::NODE_CONTINUED;
                if (asprintf(&tmp, "%s (PID %i) continued", childrens_[pid].first.c_str(), pid) != -1)
                {
                    message = tmp;
                    free(tmp);
                }
              }

              pd = this->getProcess(pid);

              if (WIFEXITED(status) || WIFSIGNALED(status))
              {
                if (WIFSIGNALED(status))
                {
                    int sig = WTERMSIG(status);
                    if (sig == SIGSEGV)
                    {
                      // inform about faulty program
                      ROS_WARN("Program %s (%s) died with segfault", childrens_[pid].first.c_str(), childrens_[pid].second.c_str());
                      char *tmp;
                      //msg.event_type |= rosspawn::NodeEvent::NODE_SEGFAULT;
                      if (asprintf(&tmp, "%s (PID %i) died with segfault", childrens_[pid].first.c_str(), pid) != -1)
                      {
                        message = tmp;
                        free(tmp);
                      }
                    }
                }
                pd.status = "Terminated";
                childrens_.erase(pid);
              }
              //Notify to eventual external monitors
              lock2.lock();
              event_list_.push_back(pd);
              lock2.unlock();
              FWARN("[RosNodeSpawner::childsMonitor]" << message);
              //__pub_node_events.publish(msg); TODO: publish to ROS
            }
        }

        inline bool hasEvent() {return (bool)event_list_.size();}

        SpawnerProcessData getEvent()
        {
            boost::mutex::scoped_lock lock(event_mutex_);
            SpawnerProcessData to_ret = event_list_.front();
            event_list_.pop_front();
            return to_ret;
        }

        SpawnerProcessData getProcess(pid_t pid)
        {
            SpawnerProcessData temp;
            ChildrenMap::iterator it = childrens_.find(pid);
            if(it == childrens_.end())temp.id = -1;
            else
            {
                temp.id = it->first;
                temp.exe_name = it->second.first;
                temp.status = this->getProcessState(it->first);
                temp.status = statusToString(temp.status[0]);
            }
            return temp;
        }

        pid_t getPid(std::string &node_file_name)
        {
            boost::mutex::scoped_lock lock(childrens_mutex_);
            for (ChildrenMap::iterator i = childrens_.begin(); i != childrens_.end(); ++i)
            {
              if (i->second.first == node_file_name)
              {
                return i->first;
              }
            }
            return 0;
        }

        bool sendSignal(std::string &node_file_name, int signum)
        {
            pid_t pid = getPid(node_file_name);
            if (pid != 0) {
              ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(signum), signum,
                   childrens_[pid].first.c_str(), pid);
              ::kill(pid, signum);
              return true;
            } else {
              return false;
            }
        }

        std::vector<SpawnerProcessData> listLoaded()
        {
            std::vector<SpawnerProcessData> to_ret;
            SpawnerProcessData temp;
            for (ChildrenMap::iterator it = childrens_.begin(); it != childrens_.end(); ++it)
            {
                temp.id = it->first;
                temp.exe_name = it->second.first;
                temp.status = this->getProcessState(it->first);
                temp.status = statusToString(temp.status[0]);
                to_ret.push_back(temp);
            }
            return to_ret;
        }

        /*  bool pause_node(rosspawn::NodeAction::Request &req,
          rosspawn::NodeAction::Response &resp)
            {
            return sendSignal(req.node_file_name, SIGSTOP);
            }

            bool continue_node(rosspawn::NodeAction::Request &req,
                     rosspawn::NodeAction::Response &resp)
            {
            return sendSignal(req.node_file_name, SIGCONT);
            }*/


    private:

        //Scans all subfolders looking for the executable
        bool findExecutable(boost::filesystem::path & path, std::string executable)
        {
            try
            {
              if (boost::filesystem::exists(path))    // does p actually exist?
              {
                  //std::cout << path << " is a directory containing:\n";
                  //Get list
                  typedef std::vector<boost::filesystem::path> vec;
                  vec v;
                  std::copy(boost::filesystem::directory_iterator(path),
                                           boost::filesystem::directory_iterator(),
                                           std::back_inserter(v));
                  //Scan list
                  for(vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
                  {
                      //std::cout << it->filename().string() << std::endl;
                      //If the file is my executable return
                      if(it->filename().string()==executable)
                      {
                          return true;
                      }
                      //If the file is a directory goes in the subdirectory
                      if(boost::filesystem::is_directory(*it))
                      {
                          boost::filesystem::path temp = *it;
                          if(findExecutable(temp, executable))
                          {
                              path = temp;
                              return true;
                          }
                      }
                  }
              }
              else return false;
            }
            catch (const boost::filesystem::filesystem_error& ex)
            {
              FERROR("[findExecutable]" << ex.what());
              return false;
            }
        }

        std::string getProcessState(pid_t pid)
        {
            char *procpath;
            if (asprintf(&procpath, "/proc/%i/stat", pid) != -1)
            {
                FILE *f = fopen(procpath, "r");
                if (f)
                {
                    int pid;char *program; char state[2]; state[1] = 0;
                    /*state     One  character  from  the string "RSDZTW" where R is
                                              running, S is sleeping in an interruptible  wait,  D
                                              is  waiting  in  uninterruptible  disk  sleep,  Z is
                                              zombie, T is traced or stopped (on a signal), and  W
                                              is paging.*/
                    if (fscanf(f, "%d %s %c", &pid, program, state) == 3)
                    {
                        free(program);fclose(f);free(procpath);
                        return state;
                    }
                    fclose(f);
                }
                free(procpath);
            }
            return "?";
        }

        std::string statusToString(char status)
        {
            std::string to_ret;
            switch(status)
            {
            case 'S':
                to_ret = "Sleeping";
                break;
            case 'D':
                to_ret = "Waiting";
                break;
            case 'R':
                to_ret = "Running";
                break;
            case 'T':
                to_ret = "Stopped";
                break;
            case 'W':
                to_ret = "Paging";
                break;
            case 'Z':
                to_ret = "Zombie";
                break;
            default:
                to_ret = "Undefined";
                break;
            }
            return to_ret;
        }

        int launch(std::string pkg_name, std::string executable, std::string params)
        {
            pid_t pid = fork();
            if (pid == -1)
            {
                //Fork failed
                return -1;
            }
            else if (pid == 0)
            {
                // child
                setsid();
                signal(SIGINT, SIG_IGN);
                FINFO("[RosNodeSpawner::execute]Launching " << executable << " from package " <<  pkg_name);
                //Get roslaunch executable
                std::string path = ros::package::getPath("roslaunch");
                size_t found;
                std::string root = "share";
                std::string location = "bin/roslaunch";
                found=path.rfind(root);
                path.replace (path.begin()+found, path.end(),location);

                std::stringstream ss(params);
                std::vector<char*> v;
                v.push_back(const_cast<char*>("roslaunch"));
                v.push_back(const_cast<char*>(pkg_name.c_str()));
                v.push_back(const_cast<char*>(executable.c_str()));
                while(!ss.eof())
                {
                    std::string temp;
                    ss >> temp;
                    if(temp!="")
                    {
                        std::cout << temp << std::endl;
                        v.push_back(const_cast<char*>(temp.c_str()));
                    }
                }
                v.push_back(0);
                //execute
                //char * cmd[] = { executable.c_str() , NULL };
                //fclose(stdout);
                //fclose(stdin);
                //fclose(stderr);
                execv (path.c_str(), &v[0]);
                FERROR("[RosNodeSpawner::launch] If you read this, something went wrong.");
                exit(-1);
            }
            else
            {
                // parent
                FDEBUG("[RosNodeSpawner::execute] Child PID " << pid);
                boost::mutex::scoped_lock lock(childrens_mutex_);
                //Note: if the pid is not present the function automatically creates it
                childrens_[pid] = std::pair<std::string, std::string> (executable, pkg_name);
                childrens_cond_.notify_all();
                return pid;
            }
        }

        std::string getRosPkgExecutableFolder(std::string pkg, std::string executable)
        {
            std::string path = ros::package::getPath(pkg);
            if(path=="")
            {
              throw ros::Exception("Package not found.");
              return path;
            }
            //If its a python executable I return immediately
            if(executable.find(".py")!=std::string::npos) return path;
            //Chop down to the /src or /share root
            size_t found;
            std::string root = "src";
            std::string location = "devel/lib/"+pkg;
            found=path.rfind(root);
            if (found==std::string::npos)
            {
              location = "lib/"+pkg;
              root = "share";
              found=path.rfind(root);
              if (found!=std::string::npos) throw ros::Exception("Package not in a ROS path: " + path);
            }
            path.replace (path.begin()+found, path.end(),location);
            return path;
        }

        boost::mutex               event_mutex_;
        std::list<SpawnerProcessData> event_list_;

        typedef std::map<int, std::pair<std::string, std::string> > ChildrenMap;
        ChildrenMap                childrens_;
        boost::mutex               childrens_mutex_;
        boost::condition_variable  childrens_cond_;
        boost::thread              childrens_wait_thread_;
    };
}

#endif //NODE_SPAWNER_H
