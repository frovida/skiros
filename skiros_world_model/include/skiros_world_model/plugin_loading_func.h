#ifndef PLUGIN_LOADING_FUNC_H
#define PLUGIN_LOADING_FUNC_H

#include <pluginlib/class_loader.h>

namespace skiros
{
  template<class T>
  boost::shared_ptr<T> loadPlugin(std::string package, std::string base_class, std::string plugin_name)
  {
        static pluginlib::ClassLoader<T> * loader = NULL;
        static boost::shared_ptr<T> plugin_ptr;
//        static std::string last_loaded;
//        if(last_loaded == plugin_name) return plugin_ptr;
//        last_loaded = plugin_name;
        if(!loader)loader = new pluginlib::ClassLoader<T>(package, base_class);
        plugin_ptr.reset();
        try
        {
          plugin_ptr = loader->createInstance(plugin_name);
        }
        catch(pluginlib::PluginlibException& ex)
        {
            FERROR("[skiros::loadPlugin]The plugin " << plugin_name << " failed to load. Error: " << ex.what());
        }
        return plugin_ptr;
  }

  template<class T>
  std::vector<std::string> getAvailablePlugins(std::string package, std::string base_class)
  {
      static pluginlib::ClassLoader<T> * loader = NULL;
      if(!loader)
          loader = new pluginlib::ClassLoader<T>(package, base_class);
      return loader->getDeclaredClasses();
  }
}


#endif //PLUGIN_LOADING_FUNC_H
