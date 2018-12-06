/*!
  @file
  @author Shizuko Hattori, Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyLoader>
#include <cnoid/BodyItem>
#include <iostream>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <sdf/sdf.hh>
#include "SDFBodyLoader.h"
#include <OGRE/OgreRoot.h>

using namespace std;

using namespace cnoid;

namespace {
    
bool loadBodyItem(BodyItem* item, const std::string& filename)
{
    SDFBodyLoader loader;
    return loader.load(item, filename);

}
    
AbstractBodyLoaderPtr sdfBodyLoaderFactory()
{
    return make_shared<SDFBodyLoader>();
}

class SDFPlugin : public Plugin
{
public:
    SDFPlugin() : Plugin("SDF") {
        require("Body");
        require("Assimp");
    }
        
    virtual ~SDFPlugin() {
    }
    
    virtual bool initialize() {
        BodyLoader::registerLoader("sdf", sdfBodyLoaderFactory);
        BodyLoader::registerLoader("urdf", sdfBodyLoaderFactory);
        itemManager().addLoader<BodyItem>(
            "Gazebo Model File", "GAZEBO-MODEL", "sdf;urdf", boost::bind(loadBodyItem, _1, _2));

        addModelSearchPath("GAZEBO_MODEL_PATH");
        addModelSearchPath("ROS_PACKAGE_PATH");
        addModelSearchPath("HOME");

        ogreRoot = new Ogre::Root();

        return true;
    }
        
    virtual bool finalize() {
        delete ogreRoot;

        return true;
    }

    void addModelSearchPath(const char *envname)
    {
        std::list<std::string> paths;
        std::string path;
        char *p;

        if (envname != NULL && (p = getenv(envname)) != NULL) {
            if (envname != "HOME") {
                boost::split(paths, p, boost::is_any_of(":"));
                BOOST_FOREACH(path, paths) {
                    if (path != "") {
                        sdf::addURIPath("model://", path);
                    }
                }
            } else {
                path = p;
                sdf::addURIPath("model://", path + "/.gazebo/models");
            }
        }

        return;
    }

private:

    Ogre::Root* ogreRoot;
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SDFPlugin);
