/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "HrpsysFileIO.h"
#include "BodyMotionItem.h"
#include "BodyItem.h"
#include "KinematicFaultChecker.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionUtil>
#include <cnoid/ZMPSeq>
#include <QMessageBox>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/format.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#ifndef _WINDOWS
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#endif
#include <boost/filesystem.hpp>
#include <fstream>
#include <list>
#include <vector>
#include <map>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
namespace filesystem = boost::filesystem;
using boost::format;

namespace {

map<string,int> labelToTypeMap;

boost::regex labelPattern("^(JA|JV|TQ|F|M|A|W|zmp|waist|R|P|Y)([XYZRP]?)(\\d*)$");
    
class HrpsysLogLoader
{
public:

    enum { NONE = 0,
           JOINT_POS, JOINT_VEL, JOINT_TORQUE,
           FORCE, TORQUE, ACC, OMEGA,
           ZMP, WAIST, RPY,
           NUM_DATA_TYPES };
    
    typedef boost::char_separator<char> Separator;
    typedef boost::tokenizer<Separator> Tokenizer;
    
    struct Element {
        Element(){
            type = NONE;
            axis = 0;
            index = 0;
        }
        int type;
        int axis;
        int index;
    };
    
    vector<Element> elements;
    
    int numComponents[NUM_DATA_TYPES];
    
    std::list< std::vector<double> > frames;
    
    HrpsysLogLoader() {
        if(labelToTypeMap.empty()){
            labelToTypeMap["JA"] = JOINT_POS;
            labelToTypeMap["JV"] = JOINT_VEL;
            labelToTypeMap["TQ"] = JOINT_TORQUE;
            labelToTypeMap["F"] = FORCE;
            labelToTypeMap["M"] = TORQUE;
            labelToTypeMap["A"] = ACC;
            labelToTypeMap["W"] = OMEGA;
            labelToTypeMap["zmp"] = ZMP;
            labelToTypeMap["waist"] = WAIST;
            labelToTypeMap["R"] = RPY;
            labelToTypeMap["P"] = RPY;
            labelToTypeMap["Y"] = RPY;
        }
    }

    bool loadLogFile(BodyMotionItem* item, const std::string& filename, std::ostream& os){

        boost::iostreams::filtering_istream is;

#ifndef _WINDOWS
        string ext = filesystem::extension(filesystem::path(filename));
        if(ext == ".gz"){
            is.push(boost::iostreams::gzip_decompressor());
        } else if(ext == ".bz2"){
            is.push(boost::iostreams::bzip2_decompressor());
        }
#endif
        ifstream ifs(filename.c_str());

        if(!ifs){
            os << (format("\"%1%\" cannot be opened.") % filename) << endl;
            return false;
        }

        is.push(ifs);
        
        elements.clear();
        frames.clear();
        Separator sep(" \t\r\n", "%");
        string line;
        
        while(getline(is, line)){
            Tokenizer tokens(line, sep);
            Tokenizer::iterator it = tokens.begin();
            if(it != tokens.end()){
                if(*it == "%"){
                    readHeader(++it, tokens.end());
                }
                break;
            }
        }

        if(elements.empty()){
            return false;
        }

        const size_t numElements = elements.size();

        while(getline(is, line)){
            Tokenizer tokens(line, sep);
            Tokenizer::iterator it = tokens.begin();
            if(it != tokens.end()){
                frames.push_back(vector<double>(numElements));
                vector<double>& frame = frames.back();
                size_t i;
                for(i=0; (i < numElements) && (it != tokens.end()); ++i, ++it){
                    frame[i] = boost::lexical_cast<double>(*it);
                }
                if(i < numElements /* || it != tokens.end() */ ){
                    os << (format("\"%1%\" contains different size columns.") % filename) << endl;
                    return false;
                }
            }
        }

        const size_t numFrames = frames.size();

        BodyMotionPtr motion = item->motion();
        motion->setDimension(numFrames, numComponents[JOINT_POS], 0);
        motion->setFrameRate(200);

        MultiValueSeqPtr qseq = item->motion()->jointPosSeq();
        //MultiValueSeqPtr dqseq;
        //MultiValueSeqPtr useq;
        ZMPSeqPtr zmpseq = getOrCreateZMPSeq(*item->motion());

        std::list< std::vector<double> >::iterator p = frames.begin();
        
        for(size_t i=0; i < numFrames; ++i){
            vector<double>& frame = *p++;
            MultiValueSeq::Frame q = qseq->frame(i);
            for(size_t j=0; j < numElements; ++j){
                Element& e = elements[j];
                switch(e.type){
                case JOINT_POS:
                    q[e.index] = frame[j];
                    break;
                case ZMP:
                    (*zmpseq)[i][e.axis] = frame[j];
                    break;
                }
            }
        }

        return true;
    }

    void readHeader(Tokenizer::iterator it, Tokenizer::iterator end){
        
        boost::smatch match;

        for(int i=0; i < NUM_DATA_TYPES; ++i){
            numComponents[i] = 0;
        }

        int waistIndex = 0;
        
        while(it != end){

            Element element;

            if(regex_match(*it, match, labelPattern)){

                map<string,int>::iterator p = labelToTypeMap.find(match.str(1));

                if(p != labelToTypeMap.end()){

                    element.type = p->second;

                    const string& axisString = match.str(2);
                    if(!axisString.empty()){
                        if(element.type != WAIST || waistIndex < 3){
                            switch(axisString[0]){
                            case 'X': element.axis = 0; break;
                            case 'Y': element.axis = 1; break;
                            case 'Z': element.axis = 2; break;
                            }
                        } else {
                            switch(axisString[0]){
                            case 'R': element.axis = 0; break;
                            case 'P': element.axis = 1; break;
                            case 'Y': element.axis = 2; break;
                            }
                        }
                    }
                    
                    const string& indexString = match.str(3);
                    if(!indexString.empty()){
                        element.index = boost::lexical_cast<int>(indexString);
                    }
                    
                    if(element.type == WAIST){
                        waistIndex++;
                    }
                }
            }
            elements.push_back(element);
            numComponents[element.type] += 1;
            ++it;
        }
    }
};


bool confirm(const std::string& message)
{
    return (QMessageBox::warning(
                0, _("Warning"), message.c_str(),
                QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok) == QMessageBox::Ok);
}


bool loadLogFile(BodyMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem)
{
    HrpsysLogLoader loader;
    return loader.loadLogFile(item, filename, os);
}


bool importHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    if(loadHrpsysSeqFileSet(*item->motion(), filename, os)){
        return true;
    }
    return false;
}

    
bool exportHrpsysSeqFileSet(BodyMotionItem* item, const std::string& filename, std::ostream& os)
{
    double frameRate = item->motion()->frameRate();
    if(frameRate != 200.0){
        static format m1(_("The frame rate of a body motion exported as HRPSYS files should be standard value 200, "
                           "but the frame rate of \"%1%\" is %2%. The exported data may cause a problem.\n\n"
                           "Do you continue to export ?"));
        
        if(!confirm(str(m1 % item->name() % frameRate))){
            return false;
        }
    }
    
    BodyPtr body;
    BodyItem* bodyItem = item->findOwnerItem<BodyItem>();
    if(bodyItem){
        body = bodyItem->body();
        KinematicFaultChecker* checker = KinematicFaultChecker::instance();
        int numFaults = checker->checkFaults(bodyItem, item, os);
        if(numFaults > 0){
            static string m2(_("A fault has been detected. Please check the report in the MessageView.\n\n"
                               "Do you continue to export ?"));
            static format m3(_("%1% faults have been detected. Please check the report in the MessageView.\n\n"
                               "Do you continue to export ?"));
            
            bool result;
            
            if(numFaults == 1){
                result = confirm(m2);
            } else {
                result = confirm(str(m3 % numFaults));
            }
            
            if(!result){
                return false;
            }
        }
    }
    
    if(!getZMPSeq(*item->motion())){
        if(!confirm(_("There is no ZMP data. Do you continue to export ?"))){
            return false;
        }
    }
    
    return saveHrpsysSeqFileSet(*item->motion(), body, filename, os);
}

} // namespace {


void cnoid::initializeHrpsysFileIO(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    
    im.addLoaderAndSaver<BodyMotionItem>(
        _("HRPSYS Sequence File Set"), "HRPSYS-SEQ-FILE-SET", "pos;vel;acc;hip;waist;gsens;zmp",
        std::bind(importHrpsysSeqFileSet, _1, _2, _3), std::bind(exportHrpsysSeqFileSet, _1, _2, _3),
        ItemManager::PRIORITY_CONVERSION);

    im.addLoader<BodyMotionItem>(
        _("HRPSYS Log File"), "HRPSYS-LOG", "log;log.gz;log.bz2", loadLogFile, ItemManager::PRIORITY_CONVERSION);
}
