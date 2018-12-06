/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_YAML_READER_H_INCLUDED
#define CNOID_UTIL_YAML_READER_H_INCLUDED

#include "ValueTree.h"
#include "exportdecl.h"

namespace cnoid {

class YAMLReaderImpl;

class CNOID_EXPORT YAMLReader
{
    class MappingFactoryBase {
    public:
        virtual Mapping* create(int line, int column) = 0;
        virtual ~MappingFactoryBase() { }
    };

    template <class MappingType> class MappingFactory : public MappingFactoryBase {
    public:
        virtual Mapping* create(int line, int column) { return new MappingType(line, column); }
    };
        
public:

    YAMLReader();
    ~YAMLReader();

    template <class TMapping> inline void setMappingClass() {
        setMappingFactory(new MappingFactory<TMapping>());
    }
        
    void expectRegularMultiListing();
#ifdef CNOID_BACKWARD_COMPATIBILITY
    void expectRegularMultiSequence() { expectRegularMultiListing(); }
    bool load_string(const std::string& yamlstring) { return parse(yamlstring); }
#endif
        
    bool load(const std::string& filename);
    bool parse(const std::string& yamlstring);

    ValueNode* loadDocument(const std::string& filename);

    int numDocuments();
    ValueNode* document(int index = 0);

    void clearDocuments();

    const std::string& errorMessage();

private:

    friend class YAMLReaderImpl;
        
    YAMLReaderImpl* impl;

    void setMappingFactory(MappingFactoryBase* factory);
};

#ifdef CNOID_BACKWARD_COMPATIBILITY
typedef YAMLReader YamlReader;
#endif
    
}

#endif
