#ifndef ONTOLOGY_INTERFACE_H
#define ONTOLOGY_INTERFACE_H

#include <string>
#include <vector>

namespace skiros_wm
{
/*!
 * \brief Base class to interface with ontology
 */
class BaseOntologyInterface
{
    std::string default_prefix_;
public:
    BaseOntologyInterface() {}
    virtual ~BaseOntologyInterface() {}
    //----------- Basic query ------------------
    /*!
     * \brief Direct query to ontology
     * \param query_string a query string in SPARQL format (https://en.wikipedia.org/wiki/SPARQL)
     * \param cut_prefix if true remove the prefix from the results URIs
     * \return a string with the results separated by an endline
     */
    virtual std::string queryOntology(std::string query_string, bool cut_prefix=true) = 0;
    //----------- Advanced methods ------------------
    /*!
     * \brief
     * \param uri an URI of a resource
     * \return the resource type
     */
    virtual std::string getType(std::string uri) = 0;
    /*!
     * \brief Set the prefix that will be added automatically to URIs without prefix
     * \param prefix the desired prefix (in short format ending with : or long format ending with #)
     */
    void setDefaultPrefix(std::string prefix) { default_prefix_ = prefix; }
    /*!
     * \brief Add the default prefix to URI, only if it doesn t have already
     * \param uri an uri, with or without prefix
     * \return uri with prefix
     */
    std::string addPrefix(std::string uri)
    {
        if(uri.find(":")==std::string::npos && uri.find("#")==std::string::npos)
            return default_prefix_+uri;
        else return uri;
    }
};
}
#endif // ONTOLOGY_INTERFACE_H
