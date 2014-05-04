#ifndef GISSUMO_H_
#define GISSUMO_H_

#include <iostream>
#include <string>
#include <pqxx/pqxx>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

bool isLineOfSight(pqxx::connection &c, float x1, float y1, float x2, float y2);

//struct fcd_export_settings
//{
//    std::string m_file;          // log filename
//    int m_fcd_export;                 // fcd-export level
//    std::set<string> m_modules;  // modules where logging is enabled
//    void load(const std::string &filename);
//};






#endif /* GISSUMO_H_ */
